#include <Arduino.h>
// #include <WiFi.h>
// #include <PubSubClient.h>
// #include <ESPAsyncWebServer.h>

#include "display.hpp"
#include "scale.hpp"
#include "config.hpp"
// #include "web_server.hpp"

// Definitions of global variables (memory allocated here)
Preferences preferences;             // Preferences object
HX711 loadcell;                      // HX711 load cell object
HX711 loadcell2;                     // Optional second HX711 load cell object
// Kalman filter for weight smoothing. Tuned for more responsiveness: higher processNoise
// and lower measurementNoise compared to previous conservative defaults.
// Make Kalman more responsive to reduce settle time after events like grinding.
// Increasing processNoise makes the filter trust the model less and follow measurements faster.
SimpleKalmanFilter kalmanFilter(1.0, 0.01, 0.01); // (processNoise, measurementNoise, estimationError)

// Secondary scale factor (loaded from preferences in setupScale)
double scaleFactor2 = LOADCELL2_SCALE_FACTOR;

TaskHandle_t ScaleTask = nullptr;    // Initialize task handles to nullptr
TaskHandle_t ScaleStatusTask = nullptr;

volatile bool displayLock = false;

// External reference to scaleFactor from scale.cpp
extern double scaleFactor;

// Serial calibration state
bool calibration_mode = false;
long calibration_tare_raw = 0;
float calibration_known_weight = 0;
// Per-sensor (sensor 2) calibration state
bool calibration_mode2 = false;
long calibration_tare_raw2 = 0;
float calibration_known_weight2 = 0;

void processSerialCommands() {
    if (!Serial.available()) return;

    // Read a full line so we can support commands like t1, c2, w1 48.2, w248.2, etc.
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    char cmd = line.charAt(0);
    int sensor = 1; // default sensor 1
    // If second char is digit, treat it as sensor index (1 or 2)
    if (line.length() > 1 && isDigit(line.charAt(1))) {
        sensor = line.charAt(1) - '0';
    }

    switch(cmd) {
        case 't': {
            // Tare command - capture current raw average for selected sensor
            Serial.println("\n[CAL] Taring - please wait...");
            if (sensor == 2) {
                if (loadcell2.wait_ready_timeout(500)) {
                    calibration_tare_raw2 = loadcell2.read_average(20);
                    Serial.printf("[CAL] Sensor2 tare captured: %ld counts\n", calibration_tare_raw2);
                    Serial.println("[CAL] Ready for calibration (sensor2). Place known weight and use 'c2' command.");
                } else {
                    Serial.println("[CAL] Error: HX711(sensor2) not ready for tare");
                }
            } else {
                if (loadcell.wait_ready_timeout(500)) {
                    calibration_tare_raw = loadcell.read_average(20);
                    Serial.printf("[CAL] Sensor1 tare captured: %ld counts\n", calibration_tare_raw);
                    Serial.println("[CAL] Ready for calibration (sensor1). Place known weight and use 'c' or 'c1' command.");
                } else {
                    Serial.println("[CAL] Error: HX711(sensor1) not ready for tare");
                }
            }
            break;
        }
        case 'c': {
            // Enter calibration mode - wait for weight input for selected sensor
            if (sensor == 2) {
                if (calibration_tare_raw2 == 0) {
                    Serial.println("[CAL] Error: Tare sensor2 first with 't2' command");
                } else {
                    calibration_mode2 = true;
                    Serial.println("[CAL] Calibration mode (sensor2) - place known weight");
                    Serial.println("[CAL] Enter weight in grams, e.g., 'w2 48.1' or 'w248.1' then press enter");
                }
            } else {
                if (calibration_tare_raw == 0) {
                    Serial.println("[CAL] Error: Tare sensor1 first with 't' or 't1' command");
                } else {
                    calibration_mode = true;
                    Serial.println("[CAL] Calibration mode (sensor1) - place known weight");
                    Serial.println("[CAL] Enter weight in grams, e.g., 'w48.1' or 'w1 48.1' then press enter");
                }
            }
            break;
        }
        case 'w': {
            // Weight input for calibration; parse the numeric value from the line
            float weight = 0.0;
            String rest;
            if (sensor == 2) {
                // rest of string after optional 'w2' prefix
                if (line.length() > 2) rest = line.substring(2);
                else rest = "";
            } else {
                if (line.length() > 1 && !isDigit(line.charAt(1))) rest = line.substring(1);
                else if (line.length() > 1 && isDigit(line.charAt(1))) rest = line.substring(1); // covers w48.1 (sensor1) or w148.1 (ambiguous but treat as sensor1 with value starting at pos1)
                else rest = "";
            }
            rest.trim();
            if (rest.length() == 0) {
                // Try to parse using parseFloat from entire Serial buffer fallback (rare)
                weight = 0.0;
            } else {
                weight = rest.toFloat();
            }

            if (sensor == 2) {
                if (!calibration_mode2) {
                    Serial.println("[CAL] Error: Enter calibration mode for sensor2 first with 'c2'");
                    break;
                }

                calibration_known_weight2 = weight;
                if (calibration_known_weight2 <= 0 || calibration_known_weight2 > 1000) {
                    Serial.println("[CAL] Error: Invalid weight for sensor2. Use format: w2 48.1");
                    break;
                }

                Serial.printf("[CAL] Sensor2: Using %.2fg as reference. Waiting for stable reading...\n", calibration_known_weight2);
                if (loadcell2.wait_ready_timeout(500)) {
                    long raw_with_weight = loadcell2.read_average(20);
                    long raw_diff = raw_with_weight - calibration_tare_raw2;
                    float new_factor = (float)raw_diff / calibration_known_weight2;

                    Serial.println("\n=== Calibration Results (sensor2) ===");
                    Serial.printf("Raw tare: %ld\n", calibration_tare_raw2);
                    Serial.printf("Raw with weight: %ld\n", raw_with_weight);
                    Serial.printf("Raw difference: %ld counts\n", raw_diff);
                    Serial.printf("Known weight: %.2fg\n", calibration_known_weight2);
                    Serial.printf("Computed factor: %.2f counts/gram\n", new_factor);
                    Serial.printf("Old factor(sensor2): %.2f\n", scaleFactor2);

                    float verified_weight = (float)raw_diff / new_factor;
                    Serial.printf("Verification: %.2fg (should match %.2fg)\n", verified_weight, calibration_known_weight2);

                    // Apply new factor immediately
                    scaleFactor2 = new_factor;
                    loadcell2.set_scale(scaleFactor2);

                    // Save to preferences
                    preferences.begin("scale", false);
                    preferences.putDouble("calibration2", (double)scaleFactor2);
                    preferences.end();

                    // Block AZT briefly after calibration to avoid auto-zero fighting the new factor
                    aztBlockUntil = millis() + 10000UL; // 10 seconds

                    Serial.printf("\n[CAL] Applied new scale factor (sensor2): %.2f\n", scaleFactor2);
                    Serial.println("[CAL] Factor saved to NVS. Calibration complete for sensor2!");
                    Serial.println("========================================\n");

                    calibration_mode2 = false;
                    calibration_tare_raw2 = 0;
                    calibration_known_weight2 = 0;
                } else {
                    Serial.println("[CAL] Error: HX711(sensor2) not ready");
                }
            } else {
                if (!calibration_mode) {
                    Serial.println("[CAL] Error: Enter calibration mode first with 'c' or 'c1'");
                    break;
                }

                calibration_known_weight = weight;
                if (calibration_known_weight <= 0 || calibration_known_weight > 1000) {
                    Serial.println("[CAL] Error: Invalid weight. Use format: w48.1 or w1 48.1");
                    break;
                }

                Serial.printf("[CAL] Using %.2fg as reference. Waiting for stable reading...\n", calibration_known_weight);

                if (loadcell.wait_ready_timeout(500)) {
                    long raw_with_weight = loadcell.read_average(20);
                    long raw_diff = raw_with_weight - calibration_tare_raw;
                    float new_factor = (float)raw_diff / calibration_known_weight;

                    Serial.println("\n=== Calibration Results ===");
                    Serial.printf("Raw tare: %ld\n", calibration_tare_raw);
                    Serial.printf("Raw with weight: %ld\n", raw_with_weight);
                    Serial.printf("Raw difference: %ld counts\n", raw_diff);
                    Serial.printf("Known weight: %.2fg\n", calibration_known_weight);
                    Serial.printf("Computed factor: %.2f counts/gram\n", new_factor);
                    Serial.printf("Old factor: %.2f\n", scaleFactor);

                    // Verify calculation
                    float verified_weight = (float)raw_diff / new_factor;
                    Serial.printf("Verification: %.2fg (should match %.2fg)\n", verified_weight, calibration_known_weight);

                    // Apply new factor immediately
                    scaleFactor = new_factor;
                    loadcell.set_scale(scaleFactor);

                    // Save to preferences
                    preferences.begin("scale", false);
                    preferences.putDouble("calibration", (double)scaleFactor);
                    preferences.end();

                    // Block AZT briefly after calibration to avoid auto-zero fighting the new factor
                    aztBlockUntil = millis() + 10000UL; // 10 seconds

                    Serial.printf("\n[CAL] Applied new scale factor: %.2f\n", scaleFactor);
                    Serial.println("[CAL] Factor saved to NVS. Calibration complete!");
                    Serial.println("===========================\n");

                    calibration_mode = false;
                    calibration_tare_raw = 0;
                    calibration_known_weight = 0;
                } else {
                    Serial.println("[CAL] Error: HX711 not ready");
                }
            }
            break;
        }
        case 's': {
            // Status
            Serial.println("\n=== Scale Status ===");
            Serial.printf("Sensor1 factor: %.2f counts/gram\n", scaleFactor);
            Serial.printf("Sensor2 factor: %.2f counts/gram\n", scaleFactor2);
            Serial.printf("Total weight (smoothed): %.2fg\n", scaleWeight);
            Serial.printf("Calibration mode (s1): %s\n", calibration_mode ? "active" : "inactive");
            Serial.printf("Calibration mode (s2): %s\n", calibration_mode2 ? "active" : "inactive");
            Serial.printf("Tare captured (s1): %s\n", calibration_tare_raw != 0 ? "yes" : "no");
            Serial.printf("Tare captured (s2): %s\n", calibration_tare_raw2 != 0 ? "yes" : "no");
            Serial.println("====================\n");
            break;
        }
        case 'p': {
            // Print raw averaged readings for diagnostics. Use 'p' or 'p1' for sensor1, 'p2' for sensor2
            if (sensor == 2) {
                if (LOADCELL2_DOUT_PIN == -1) {
                    Serial.println("[RAW] Sensor2 not configured");
                    break;
                }
                if (loadcell2.wait_ready_timeout(1000)) {
                    long raw2 = loadcell2.read_average(30);
                    long off2 = loadcell2_offset;
                    double g2 = (double)(raw2 - off2) / scaleFactor2;
                    Serial.printf("[RAW s2] raw=%ld offset=%ld factor=%.5f grams=%.3f\n", raw2, off2, scaleFactor2, g2);
                } else {
                    Serial.println("[RAW] HX711(sensor2) not ready for raw read");
                }
            } else {
                if (loadcell.wait_ready_timeout(1000)) {
                    long raw1 = loadcell.read_average(30);
                    long off1 = loadcell.get_offset();
                    double g1 = (double)(raw1 - off1) / scaleFactor;
                    Serial.printf("[RAW s1] raw=%ld offset=%ld factor=%.5f grams=%.3f\n", raw1, off1, scaleFactor, g1);
                } else {
                    Serial.println("[RAW] HX711(sensor1) not ready for raw read");
                }
            }
            break;
        }
        case 'T': {
            // Combined tare: tare primary sensor (request) and set offset for sensor2
            Serial.println("[CAL] Combined tare: taring primary and capturing sensor2 offset...");
            // Request tare for primary (handled in updateScale)
            tareScale();
            // Give updateScale some time to perform the tare
            delay(700);
            // Now capture sensor2 offset if available
            if (LOADCELL2_DOUT_PIN != -1) {
                if (loadcell2.wait_ready_timeout(500)) {
                    long off2 = loadcell2.read_average(20);
                    loadcell2.set_offset(off2);
                    // Update runtime variable used by scale.cpp and persist
                    loadcell2_offset = off2;
                    preferences.begin("scale", false);
                    preferences.putLong("offset2", off2);
                    preferences.end();
                    Serial.printf("[CAL] Sensor2 offset set to %ld and saved to NVS\n", off2);
                    // Block AZT briefly after setting offsets
                    aztBlockUntil = millis() + 10000UL;
                } else {
                    Serial.println("[CAL] Error: HX711(sensor2) not ready to capture offset");
                }
            } else {
                Serial.println("[CAL] No sensor2 configured");
            }
            break;
        }
        case 'O': {
            // Set sensor2 offset from current reading and save to preferences
            if (LOADCELL2_DOUT_PIN == -1) {
                Serial.println("[CAL] Sensor2 not configured");
                break;
            }
            Serial.println("[CAL] Capturing sensor2 offset (this will set offset2)...");
            if (loadcell2.wait_ready_timeout(500)) {
                long off2 = loadcell2.read_average(20);
                loadcell2.set_offset(off2);
                loadcell2_offset = off2; // keep runtime in sync
                preferences.begin("scale", false);
                preferences.putLong("offset2", off2);
                preferences.end();
                Serial.printf("[CAL] Sensor2 offset set to %ld and saved to NVS\n", off2);
                // Block AZT briefly after setting offsets
                aztBlockUntil = millis() + 10000UL;
            } else {
                Serial.println("[CAL] Error: HX711(sensor2) not ready");
            }
            break;
        }
        case 'h': {
            // Help
            Serial.println("\n=== Calibration Commands ===");
            Serial.println("t  - Tare sensor1 (use 't' or 't1')");
            Serial.println("t2 - Tare sensor2");
            Serial.println("c  - Enter calibration mode for sensor1 (then use w..)");
            Serial.println("c2 - Enter calibration mode for sensor2");
            Serial.println("w48.1 or w1 48.1 - Provide known weight for sensor1");
            Serial.println("w2 48.1 or w248.1 - Provide known weight for sensor2");
            Serial.println("s  - Show current status");
            Serial.println("h  - Show this help");
            Serial.println("============================\n");
            break;
        }
        case 'R': {
            // Reset scale calibration and offsets to defaults (destructive)
            Serial.println("[CAL] RESET: Clearing saved calibration and offsets in NVS and restoring defaults...");
            preferences.begin("scale", false);
            preferences.remove("calibration");
            preferences.remove("calibration2");
            preferences.remove("offset1");
            preferences.remove("offset2");
            preferences.remove("shotOffset");
            preferences.remove("shotCount");
            // Re-write sane defaults so runtime picks them up immediately
            preferences.putDouble("calibration", (double)LOADCELL_SCALE_FACTOR);
            if (LOADCELL2_DOUT_PIN != -1) preferences.putDouble("calibration2", (double)LOADCELL2_SCALE_FACTOR);
            preferences.end();

            // Apply defaults to runtime immediately
            scaleFactor = (double)LOADCELL_SCALE_FACTOR;
            loadcell.set_scale(scaleFactor);
            Serial.printf("[CAL] scaleFactor reset to default: %.6f\n", scaleFactor);
            if (LOADCELL2_DOUT_PIN != -1) {
                scaleFactor2 = (double)LOADCELL2_SCALE_FACTOR;
                loadcell2.set_scale(scaleFactor2);
                Serial.printf("[CAL] scaleFactor2 reset to default: %.6f\n", scaleFactor2);
            }

            Serial.println("[CAL] RESET complete. Please run 'T' to tare the empty platform and then re-calibrate.");
            break;
        }
        case 'G': {
            // Guided combined calibration: usage G77.08 or G 77.08
            String rest = "";
            if (line.length() > 1) rest = line.substring(1);
            rest.trim();
            float known = rest.toFloat();
            if (known <= 0.0f) {
                Serial.println("[CAL] Usage: G<grams>  e.g. G77.08  -> combined tare + per-sensor multipliers");
                break;
            }

            Serial.printf("[CAL] Guided combined calibration starting with known mass = %.3fg\n", known);
            // Safety check: do not run combined guided calibration if the platform is not empty.
            // The previous implementation called tare internally which could capture the mass
            // if the user had already placed it. Require a clean empty tare first.
            if (fabs(scaleWeight) > 2.0) {
                Serial.println("[CAL] Aborting: platform is not empty or scale reads >2g.");
                Serial.println("       Please remove any mass, run 'T' to tare the empty platform, then place the known mass and run 'G' again.");
                break;
            }

            Serial.println("[CAL] Platform looks empty. Please place the known mass now and wait a few seconds for readings to settle...");
            // Give user time to place the known mass and allow readings to stabilize
            delay(1200);

            // Interactive confirmation: wait for the user to press ENTER to continue, or 'a' to abort.
            Serial.println("[CAL] When the mass is placed and stable, press ENTER to continue (or type 'a' then ENTER to abort).");
            bool user_aborted = false;
            unsigned long user_wait_start = millis();
            // Wait up to 30 seconds for user confirmation
            while (true) {
                if (Serial.available()) {
                    String confirm = Serial.readStringUntil('\n');
                    confirm.trim();
                    if (confirm.length() == 0) {
                        // ENTER pressed with empty line -> proceed
                        break;
                    }
                    if (confirm.equalsIgnoreCase("a")) {
                        user_aborted = true;
                        break;
                    }
                    // Any other input -> proceed as confirmation
                    break;
                }
                if (millis() - user_wait_start > 30000UL) {
                    Serial.println("[CAL] Timeout waiting for user confirmation - aborting");
                    user_aborted = true;
                    break;
                }
                delay(50);
            }
            if (user_aborted) {
                Serial.println("[CAL] Guided calibration aborted by user.");
                break;
            }

            // 2) Do NOT capture/tare sensor2 here - using previously saved offsets is safer.
            // Capturing offsets while the mass is already on the platform will corrupt the tare.
            if (LOADCELL2_DOUT_PIN != -1) {
                if (loadcell2_offset == 0) {
                    Serial.println("[CAL] Error: sensor2 offset not set. Run 'T' (combined tare) on an empty platform first, then retry 'G'.");
                    break;
                } else {
                    Serial.printf("[CAL] Using existing sensor2 offset: %ld\n", loadcell2_offset);
                }
            }

            // Prompt: user should have placed the known mass already
            Serial.println("[CAL] Reading sensors for calibration (ensure mass is placed and stable)...");
            // Read averages
            if (!loadcell.wait_ready_timeout(1000)) {
                Serial.println("[CAL] Error: HX711(sensor1) not ready");
                break;
            }
            long raw1 = loadcell.read_average(30);
            long off1 = loadcell.get_offset();
            double rawdiff1 = (double)(raw1 - off1);

            double measured1 = rawdiff1 / scaleFactor;

            double measured2 = 0.0;
            long raw2 = 0;
            if (LOADCELL2_DOUT_PIN != -1) {
                if (!loadcell2.wait_ready_timeout(1000)) {
                    Serial.println("[CAL] Error: HX711(sensor2) not ready for measurement");
                    break;
                }
                raw2 = loadcell2.read_average(30);
                long off2 = loadcell2_offset;
                double rawdiff2 = (double)(raw2 - off2);
                measured2 = rawdiff2 / scaleFactor2;
            }

            // Compute measured total as the average of both sensor contributions.
            // The system uses averaging for fusion so adjust calibration accordingly.
            double measured_total = (measured1 + measured2) / 2.0;
            Serial.printf("[CAL] Measured contributions (avg): s1=%.3fg  s2=%.3fg  avg=%.3fg\n", measured1, measured2, measured_total);
            if (measured_total <= 0.0001) {
                Serial.println("[CAL] Error: measured total is zero or negative - aborting");
                break;
            }

            double multiplier = (double)known / measured_total;
            Serial.printf("[CAL] Multiplier computed: %.6f\n", multiplier);

            // Apply multiplier to both scale factors (keep relative split)
            scaleFactor = scaleFactor * multiplier;
            loadcell.set_scale(scaleFactor);
            preferences.begin("scale", false);
            preferences.putDouble("calibration", (double)scaleFactor);
            preferences.end();

            if (LOADCELL2_DOUT_PIN != -1) {
                scaleFactor2 = scaleFactor2 * multiplier;
                loadcell2.set_scale(scaleFactor2);
                preferences.begin("scale", false);
                preferences.putDouble("calibration2", (double)scaleFactor2);
                preferences.end();
            }

            // Block AZT briefly after guided combined calibration so AZT does not undo calibration
            aztBlockUntil = millis() + 10000UL;

            Serial.printf("[CAL] New factors saved: s1=%.6f  s2=%.6f\n", scaleFactor, scaleFactor2);
            Serial.println("[CAL] Verification read:");
            // Do a verification read
            if (loadcell.wait_ready_timeout(1000)) {
                long v1 = loadcell.read_average(10);
                double g1 = (double)(v1 - loadcell.get_offset()) / scaleFactor;
                double g2 = 0.0;
                if (LOADCELL2_DOUT_PIN != -1 && loadcell2.wait_ready_timeout(1000)) {
                    long v2 = loadcell2.read_average(10);
                    g2 = (double)(v2 - loadcell2_offset) / scaleFactor2;
                }
                double verified_total = (g1 + g2) / 2.0;
                Serial.printf("[CAL] Verified contributions (avg): s1=%.3fg  s2=%.3fg  avg=%.3fg\n", g1, g2, verified_total);
            }
            break;
        }
    }
    // Flush remaining input
    while(Serial.available()) Serial.read();
} 

void setup() {
    Serial.begin(115200);
    
    // WiFi and Bluetooth disabled - fully commented out
    // WiFi.mode(WIFI_OFF);
    // WiFi.disconnect(true);
// #ifdef ARDUINO_ARCH_ESP32
//     btStop();
// #endif
    
    // Setup other components
    setupDisplay();
    setupScale();
    // setupWebServer(); // Disabled
}

void loop() {
    processSerialCommands();
    delay(100);
}
