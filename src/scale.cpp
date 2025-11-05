#include "config.hpp"
#include "rotary.hpp"
#include "scale.hpp"
#include "display.hpp"

// Variables for scale functionality
// HX711 operation flags
volatile bool requestTare = false;
volatile bool requestSetOffset = false;
volatile bool requestCalibration = false;
double scaleWeight = 0;       // Current weight measured by the scale
double setWeight = 0;         // Target weight set by the user
double setCupWeight = 0;      // Weight of the cup set by the user
// shotOffset: grams adjustment applied after a grind (used to bias target to
// compensate for grinder runout). Separate from the HX711 raw offset counts.
double shotOffset = 0;        // grams
// Primary HX711 raw offset (counts)
long loadcell_offset = 0;
bool scaleMode = false;       // Indicates if the scale is used in timer mode
bool grindMode = false;       // Grinder mode: impulse (false) or continuous (true)
bool grinderActive = false;   // Grinder state (on/off)
unsigned int shotCount;

// Buffer for storing recent weight history
MathBuffer<double, 100> weightHistory;
// Per-sensor history for AZT to operate on each cell individually
MathBuffer<double, 100> weightHistory2;

// Auto-Zero Tracking (AZT) - helps correct residual offsets when stable near zero
bool auto_zero_enabled = true;
int auto_zero_stable = 0;
int auto_zero_stable2 = 0; // independent stability counter for sensor2
const float AZT_WINDOW_G = 1.0f;   // grams window considered "near zero"
const int AZT_REQUIRED = 8;        // consecutive stable cycles required to auto-zero
const float AZT_MIN_G = 0.25f;     // only allow AZT when very close to zero (smaller than this)
bool history_seeded = false;       // seed history on first successful read

// Block AZT for a short time after calibration operations to avoid AZT fighting calibration
unsigned long aztBlockUntil = 0;

// Timing and status variables
unsigned long scaleLastUpdatedAt = 0;  // Timestamp of the last scale update
unsigned long lastSignificantWeightChangeAt = 0; // Timestamp of the last significant weight change
unsigned long lastTareAt = 0; // Timestamp of the last tare operation
bool scaleReady = false;      // Indicates if the scale is ready to measure
int scaleStatus = STATUS_EMPTY; // Current status of the scale
double cupWeightEmpty = 0;    // Measured weight of the empty cup
unsigned long startedGrindingAt = 0;  // Timestamp of when grinding started
unsigned long finishedGrindingAt = 0; // Timestamp of when grinding finished
bool greset = false;          // Flag for reset operation
bool newOffset = false;       // Indicates if a new offset value is pending

bool useButtonToGrind = DEFAULT_GRIND_TRIGGER_MODE;

// Taring message state variables
bool showingTaringMessage = false;
unsigned long taringMessageStartTime = 0;

// Manual grinding mode - grinder controlled directly by button
bool manualGrindMode = false;

double scaleFactor = 1409.88; // Standard scale factor, can be updated by calibration
// Secondary scale factor and weight (optional) — defined in main.cpp
double scaleWeight2 = 0;
long loadcell2_offset = 0;

// Optional: attempt to vibrate the grinder for a short moment after grinding
// finishes to help dislodge adhered grounds and improve final weight accuracy.
// Controlled by preference key "autoVibe" (default false).
bool auto_vibe_after_grind = false;
// When true, the display will add the stuck-grounds compensation (+0.5g)
// to the shown weight while on the "Grinding finished" screen. It is set
// when grinding transitions to finished and cleared when the user leaves
// that screen (handled in the display task).
bool display_compensate_shot = false;
// Amount (grams) to add to displayed and auto-adjusted weight to compensate
// for stuck/adhered grounds. Default set to 1.0 g per user request.
double display_compensation_g = 1.0;

                    // displayWeight is updated in the display task when needed
bool tareScale()
{
    // Set tare request flag, actual tare will be performed in updateScale
    requestTare = true;
    // Also request that the secondary sensor's offset be captured when the tare runs.
    // This makes a single tare operation (e.g. rotary double-tap) apply to both HX711 modules.
    requestSetOffset = true;
    // Do not perform HX711 operations here
    return true;
}

// Task to continuously update the scale readings
void updateScale(void *parameter) {
    float lastEstimate;
    const TickType_t xDelay = 50 / portTICK_PERIOD_MS; // 20Hz = 50ms interval (faster sampling)
    int hx711_fail_count = 0;
    for (;;) {
        vTaskDelay(1); // Minimal delay to mitigate timing/race condition
        // Request tare on startup if needed (capture both primary and secondary offsets)
        if (lastTareAt == 0) {
            requestTare = true;
            requestSetOffset = true; // ensure secondary HX711 offset is captured on first startup tare
        }
        // Serialize HX711 access: handle tare request first
        if (requestTare) {
            Serial.println("Taring scale (serialized in updateScale)...");
            bool tareSuccess = false;
            for (int attempt = 1; attempt <= 3; ++attempt) {
                Serial.printf("[tareScale] Attempt %d: Waiting for HX711 ready...\n", attempt);
                unsigned long t0 = millis();
                bool ready = loadcell.wait_ready_timeout(1000);
                Serial.printf("[tareScale] wait_ready_timeout returned %s after %lu ms\n", ready ? "true" : "false", millis() - t0);
                if (ready) {
                    Serial.println("[tareScale] HX711 ready, reading average...");
                    t0 = millis();
                    long off1 = loadcell.read_average(10); // Average 10 readings for stability
                    Serial.printf("[tareScale] read_average finished after %lu ms\n", millis() - t0);
                    loadcell.set_offset(off1);
                    loadcell_offset = off1; // store in runtime var
                    // persist primary HX711 counts so taring survives reboot
                    preferences.begin("scale", false);
                    preferences.putLong("offset1", off1);
                    preferences.end();
                    lastTareAt = millis();
                    scaleWeight = 0;
                    // Reinitialize Kalman with the same responsive parameters used at startup
                    kalmanFilter = SimpleKalmanFilter(0.5, 0.01, 0.01);
                    Serial.println("Scale tared successfully");
                    tareSuccess = true;
                    break;
                } else {
                    Serial.print("Tare attempt ");
                    Serial.print(attempt);
                    Serial.println(": HX711 not ready, retrying...");
                    Serial.println("[tareScale] Delaying for 200ms...");
                    unsigned long d0 = millis();
                    delay(200);
                    Serial.printf("[tareScale] Delay finished after %lu ms\n", millis() - d0);
                }
            }
            requestTare = false;
            // If tare failed, skip regular sampling this cycle
            if (!tareSuccess) {
                vTaskDelay(xDelay);
                continue;
            }
                // If requested, capture and persist the secondary HX711 offset now that primary tare succeeded
                if (requestSetOffset && LOADCELL2_DOUT_PIN != -1) {
                    Serial.println("[tareScale] Capturing secondary HX711 offset as requested...");
                    if (loadcell2.wait_ready_timeout(1000)) {
                        long off2 = loadcell2.read_average(20);
                        loadcell2.set_offset(off2);
                        loadcell2_offset = off2;
                        preferences.begin("scale", false);
                        preferences.putLong("offset2", off2);
                        preferences.end();
                        Serial.printf("[tareScale] Sensor2 offset set to %ld and saved to NVS\n", off2);
                        // Block AZT briefly after setting offsets
                        aztBlockUntil = millis() + 10000UL;
                    } else {
                        Serial.println("[tareScale] Warning: HX711(sensor2) not ready to capture offset");
                    }
                    requestSetOffset = false;
                }
        }
        // Regular HX711 sampling
        unsigned long t0 = millis();
    bool ready = loadcell.wait_ready_timeout(300);
        if (ready) {
            hx711_fail_count = 0;
        long raw;
        long raw_offset = loadcell.get_offset();
        double grams;
        // Optional second sensor
        long raw2 = 0;
        long raw2_offset = loadcell2_offset;
        double grams2 = 0;
                
                // Seed history on first successful read to avoid large initial deltas
                if (!history_seeded) {
                    long seed_raw = loadcell.read_average(5);
                    double seed_grams = (double)(seed_raw - raw_offset) / scaleFactor;
                    for (int i = 0; i < 20; ++i) {  // Seed last 20 values
                        weightHistory.push(seed_grams);
                    }
                    // Seed sensor2 history if present
                    if (LOADCELL2_DOUT_PIN != -1) {
                        long seed_raw2 = loadcell2.read_average(5);
                        double seed_grams2 = (double)(seed_raw2 - raw2_offset) / scaleFactor2;
                        for (int i = 0; i < 20; ++i) {
                            weightHistory2.push(seed_grams2);
                        }
                    }
                    history_seeded = true;
                    Serial.println("Weight history seeded to reduce initial spikes.");
                }
                
                if (scaleStatus == STATUS_GRINDING_IN_PROGRESS) {
                    // When grinding, use single reads for speed
                    raw = loadcell.read();
                    grams = (double)(raw - raw_offset) / scaleFactor;
                    if (LOADCELL2_DOUT_PIN != -1) {
                        raw2 = loadcell2.read();
                        grams2 = (double)(raw2 - raw2_offset) / scaleFactor2;
                    }
                } else {
                    // Use minimal averaging to reduce latency; Kalman provides smoothing
                    raw = loadcell.read_average(1);
                    grams = (double)(raw - raw_offset) / scaleFactor;
                    if (LOADCELL2_DOUT_PIN != -1) {
                        raw2 = loadcell2.read_average(1);
                        grams2 = (double)(raw2 - raw2_offset) / scaleFactor2;
                    }
                }
                // Debug: print raw HX711 values to help troubleshoot calibration/noise
                if (LOADCELL2_DOUT_PIN != -1) {
                    Serial.printf("[HX711-1] raw=%ld offset=%ld factor=%.5f grams=%.3f  |  [HX711-2] raw=%ld offset=%ld factor=%.5f grams=%.3f\n", 
                                  raw, raw_offset, scaleFactor, grams, raw2, raw2_offset, scaleFactor2, grams2);
                    // Combine sensors by averaging their gram contributions when both are present.
                    // Many rigs have each sensor measuring the full platform load; averaging
                    // produces the correct single-mass reading when both sensors see the same
                    // mass. If you later want to revert to summing, change this back to (grams + grams2).
                    double combined = (grams + grams2) / 2.0;
                    scaleWeight = kalmanFilter.updateEstimate(combined);
                    scaleWeight2 = grams2;
                    // push per-sensor values for AZT
                    weightHistory.push(scaleWeight);
                    if (LOADCELL2_DOUT_PIN != -1) weightHistory2.push(scaleWeight2);
                } else {
                    Serial.printf("[HX711] raw=%ld offset=%ld factor=%.5f grams=%.3f\n", raw, raw_offset, scaleFactor, grams);
                    scaleWeight = kalmanFilter.updateEstimate(grams);
                    // push primary sensor value and seed sensor2 history with zero if absent
                    weightHistory.push(scaleWeight);
                    if (LOADCELL2_DOUT_PIN != -1) weightHistory2.push(scaleWeight2);
                }
            
            // Auto-Zero Tracking: gently correct tare when stable and very close to zero
                if (auto_zero_enabled && scaleStatus == STATUS_EMPTY) {
                // Respect global AZT block (set by calibration commands) to avoid AZT fighting calibration
                if (millis() < aztBlockUntil) {
                    // AZT blocked during cooldown
                    auto_zero_stable = 0;
                    auto_zero_stable2 = 0;
                } else if (scaleReady && fabs(scaleWeight) <= AZT_MIN_G) {
                    // Check per-sensor stability and adjust each cell independently
                    double recent_avg1 = weightHistory.averageSince(millis() - 2000);
                    double recent_avg2 = 0.0;
                    if (LOADCELL2_DOUT_PIN != -1) recent_avg2 = weightHistory2.averageSince(millis() - 2000);

                    // Primary sensor AZT
                    if (fabs(recent_avg1) <= AZT_MIN_G) {
                        auto_zero_stable++;
                        if (auto_zero_stable >= AZT_REQUIRED) {
                            long adjustment = (long)(recent_avg1 * scaleFactor);
                            long old_offset = loadcell.get_offset();
                            loadcell.set_offset(old_offset + adjustment);
                            auto_zero_stable = 0;
                            Serial.printf("[AZT] Auto-zero adjusted primary tare by %+ld counts (%.2fg)\n", adjustment, recent_avg1);
                            // do not persist here; primary will be persisted on next manual tare
                        }
                    } else {
                        auto_zero_stable = 0;
                    }

                    // Secondary sensor AZT (if available)
                    if (LOADCELL2_DOUT_PIN != -1) {
                        if (fabs(recent_avg2) <= AZT_MIN_G) {
                            auto_zero_stable2++;
                            if (auto_zero_stable2 >= AZT_REQUIRED) {
                                long adjustment2 = (long)(recent_avg2 * scaleFactor2);
                                long old_offset2 = loadcell2.get_offset();
                                loadcell2.set_offset(old_offset2 + adjustment2);
                                auto_zero_stable2 = 0;
                                Serial.printf("[AZT] Auto-zero adjusted secondary tare by %+ld counts (%.2fg)\n", adjustment2, recent_avg2);
                                // do not persist here; secondary offset persisted on manual tare
                            }
                        } else {
                            auto_zero_stable2 = 0;
                        }
                    }
                } else {
                    auto_zero_stable = 0;
                    auto_zero_stable2 = 0;
                }
            }
            
            // Removed: always report true scaleWeight, even near zero
            scaleLastUpdatedAt = millis();
            weightHistory.push(scaleWeight);
            scaleReady = true;
        } else {
            hx711_fail_count++;
            Serial.println("HX711 not found.");
            scaleReady = false;
            if (scaleStatus != STATUS_GRINDING_IN_PROGRESS && hx711_fail_count >= 5) {
                Serial.println("HX711 failed 5 times, skipping readings for 500ms.");
                vTaskDelay(500 / portTICK_PERIOD_MS);
                hx711_fail_count = 0;
            }
        }
        vTaskDelay(xDelay); // Wait 100ms before next read (10Hz)
    }
}

// Toggles the grinder on or off based on mode
void grinderToggle() {
    Serial.println("[grinderToggle] called");
    // Toggle grinder/LED output
    if (!grinderActive) {
        Serial.println("[grinderToggle] BEFORE digitalWrite ON");
        digitalWrite(GRINDER_ACTIVE_PIN, LOW); // Relay/LED ON
        delay(1); // Minimal delay after toggling ON
        Serial.println("[grinderToggle] AFTER digitalWrite ON");
        grinderActive = true;
        Serial.println("Grinder/LED ON");
    } else {
        Serial.println("[grinderToggle] BEFORE digitalWrite OFF");
        digitalWrite(GRINDER_ACTIVE_PIN, HIGH); // Relay/LED OFF
        delay(1); // Minimal delay after toggling OFF
        Serial.println("[grinderToggle] AFTER digitalWrite OFF");
        grinderActive = false;
        Serial.println("Grinder/LED OFF");
    }
}

// Task to manage the status of the scale
void scaleStatusLoop(void *p) {
    for (;;) {
        // Monitor heap and stack usage (deactivated)
        // Serial.printf("[Heap] Free: %u | [Stack] High Water Mark: %u\n", ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1); // Minimal delay to mitigate timing/race condition
        double tenSecAvg = weightHistory.averageSince((int64_t)millis() - 10000);
        if (ABS(tenSecAvg - scaleWeight) > SIGNIFICANT_WEIGHT_CHANGE) {
            lastSignificantWeightChangeAt = millis();
        }
        switch (scaleStatus) {
            case STATUS_EMPTY: {
                // Auto-tare is disabled except for startup (handled in updateScale)
                static bool grinderButtonPressed = false;
                static unsigned long grinderButtonPressedAt = 0;
                static bool manualGrinderActive = false;

                // Manual grind mode - direct control of grinder with button
                if (manualGrindMode) {
                    bool buttonCurrentlyPressed = (digitalRead(GRIND_BUTTON_PIN) == LOW);
                    if (buttonCurrentlyPressed && !manualGrinderActive) {
                        // Button just pressed - start grinder
                        manualGrinderActive = true;
                        Serial.println("[ManualGrind] BEFORE digitalWrite ON");
                        digitalWrite(GRINDER_ACTIVE_PIN, LOW); // Relay ON
                        delay(1); // Minimal delay after toggling ON
                        Serial.println("[ManualGrind] AFTER digitalWrite ON");
                        Serial.println("Manual grind: Grinder ON");
                        wakeScreen();
                    } else if (!buttonCurrentlyPressed && manualGrinderActive) {
                        // Button just released - stop grinder
                        manualGrinderActive = false;
                        Serial.println("[ManualGrind] BEFORE digitalWrite OFF");
                        digitalWrite(GRINDER_ACTIVE_PIN, HIGH); // Relay OFF
                        delay(1); // Minimal delay after toggling OFF
                        Serial.println("[ManualGrind] AFTER digitalWrite OFF");
                        Serial.println("Manual grind: Grinder OFF");
                    }
                    break; // Skip automatic grinding logic when in manual mode
                }

                // Only allow button trigger if grindMode == true (automatic mode)
                if (grindMode && digitalRead(GRIND_BUTTON_PIN) == LOW && !grinderButtonPressed) {
                    grinderButtonPressed = true;
                    grinderButtonPressedAt = millis();
                    wakeScreen(); // wake screen immediately
                    Serial.println("Grinder button pressed, taring and waking screen...");
                    // Tare the scale before starting grinding
                    requestTare = true;
                }
            
                if (grindMode && grinderButtonPressed && millis() - grinderButtonPressedAt >= 600) {
                    grinderButtonPressed = false; // reset flag
                        // Capture a fresh, short-average reading for the empty cup weight so we don't
                        // rely on heavily-filtered `scaleWeight` which can lag and introduce bias.
                        if (loadcell.wait_ready_timeout(500)) {
                            long raw_tare = loadcell.read_average(5);
                            long off = loadcell.get_offset();
                            cupWeightEmpty = (double)(raw_tare - off) / scaleFactor;
                        } else {
                            // fallback to filtered value if HX711 not ready
                            cupWeightEmpty = scaleWeight;
                        }
                    scaleStatus = STATUS_GRINDING_IN_PROGRESS;
                    // Ensure display shows the stuck-grounds compensation from the
                    // start of the grind so the finished screen already reflects it.
                    display_compensate_shot = true;
                    if (!scaleMode) {
                        newOffset = true;
                        startedGrindingAt = millis();
                    }
                    grinderToggle();
                    Serial.println("Grinding started after tare and delay.");
                    continue;
                }
            
                // Only allow cup trigger if grindMode == false
                if (!grindMode &&
                    ABS(weightHistory.minSince(millis() - 1000) - setCupWeight) < CUP_DETECTION_TOLERANCE &&
                    ABS(weightHistory.maxSince(millis() - 1000) - setCupWeight) < CUP_DETECTION_TOLERANCE) {
                    
                    cupWeightEmpty = weightHistory.averageSince(millis() - 500);
                    scaleStatus = STATUS_GRINDING_IN_PROGRESS;
                    // Ensure display shows the stuck-grounds compensation from the
                    // start of the grind so the finished screen already reflects it.
                    display_compensate_shot = true;
                    if (!scaleMode) {
                        newOffset = true;
                        startedGrindingAt = millis();
                    }
                    grinderToggle();
                    Serial.println("Grinding started from cup detection.");
                    continue;
                }
            
                break;
            }            
        case STATUS_GRINDING_IN_PROGRESS:
        {
            if (scaleWeight < -10.0)
            { // Only fail if weight is significantly negative (cup removed)
                Serial.println("GRINDING FAILED: Significantly negative weight detected (cup removed).");
                grinderToggle(); // Ensure grinder is off
                scaleStatus = STATUS_GRINDING_FAILED;
                continue;
            }
            if (!scaleReady)
            {
                Serial.println("GRINDING FAILED: Scale not ready");
                grinderToggle();
                scaleStatus = STATUS_GRINDING_FAILED;
                continue;
            }
                if (scaleMode && startedGrindingAt == 0 && scaleWeight - cupWeightEmpty >= 0.1) {
                startedGrindingAt = millis();
                continue;
            }
                if (millis() - startedGrindingAt > MAX_GRINDING_TIME && !scaleMode) {
                Serial.println("GRINDING FAILED: Max grinding time exceeded");
                grinderToggle();
                scaleStatus = STATUS_GRINDING_FAILED;
                continue;
            }
            if (millis() - startedGrindingAt > 5000 &&
                scaleWeight - weightHistory.firstValueOlderThan(millis() - 5000) < 1 &&
                    !scaleMode) {
                Serial.println("GRINDING FAILED: No weight increase after 2 seconds");
                grinderToggle();
                scaleStatus = STATUS_GRINDING_FAILED;
                continue;
            }
                if (weightHistory.minSince((int64_t)millis() - 200) < cupWeightEmpty - CUP_DETECTION_TOLERANCE && !scaleMode) {
                Serial.printf("GRINDING FAILED: Cup removed - min weight: %.2f, cup weight: %.2f, tolerance: %d\n", 
                             weightHistory.minSince((int64_t)millis() - 200), cupWeightEmpty, CUP_DETECTION_TOLERANCE);
                grinderToggle();
                scaleStatus = STATUS_GRINDING_FAILED;
                continue;
            }
            double currentOffset = shotOffset;
                if (scaleMode) {
                currentOffset = 0;
            }
                double grindTarget;
                if (grindMode && !manualGrindMode) {
                    // Button-activated automatic grinding: ignore cup weight
                    grindTarget = setWeight + currentOffset;
                } else {
                    // Other modes: include cup weight
                    grindTarget = cupWeightEmpty + setWeight + currentOffset;
                }
                if (weightHistory.maxSince((int64_t)millis() - 200) >= grindTarget) {
                    finishedGrindingAt = millis();
                    grinderToggle();
                    scaleStatus = STATUS_GRINDING_FINISHED;
                    // Mark that the display should apply the stuck-grounds compensation
                    // until the user leaves the "Grinding finished" screen.
                    display_compensate_shot = true;
                    continue;
                }
            break;
        }
        case STATUS_GRINDING_FINISHED:
        {
            static unsigned long grindingFinishedAt = 0;

            // Record the time when grinding finished if not already recorded
            if (grindingFinishedAt == 0)
            {
                grindingFinishedAt = millis();
                Serial.print("Grinder was on for: ");
                Serial.print(grindingFinishedAt);
                Serial.println(" seconds");
            }

            // Short-term average of recent weights (used as fallback)
            double currentWeight = weightHistory.averageSince((int64_t)millis() - 500);
            if (scaleWeight < 5) {
                startedGrindingAt = 0;
                grindingFinishedAt = 0; // Reset the timestamp
                scaleWeight = 0;
                scaleStatus = STATUS_EMPTY;
                continue;
            } else if (millis() - finishedGrindingAt > 5000) {
                // After the finish timeout we may optionally pulse the grinder relay to
                // help dislodge adhered grounds. The automatic shotOffset adjustment
                // used to run here; the adjustment now occurs only when the user
                // presses the button to leave the finished screen (see
                // applyShotOffsetAdjustmentOnExit()).
                if (auto_vibe_after_grind && !grinderActive) {
                    Serial.println("Auto-vibe: pulsing grinder relay to settle grounds...");
                    for (int i = 0; i < 2; ++i) {
                        digitalWrite(GRINDER_ACTIVE_PIN, LOW); // ON
                        delay(60);
                        digitalWrite(GRINDER_ACTIVE_PIN, HIGH); // OFF
                        delay(80);
                    }
                    // small settle delay
                    delay(150);
                }
            }

            // Timeout to transition back to the main menu after grinding finishes
            if (millis() - grindingFinishedAt > 5000)
            { // 5-second delay after grinding finishes
                if (scaleWeight >= 3)
                { // If weight is still on the scale, wait for cup removal
                    Serial.println("Waiting for cup to be removed...");
                }
                else
                {
                    startedGrindingAt = 0;
                    grindingFinishedAt = 0; // Reset the timestamp
                    scaleStatus = STATUS_EMPTY;
                    Serial.println("Grinding finished. Transitioning to main menu.");
                }
            }
            break;
        }
        case STATUS_GRINDING_FAILED:
        {
            if (scaleWeight >= GRINDING_FAILED_WEIGHT_TO_RESET)
            {
                scaleStatus = STATUS_EMPTY;
                continue;
            }
            break;
        }
        }
        rotary_loop();
        delay(50);
    }
}

// Alternative ISR function that calls the library ISR
void IRAM_ATTR encoderISR() {
    rotaryEncoder.readEncoder_ISR();
}

// Initializes the scale hardware and settings
void setupScale() {
    Serial.println("Initializing rotary encoder...");
    Serial.print("Encoder A pin: ");
    Serial.println(ROTARY_ENCODER_A_PIN);
    Serial.print("Encoder B pin: ");
    Serial.println(ROTARY_ENCODER_B_PIN);
    Serial.print("Encoder Button pin: ");
    Serial.println(ROTARY_ENCODER_BUTTON_PIN);
    
    // Set pin modes first with pullups
    pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
    pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);
    
    Serial.println("Pin modes set with pullups");
    
    // Test initial pin states
    Serial.println("Testing encoder pins directly:");
    Serial.print("Pin A state: ");
    Serial.println(digitalRead(ROTARY_ENCODER_A_PIN));
    Serial.print("Pin B state: ");
    Serial.println(digitalRead(ROTARY_ENCODER_B_PIN));
    
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    
    // Enable circuitry for encoder
    rotaryEncoder.enable();
    
    // Try manual interrupt attachment as fallback
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_A_PIN), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_B_PIN), encoderISR, CHANGE);
    
    Serial.println("Interrupts attached manually as fallback");
    
    // Set boundaries and acceleration - make it more responsive
    rotaryEncoder.setBoundaries(-10000, 10000, true);
    rotaryEncoder.setAcceleration(0); // Disable acceleration for more predictable response
    
    // Test encoder by reading initial value
    int initialValue = rotaryEncoder.readEncoder();
    Serial.print("Initial encoder value: ");
    Serial.println(initialValue);
    
    Serial.println("Rotary encoder initialized successfully.");
    
    Serial.println("Initializing load cell...");
    // Set HX711 to 10Hz (default debug mode)
    loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); // default 10 Hz mode
    pinMode(GRINDER_ACTIVE_PIN, OUTPUT);
    pinMode(GRIND_BUTTON_PIN, INPUT_PULLUP);
    digitalWrite(GRINDER_ACTIVE_PIN, HIGH); // Initialize HIGH = Relay OFF = Grinder stopped
    Serial.println("Load cell and pins initialized.");

    preferences.begin("scale", false);

    // Load stored calibration into the global scaleFactor (do NOT shadow)
    scaleFactor = preferences.getDouble("calibration", (double)LOADCELL_SCALE_FACTOR);
    if (scaleFactor <= 0 || std::isnan(scaleFactor)) {
        scaleFactor = LOADCELL_SCALE_FACTOR;
        preferences.putDouble("calibration", scaleFactor);
        Serial.println("Invalid scale factor detected. Resetting to default.");
    }
    // Load secondary calibration if configured
    if (LOADCELL2_DOUT_PIN != -1) {
        scaleFactor2 = preferences.getDouble("calibration2", (double)LOADCELL2_SCALE_FACTOR);
        loadcell2_offset = (long)preferences.getLong("offset2", 0);
        if (scaleFactor2 <= 0 || std::isnan(scaleFactor2)) {
            scaleFactor2 = LOADCELL2_SCALE_FACTOR;
            preferences.putDouble("calibration2", scaleFactor2);
            Serial.println("Invalid scaleFactor2 detected. Resetting to default.");
        }
    }
    setWeight = preferences.getDouble("setWeight", (double)COFFEE_DOSE_WEIGHT);
    // Load shotOffset with migration: prefer stored "shotOffset", fallback to legacy "offset"
    shotOffset = preferences.getDouble("shotOffset", preferences.getDouble("offset", (double)COFFEE_DOSE_OFFSET));
    // Load primary HX711 raw offset counts (fallback to current library offset)
    loadcell_offset = (long)preferences.getLong("offset1", loadcell.get_offset());
    setCupWeight = preferences.getDouble("cup", (double)CUP_WEIGHT);
    scaleMode = preferences.getBool("scaleMode", false);
    grindMode = preferences.getBool("grindMode", false);
    shotCount = preferences.getUInt("shotCount", 0);
    sleepTime = preferences.getInt("sleepTime", SLEEP_AFTER_MS); // Default to SLEEP_AFTER_MS if not set
    useButtonToGrind = preferences.getBool("grindTrigger", DEFAULT_GRIND_TRIGGER_MODE);
    manualGrindMode = preferences.getBool("manualGrindMode", false);
    // Load persisted display compensation (fallback to current default)
    display_compensation_g = preferences.getDouble("displayCompensation", display_compensation_g);
    preferences.end();
    Serial.printf("→ scaleFactor = %.6f  |  shotOffset = %.6f\n", scaleFactor, shotOffset);
    // Apply calibration to HX711 library and set stored raw offset counts
    loadcell.set_scale(scaleFactor);
    loadcell.set_offset(loadcell_offset);
    // Initialize second loadcell if configured
    if (LOADCELL2_DOUT_PIN != -1) {
        loadcell2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
        loadcell2.set_scale(scaleFactor2);
        loadcell2.set_offset(loadcell2_offset);
        Serial.printf("→ scaleFactor2 = %.6f  |  offset2 = %ld\n", scaleFactor2, loadcell2_offset);
        }
    Serial.printf("→ Manual Grind Mode: %s\n", manualGrindMode ? "ENABLED" : "DISABLED");
        // Load optional micro-vibe preference (default disabled)
        preferences.begin("scale", false);
        auto_vibe_after_grind = preferences.getBool("autoVibe", false);
        preferences.end();
        Serial.printf("→ autoVibeAfterGrind = %s\n", auto_vibe_after_grind ? "ENABLED" : "DISABLED");
        // loadcell.set_scale(scaleFactor); // Not used in debug form
        // loadcell.set_offset(offset); // Not used in debug form

    xTaskCreatePinnedToCore(updateScale, "Scale", 20000, NULL, 0, &ScaleTask, 1);
    xTaskCreatePinnedToCore(scaleStatusLoop, "ScaleStatus", 20000, NULL, 0, &ScaleStatusTask, 1);
}

// Perform the final raw reads and apply shotOffset adjustment if needed.
// This function is called when the user explicitly exits the "Grinding finished"
// screen (button press). It mirrors the previous timer-based adjustment logic
// but runs synchronously from the button handler so the user controls when the
// adjustment occurs.
void applyShotOffsetAdjustmentOnExit()
{
    if (!newOffset) {
        // Nothing to do
        Serial.println("applyShotOffsetAdjustmentOnExit: no pending offset to adjust");
        return;
    }

    // Read fresh raw averages from both sensors for the final weight to avoid
    // long filter tails; fall back to the filtered values if HX711 isn't ready.
    double final1 = weightHistory.averageSince((int64_t)millis() - 500);
    double final2 = 0.0;
    if (loadcell.wait_ready_timeout(500)) {
        long raw_final1 = loadcell.read_average(5);
        final1 = (double)(raw_final1 - loadcell.get_offset()) / scaleFactor;
    }
    if (LOADCELL2_DOUT_PIN != -1) {
        if (loadcell2.wait_ready_timeout(500)) {
            long raw_final2 = loadcell2.read_average(5);
            final2 = (double)(raw_final2 - loadcell2.get_offset()) / scaleFactor2;
        } else {
            final2 = scaleWeight2; // fallback
        }
    }

    double actualWeight = (LOADCELL2_DOUT_PIN != -1) ? ((final1 + final2) / 2.0) : final1;
    // Apply a small compensation for adhered grounds observed in some setups.
    if (startedGrindingAt > 0) {
        actualWeight += display_compensation_g;
        Serial.printf("[AUTO ADJUST] Applied +%.2fg compensation to actualWeight to account for stuck grounds\n", display_compensation_g);
    }

    double targetTotalWeight = setWeight + cupWeightEmpty;
    double weightError = targetTotalWeight - actualWeight;

#if defined(AUTO_OFFSET_ADJUSTMENT) && AUTO_OFFSET_ADJUSTMENT
    if (ABS(weightError) > 0.3) { // Only adjust if error is significant (>0.3g)
        double oldShotOffset = shotOffset;
        shotOffset += weightError;

        // Constrain shotOffset to reasonable limits
        if (shotOffset > 10.0) shotOffset = 10.0;
        if (shotOffset < -10.0) shotOffset = -10.0;

        Serial.printf("AUTO SHOT OFFSET ADJUSTMENT:\n");
        Serial.printf("  Target: %.1fg, Actual: %.1fg, Error: %.1fg\n", 
                     targetTotalWeight, actualWeight, weightError);
        Serial.printf("  Old shotOffset: %.2fg -> New shotOffset: %.2fg\n", 
                     oldShotOffset, shotOffset);

        shotCount++;
        preferences.begin("scale", false);
        preferences.putDouble("shotOffset", shotOffset);
        preferences.putUInt("shotCount", shotCount);
        preferences.end();
    } else {
        Serial.printf("Grinding accuracy good (error: %.1fg) - no shotOffset adjustment needed\n", weightError);
        shotCount++;
        preferences.begin("scale", false);
        preferences.putUInt("shotCount", shotCount);
        preferences.end();
    }
#else
    // Auto-offset adjustment disabled - just increment shot count
    shotCount++;
    preferences.begin("scale", false);
    preferences.putUInt("shotCount", shotCount);
    preferences.end();
#endif

    newOffset = false;
    Serial.println("applyShotOffsetAdjustmentOnExit: finished");
}
