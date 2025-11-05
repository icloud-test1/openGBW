#include <Arduino.h>
#include "HX711.h"

// Adjust pins if your wiring uses different GPIOs
#define LOADCELL_DOUT_PIN 16
#define LOADCELL_SCK_PIN 18
#define SCALE_FACTOR 4264.66  // Calibrated with 48.1g reference weight on 2025-10-28
#define HISTORY_SIZE 20       // Number of samples to keep for stability detection
#define STABLE_THRESHOLD 2000  // Max variation in raw counts to consider stable (about 0.5g)
#define STABLE_REQUIRED 5     // Number of consecutive stable readings required
#define MOVEMENT_THRESHOLD 4000  // Raw count difference that indicates movement

HX711 loadcell;

// forward declaration for manual bit-bang read used in setup
long manual_read_hx711(int doutPin, int sckPin);

bool checkStability(long* samples, int count, float* average) {
    if (count < HISTORY_SIZE) return false;
    
    long sum = 0;
    long min_val = samples[0];
    long max_val = samples[0];
    
    for(int i = 0; i < count; i++) {
        sum += samples[i];
        min_val = min(min_val, samples[i]);
        max_val = max(max_val, samples[i]);
    }
    
    *average = sum / (float)count;
    return (max_val - min_val) < STABLE_THRESHOLD;
}

void printHelp() {
    Serial.println("\nCommands:");
    Serial.println("t - Tare (zero) the scale");
    Serial.println("c - Enter calibration mode");
    Serial.println("w [weight] - In calibration mode, set known weight in grams");
    Serial.println("h - Show this help");
    Serial.println("s - Show current statistics");
    Serial.println("f [factor] - Set active scale factor (counts/gram), e.g., f3670.49");
    Serial.println("a - Toggle Auto-Zero Tracking (AZT) on/off\n");
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\nHX711 enhanced test & calibration utility");
    printHelp();
  pinMode(LOADCELL_DOUT_PIN, INPUT);
  pinMode(LOADCELL_SCK_PIN, OUTPUT);
  digitalWrite(LOADCELL_SCK_PIN, LOW);
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  // Run an automatic pin scan to detect possible mis-wiring
  Serial.println("Starting automatic pin-scan (probing common GPIOs)...");
  int doutCandidates[] = {16, 17, 4, 5, 23};
  int sckCandidates[] = {4, 5, 16, 17, 18, 23};
  for (int i = 0; i < (int)(sizeof(sckCandidates)/sizeof(sckCandidates[0])); ++i) {
    int sck = sckCandidates[i];
    for (int j = 0; j < (int)(sizeof(doutCandidates)/sizeof(doutCandidates[0])); ++j) {
      int dout = doutCandidates[j];
      if (dout == sck) continue; // skip invalid same-pin case
      // Reconfigure pins for this test
      pinMode(sck, OUTPUT);
      digitalWrite(sck, LOW);
      pinMode(dout, INPUT);
      // Initialize library on these pins
      loadcell.begin(dout, sck);
      delay(50);
      bool ready = loadcell.wait_ready_timeout(200);
      long lib_r = 0; long lib_avg = 0; long manual_r = 0; long off = 0;
      if (ready) {
        lib_r = loadcell.read();
        lib_avg = loadcell.read_average(5);
        off = loadcell.get_offset();
        manual_r = manual_read_hx711(dout, sck);
      }
      Serial.printf("probe dout=%d sck=%d | ready=%s | lib=%ld avg=%ld manual=%ld off=%ld\n",
                    dout, sck, ready?"true":"false", lib_r, lib_avg, manual_r, off);
      delay(30);
    }
  }
  // Re-init to configured pins
  loadcell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

// Manual bit-bang read of HX711 (24-bit two's complement)
long manual_read_hx711(int doutPin, int sckPin) {
  // Wait for ready (DOUT low)
  unsigned long t0 = millis();
  while (digitalRead(doutPin) == HIGH) {
    if (millis() - t0 > 1000) return 0x7FFFFF; // timeout marker
    delay(1);
  }
  uint32_t value = 0;
  for (int i = 0; i < 24; ++i) {
    digitalWrite(sckPin, HIGH);
    delayMicroseconds(1);
    int bit = digitalRead(doutPin);
    value = (value << 1) | (bit & 0x1);
    digitalWrite(sckPin, LOW);
    delayMicroseconds(1);
  }
  // Pulse the clock once to set channel/gain (128)
  digitalWrite(sckPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(sckPin, LOW);

  // Convert from 24-bit two's complement to signed long
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  return (long)value;
}
// Circular buffer for raw readings and stability detection
long raw_history[HISTORY_SIZE];
int history_index = 0;
unsigned long last_reading_time = 0;
unsigned long readings_count = 0;
unsigned long total_time = 0;
int stable_count = 0;
float last_stable_average = 0;
bool calibration_mode = false;
long tare_value = 0;
bool tare_complete = false;
// Runtime-adjustable scale factor (counts per gram). Initialized from macro.
float scale_factor = SCALE_FACTOR;
bool history_seeded = false; // seed history on first successful read
// Auto-Zero Tracking (helps correct residual offsets when near zero)
bool auto_zero_enabled = true;
int auto_zero_stable = 0;
const float AZT_WINDOW_G = 1.0f;   // grams window considered near zero
const int AZT_REQUIRED = 8;        // consecutive stable cycles required to auto-zero

void loop() {
    static float known_weight = 0;
    
    // Check for serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 't':
                tare_complete = false;
                calibration_mode = false;
                stable_count = 0;
                Serial.println("\nTaring - please wait for stability...");
                break;
            case 'c':
                if (!tare_complete) {
                    Serial.println("\nError: Please tare first!");
                    Serial.println("Type 't' to tare the scale");
                } else {
                    calibration_mode = true;
                    stable_count = 0;
                    Serial.println("\nCalibration mode - place known weight and use 'w [grams]'");
                    Serial.println("For example: w48.1");
                    Serial.println("Waiting for input...");
                }
                break;
            case 'w':
                if (calibration_mode) {
                    known_weight = Serial.parseFloat();
                    stable_count = 0;
                    Serial.printf("Using %.1fg as reference. Waiting for stability...\n", known_weight);
                } else {
                    Serial.println("Error: Enter calibration mode first!");
                }
                break;
            case 's':
                Serial.printf("\nCurrent status:\n");
                Serial.printf("Tare complete: %s\n", tare_complete ? "yes" : "no");
                Serial.printf("Calibration mode: %s\n", calibration_mode ? "yes" : "no");
                Serial.printf("Build-time factor (SCALE_FACTOR): %.2f\n", SCALE_FACTOR);
                Serial.printf("Active factor (runtime): %.2f\n", scale_factor);
                Serial.printf("Auto-Zero Tracking: %s (window=%.1fg, require=%d stable)\n", auto_zero_enabled?"on":"off", AZT_WINDOW_G, AZT_REQUIRED);
                Serial.printf("Last stable reading: %.1f\n", last_stable_average);
                break;
            case 'f': {
                // Set factor manually: e.g., f3670.49 or f 3670.49
                float new_f = Serial.parseFloat();
                if (new_f > 100.0f && new_f < 20000.0f) { // sanity range for counts/gram
                    scale_factor = new_f;
                    Serial.printf("Active scale factor set to %.2f counts/gram\n", scale_factor);
                } else {
                    Serial.println("Invalid factor. Expected counts/gram, e.g., f4264.66");
                }
                break;
            }
            case 'a': {
                auto_zero_enabled = !auto_zero_enabled;
                auto_zero_stable = 0;
                Serial.printf("Auto-Zero Tracking is now %s\n", auto_zero_enabled?"ENABLED":"DISABLED");
                break;
            }
        }
        // Flush any remaining characters
        while(Serial.available()) Serial.read();
    }
    
    unsigned long start_time = micros();
    
    // Add delay between readings to make serial input easier
    delay(500);  // 500ms delay between readings
    
    if (loadcell.wait_ready_timeout(200)) {
        unsigned long now = millis();
        // Use average of 3 readings for better stability
        long raw = loadcell.read_average(3);
        
        // Seed history buffer on first read to avoid large initial deltas
        if (!history_seeded) {
            for (int i = 0; i < HISTORY_SIZE; ++i) raw_history[i] = raw;
            history_seeded = true;
        }
        // Store raw reading in history
        long prev_raw = raw_history[history_index];
        raw_history[history_index] = raw;
        history_index = (history_index + 1) % HISTORY_SIZE;
        
        float average;
        bool is_stable = checkStability(raw_history, (int)min((long)readings_count + 1, (long)HISTORY_SIZE), &average);
        
        // Check for movement
        if (abs(raw - prev_raw) > MOVEMENT_THRESHOLD) {
            stable_count = 0;
        } else if (is_stable) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        
        // Process stable readings
        if (stable_count >= STABLE_REQUIRED) {
            last_stable_average = average;
            
            if (!tare_complete) {
                tare_value = average;
                tare_complete = true;
                Serial.println("Tare complete!");
            }
            else if (calibration_mode && known_weight > 0) {
                float raw_diff = average - tare_value;
                float new_factor = raw_diff / known_weight;
                float reverse_factor = known_weight / raw_diff;
                
                Serial.printf("\nCalibration details:\n");
                Serial.printf("Raw with weight: %.1f\n", average);
                Serial.printf("Raw tare: %.1f\n", (float)tare_value);
                Serial.printf("Raw difference: %.1f counts\n", raw_diff);
                Serial.printf("Known weight: %.1fg\n", known_weight);
                Serial.printf("Counts per gram: %.2f (raw_diff/weight)\n", new_factor);
                Serial.printf("Build-time factor (SCALE_FACTOR): %.2f\n", SCALE_FACTOR);
                Serial.printf("Active factor before apply: %.2f\n", scale_factor);
                Serial.printf("Verification:\n");
                Serial.printf("- Using counts/gram: %.1fg\n", raw_diff / new_factor);
                Serial.printf("- Using current factor: %.1fg\n", raw_diff / SCALE_FACTOR);
                Serial.printf("- Using grams/count: %.1fg\n", raw_diff * reverse_factor);
                Serial.printf("\nPossible scale factors:\n");
                Serial.printf("#define SCALE_FACTOR %.2f  // counts/gram\n", new_factor);
                Serial.printf("#define SCALE_FACTOR %.2f  // grams/count\n", reverse_factor);
                // Apply new factor at runtime immediately
                scale_factor = new_factor;
                Serial.printf("\nApplied active factor: %.2f (runtime). Readings now reflect this.\n", scale_factor);
                calibration_mode = false;
            } else if (auto_zero_enabled) {
                // Auto-zero when near zero and stable for a while (helps with stickiness/creep)
                float near_zero_weight = (average - tare_value) / scale_factor;
                if (fabs(near_zero_weight) <= AZT_WINDOW_G) {
                    auto_zero_stable++;
                    if (auto_zero_stable >= AZT_REQUIRED) {
                        long old_tare = tare_value;
                        tare_value = average; // pull tare to current stable average
                        auto_zero_stable = 0;
                        float adjusted_g = (tare_value - old_tare) / scale_factor;
                        Serial.printf("Auto-zero adjusted tare by %+ld counts (%.2fg)\n", (tare_value - old_tare), adjusted_g);
                    }
                } else {
                    auto_zero_stable = 0;
                }
            }
        }
        
    // Calculate weight using active runtime factor
    float weight = (raw - tare_value) / scale_factor;
        
        // Update timing statistics
        readings_count++;
        unsigned long elapsed = micros() - start_time;
        total_time += elapsed;
        float avg_time = total_time / (float)readings_count;
        float rate = 1000000.0f / avg_time;
        float delta_t = (now - last_reading_time) / 1000.0f;
        last_reading_time = now;
        
        // Only print every other reading to reduce output frequency
        static bool print_this_reading = true;
        if (print_this_reading) {
            Serial.printf("[HX711] t=%.3fs %s raw=%ld Δ=%ld avg=%.1f stable=%d/%d weight=%.1fg\n",
                         delta_t,
                         is_stable ? "STABLE  " : "SETTLING",
                         raw,
                         raw - prev_raw,
                         average,
                         stable_count,
                         STABLE_REQUIRED,
                         weight);
        }
        print_this_reading = !print_this_reading;
    } else {
        Serial.println("HX711 not found or timeout.");
    }
}
