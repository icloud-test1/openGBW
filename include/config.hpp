#pragma once

#include <SimpleKalmanFilter.h>
#include "HX711.h"
#include <MathBuffer.h>
#include <AiEsp32RotaryEncoder.h>
#include <Preferences.h>
#include <SPI.h>
#include <U8g2lib.h>
#include "menu_item.hpp"

// Declarations of global variables (no memory allocation here)
extern Preferences preferences;       // Preferences object
extern HX711 loadcell;                // HX711 load cell object
extern HX711 loadcell2;               // Optional second HX711 load cell object
extern SimpleKalmanFilter kalmanFilter; // Kalman filter for smoothing weight measurements

extern TaskHandle_t ScaleTask;        // Task handle for the scale task
extern TaskHandle_t ScaleStatusTask;  // Task handle for the scale status task

//Set your sleep variable
#define SLEEP_AFTER_MS 60000

//Main Variables and Pins
#define STATUS_EMPTY 0
#define STATUS_GRINDING_IN_PROGRESS 1
#define STATUS_GRINDING_FINISHED 2
#define STATUS_GRINDING_FAILED 3
#define STATUS_IN_MENU 4
#define STATUS_IN_SUBMENU 5
#define STATUS_INFO_MENU 8

#define CUP_WEIGHT 70
#define CUP_DETECTION_TOLERANCE 5 // 5 grams tolerance above or bellow cup weight to detect it

#define LOADCELL_DOUT_PIN 5 // primary HX711 DOUT (sensor1) -> use GPIO5
#define LOADCELL_SCK_PIN 18

#define LOADCELL_SCALE_FACTOR 4362.59  // Calibrated 2025-10-29

#define LOADCELL2_DOUT_PIN 16 // secondary HX711 DOUT (sensor2) -> use GPIO16
// Use same SCK pin for both HX711 modules (shared clock)
#define LOADCELL2_SCK_PIN LOADCELL_SCK_PIN
#define LOADCELL2_SCALE_FACTOR 4362.59 // Default fallback, can be calibrated separately

#define TARE_MEASURES 20 // use the average of measure for taring
#define SIGNIFICANT_WEIGHT_CHANGE 5 // 5 grams changes are used to detect a significant change
#define COFFEE_DOSE_WEIGHT 18
#define COFFEE_DOSE_OFFSET -2.5
#define MAX_GRINDING_TIME 20000 // 20 seconds diff
#define GRINDING_FAILED_WEIGHT_TO_RESET 150 // force on balance need to be measured to reset grinding

#define GRINDER_ACTIVE_PIN 14

#define GRIND_BUTTON_PIN 25
#define DEFAULT_GRIND_TRIGGER_MODE true  // true = use button, false = cup detection
#define AUTO_OFFSET_ADJUSTMENT true     // Enable automatic offset adjustment after grinding

#define TARE_MIN_INTERVAL 10 * 1000 // auto-tare at most once every 10 seconds

#define ROTARY_ENCODER_A_PIN 23
#define ROTARY_ENCODER_B_PIN 32
#define ROTARY_ENCODER_BUTTON_PIN 27
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

// Screen 
#define OLED_SDA 21
#define OLED_SCL 22

// External User Variables
extern volatile bool displayLock; // Add this declaration
extern double scaleWeight;
extern unsigned long scaleLastUpdatedAt;
extern unsigned long lastSignificantWeightChangeAt;
extern unsigned long lastTareAt;
extern bool scaleReady;
extern int scaleStatus;
extern double cupWeightEmpty;
extern unsigned long startedGrindingAt;
extern unsigned long finishedGrindingAt;
extern double setWeight;
// shotOffset: grams adjustment applied after a grind (used to bias target to
// compensate for grinder runout). This is different from the HX711 raw
// tare counts which are stored separately as loadcell offsets.
extern double shotOffset;
// Display compensation for stuck/adhered grounds (grams)
extern double display_compensation_g;
// When true, the display adds the above compensation while on the finished screen
extern bool display_compensate_shot;
// Optional micro-vibe setting to pulse grinder after grind
extern bool auto_vibe_after_grind;
// Primary HX711 raw offset (counts). Secondary offset is `loadcell2_offset`.
extern long loadcell_offset;
extern bool scaleMode;
extern bool grindMode;
extern bool greset;
extern double scaleWeight2; // secondary load cell computed weight (grams)
extern double scaleFactor2; // secondary scale factor (counts/gram)
extern long loadcell2_offset; // secondary load cell offset (counts)
extern unsigned long aztBlockUntil; // timestamp (ms) until AZT (auto-zero tracking) is blocked
extern int menuItemsCount;
extern double setCupWeight;
extern MenuItem menuItems[];
extern int currentMenuItem;
extern int currentSetting;
extern int currentSubmenu; // 0 = main menu, 1 = mode submenu, 2 = config submenu
extern int currentSubmenuItem;
extern int modeMenuItemsCount;
extern int configMenuItemsCount;
extern MenuItem modeMenuItems[];
extern MenuItem configMenuItems[];
extern int sleepTime;
extern bool screenJustWoke;
extern unsigned int shotCount;
extern bool useButtonToGrind;
extern bool showingTaringMessage;
extern unsigned long taringMessageStartTime;
extern bool manualGrindMode;