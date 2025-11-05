#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.hpp"
#include "rotary.hpp"
#include "display.hpp"
#include "scale.hpp"

// Rotary encoder for user input
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
    ROTARY_ENCODER_A_PIN,
    ROTARY_ENCODER_B_PIN,
    ROTARY_ENCODER_BUTTON_PIN,
    ROTARY_ENCODER_VCC_PIN,
    ROTARY_ENCODER_STEPS);

// Vars
int encoderDir = -1;   // Direction of the rotary encoder: 1 = normal, -1 = reversed
int encoderValue = 0; // Current value of the rotary encoder
static int clickCount = 0;
const unsigned long clickThreshold = 500; // 500ms max interval for rapid clicks

static void handleSingleClickTask(void *param) {
    bool *pendingFlag = reinterpret_cast<bool *>(param);
    delay(300); // Wait for the delay period

    // Ensure the task only acts if `scaleStatus` is valid for the menu
    if (*pendingFlag && scaleStatus == STATUS_EMPTY) {
        *pendingFlag = false;
        Serial.println("Single click detected. Opening menu...");
        scaleStatus = STATUS_IN_MENU;
        currentMenuItem = 0;
        rotaryEncoder.setAcceleration(0);
        Serial.println("Entering Menu...");
    }
    vTaskDelete(NULL); // End the task
}

// Incase you can't set something you can exit
void exitToMenu()
{
    if (scaleStatus == STATUS_IN_SUBMENU || scaleStatus == STATUS_INFO_MENU)
    {
        if (currentSubmenu == 0) {
            // Return to main menu if we were in a main menu setting
            scaleStatus = STATUS_IN_MENU;
        } else {
            // Return to appropriate submenu if we were in a submenu setting
            scaleStatus = STATUS_IN_MENU;
        }
        currentSetting = -1;
        Serial.println("Exiting to menu");
    }
    else if (scaleStatus == STATUS_IN_MENU)
    {
        if (currentSubmenu != 0) {
            // Go back to main menu from submenu
            currentSubmenu = 0;
            currentSubmenuItem = 0;
            Serial.println("Returning to main menu");
        } else {
            // Exit to empty state from main menu
            scaleStatus = STATUS_EMPTY;
            currentMenuItem = 0;
            currentSubmenu = 0;
            currentSubmenuItem = 0;
            Serial.println("Exiting to empty state");
        }
    }
}

// Handles button clicks on the rotary encoder

// Task function to unlock display after taring
static void unlockDisplayTask(void *param) {
    delay(2000); // Wait 2 seconds
    displayLock = false;
    showingTaringMessage = false;
    vTaskDelete(NULL);
}

void rotary_onButtonClick()
{
    // Don't process button clicks while display is locked
    if (displayLock) {
        return;
    }
    // If we're currently showing the finished screen, a single button press
    // should exit that screen and perform the shotOffset auto-adjust (if
    // pending). This moves the adjustment trigger from the timer to an
    // explicit user action.
    if (scaleStatus == STATUS_GRINDING_FINISHED) {
        Serial.println("Button press while in FINISHED state: running shotOffset adjustment and exiting...");
        applyShotOffsetAdjustmentOnExit();
        // Reset grinding timestamps and transition to empty state
        startedGrindingAt = 0;
        finishedGrindingAt = 0;
        scaleStatus = STATUS_EMPTY;
        display_compensate_shot = false;
        return;
    }
    
    unsigned long currentTime = millis();
    static unsigned long lastTimePressed = 0;      // Timestamp of the last button press
    const unsigned long clickDelay = 300;          // Delay to differentiate single vs double click (in ms)
    static bool menuPending = false;               // Flag to track if a single click action is pending

    // Handle rapid clicks for double-click detection
    if (currentTime - lastTimePressed < clickThreshold)
    {
        clickCount++;
    }
    else
    {
        clickCount = 1; // Reset click count if too much time has passed
    }
    lastTimePressed = currentTime;
    if (clickCount == 2)
    {
        menuPending = false; // Cancel pending single click action
        Serial.println("Double press detected. Taring scale...");
        
        // Reset click count immediately to prevent loops
        clickCount = 0;
        
        // Exit menu if we're in any menu state
        if (scaleStatus == STATUS_IN_MENU || scaleStatus == STATUS_IN_SUBMENU) {
            scaleStatus = STATUS_EMPTY;
            currentMenuItem = 0;
            currentSetting = -1;
            rotaryEncoder.setAcceleration(100); // Restore encoder acceleration
            Serial.println("Exited menu due to tare operation");
        }
        
        // Show taring message on display (non-blocking)
        displayLock = true;
        showTaringMessage();
        
        // Perform the tare operation
        if (!tareScale()) {
            Serial.println("Tare failed: HX711 not ready. Returning to menu.");
            showErrorMessage("Tare failed\nHX711 not ready");
            displayLock = false;
            scaleStatus = STATUS_IN_MENU;
            return;
        }
        
        // Use a task to unlock the display after a delay instead of relying on rotary_loop
        xTaskCreatePinnedToCore(
            unlockDisplayTask,
            "UnlockDisplayTask",
            1000,
            NULL,
            1,
            NULL,
            1
        );
        
        return;
    }

    // Delay single click action to allow for double-click detection
    if (!menuPending && scaleStatus == STATUS_EMPTY)
    {
        menuPending = true; // Set pending flag

        // Pass a pointer to `menuPending` as a parameter
        xTaskCreatePinnedToCore(
            handleSingleClickTask, // Task function
            "SingleClickDelay",    // Task name
            1000,                  // Stack size
            &menuPending,          // Parameter (pointer to menuPending)
            1,                     // Priority
            NULL,                  // Task handle (can be NULL)
            1                      // Core ID
        );
    }

    // Process menu navigation based on current state
    if (scaleStatus == STATUS_EMPTY)
    {
        // Enter the menu when the scale is empty
        scaleStatus = STATUS_IN_MENU;
        currentMenuItem = 0;
        rotaryEncoder.setAcceleration(0);
        Serial.println("Entering Menu...");
    }
    else if (scaleStatus == STATUS_IN_MENU)
    {
        // Navigate through the menu items based on current submenu
        if (currentSubmenu == 0) // Main menu
        {
            switch (currentMenuItem)
            {
            case 0: // Exit
                menuPending = false;                // Reset pending flag
                scaleStatus = STATUS_EMPTY;         // Reset to the empty state
                currentMenuItem = 0;                // Reset menu index
                currentSubmenu = 0;                 // Reset submenu
                currentSubmenuItem = 0;             // Reset submenu item
                rotaryEncoder.setAcceleration(100); // Restore encoder acceleration
                Serial.println("Exited Menu to main screen");
                delay(200); // Debounce to prevent immediate re-trigger
                break;
            case 1: // Mode submenu
                currentSubmenu = 1;
                currentSubmenuItem = 0;
                Serial.println("Entering Mode submenu");
                break;
            case 2: // Shot Offset Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 2;
                Serial.println("Shot Offset Menu");
                break;
            case 3: // Info Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 5;
                Serial.println("Info Menu");
                break;
            case 4: // Configuration submenu
                currentSubmenu = 2;
                currentSubmenuItem = 0;
                Serial.println("Entering Configuration submenu");
                break;
            }
        }
        else if (currentSubmenu == 1) // Mode submenu
        {
            switch (currentSubmenuItem)
            {
            case 0: // GBW mode
                manualGrindMode = false;
                preferences.begin("scale", false);
                preferences.putBool("manualGrindMode", manualGrindMode);
                preferences.end();
                displayLock = true;
                showModeChangeMessage("GBW", "Selected");
                xTaskCreatePinnedToCore(
                    unlockDisplayTask,
                    "ModeChangeDisplayTask",
                    1000,
                    NULL,
                    1,
                    NULL,
                    1
                );
                currentSubmenu = 0; // Return to main menu
                currentMenuItem = 0;
                Serial.println("GBW mode selected");
                break;
            case 1: // Manual mode
                manualGrindMode = true;
                preferences.begin("scale", false);
                preferences.putBool("manualGrindMode", manualGrindMode);
                preferences.end();
                displayLock = true;
                showModeChangeMessage("Manual", "Selected");
                xTaskCreatePinnedToCore(
                    unlockDisplayTask,
                    "ModeChangeDisplayTask",
                    1000,
                    NULL,
                    1,
                    NULL,
                    1
                );
                currentSubmenu = 0; // Return to main menu
                currentMenuItem = 0;
                Serial.println("Manual mode selected");
                break;
            case 2: // Back
                currentSubmenu = 0; // Return to main menu
                currentSubmenuItem = 0;
                Serial.println("Returning to main menu from Mode submenu");
                break;
            }
        }
        else if (currentSubmenu == 2) // Configuration submenu
        {
            switch (currentSubmenuItem)
            {
            case 0: // Calibration Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 1;
                if (!tareScale()) {
                    Serial.println("Tare failed: HX711 not ready. Returning to menu.");
                    showErrorMessage("Tare failed\nHX711 not ready");
                    scaleStatus = STATUS_IN_MENU;
                    break;
                }
                Serial.println("Calibration Menu");
                break;
            case 1: // Compensation Menu (new dedicated item)
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 9; // new submenu id for Compensation
                Serial.println("Compensation Menu");
                break;
            case 2: // Cup Weight Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 0;
                if (!tareScale()) {
                    Serial.println("Tare failed: HX711 not ready. Returning to menu.");
                    showErrorMessage("Tare failed\nHX711 not ready");
                    scaleStatus = STATUS_IN_MENU;
                    break;
                }
                delay(500);  // Wait for stabilization

                if (scaleWeight > 0)
                {
                    setCupWeight = scaleWeight;
                    preferences.begin("scale", false);
                    preferences.putDouble("cup", setCupWeight);
                    preferences.end();

                    Serial.println("Cup weight set successfully");
                }
                else
                {
                    Serial.println("Error: Invalid cup weight detected");
                }
                break;
            case 3: // Scale Mode Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 3;
                Serial.println("Scale Mode Menu");
                break;
            case 4: // Grinding Mode Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 4;
                Serial.println("Grind Mode Menu");
                break;
            case 5: // Grind Trigger Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 8; // Use setting 8 for grind trigger
                Serial.println("Grind Trigger Menu");
                break;
            case 6: // Reset Menu
                scaleStatus = STATUS_IN_SUBMENU;
                currentSetting = 6;
                Serial.println("Reset Menu");
                break;
            case 7: // Back
                currentSubmenu = 0; // Return to main menu
                currentSubmenuItem = 0;
                Serial.println("Returning to main menu from Configuration submenu");
                break;
            }
        }
    }
    else if (scaleStatus == STATUS_IN_SUBMENU)
    {
        // Handle submenu actions based on the current setting
        switch (currentSetting)
        {
        case 0: // Cup Weight Menu
        {
            if (scaleWeight > 5)
            { // Ensure cup weight is valid
                setCupWeight = scaleWeight;
                Serial.println(setCupWeight);

                preferences.begin("scale", false);
                preferences.putDouble("cup", setCupWeight);
                preferences.end();

                displayLock = true;
                showCupWeightSetScreen(setCupWeight); // Show confirmation
                displayLock = false;

                exitToMenu();
            }
            else
            {
                Serial.println("Error: Invalid cup weight detected. Setting default value.");
                setCupWeight = 10.0; // Assign a reasonable default value
                preferences.begin("scale", false);
                preferences.putDouble("cup", setCupWeight);
                preferences.end();
                Serial.println("Failsafe: Exiting cup weight menu due to zero weight");
                exitToMenu();
            }
            break;
        }
        case 1: // Calibration Menu
        {
            // Get the raw reading from the load cell (without scale factor applied)
            loadcell.set_scale(1.0); // Temporarily set scale to 1 to get raw reading
            delay(500); // Allow stabilization
            double rawReading = loadcell.get_units(10); // Take 10 readings for accuracy
            
            // Calculate the correct calibration factor: rawReading / knownWeight
            double newCalibrationValue = rawReading / 100.0; // 100g known weight
            
            // Basic validation - ensure we got a reasonable reading
            if (abs(rawReading) < 1000 || abs(newCalibrationValue) < 100 || abs(newCalibrationValue) > 10000) {
                Serial.printf("Error: Invalid calibration values (raw: %.2f, factor: %.2f). Using default.\n", 
                             rawReading, newCalibrationValue);
                newCalibrationValue = (double)LOADCELL_SCALE_FACTOR;
            }
            
            // Save and apply the new calibration
            preferences.begin("scale", false);
            preferences.putDouble("calibration", newCalibrationValue);
            preferences.end();
            
            loadcell.set_scale(newCalibrationValue);
            
            Serial.printf("Calibration completed: Raw reading = %.2f, New scale factor = %.2f\n", 
                         rawReading, newCalibrationValue);
            // Persist calibration and current display compensation value
            preferences.begin("scale", false);
            preferences.putDouble("calibration", newCalibrationValue);
            preferences.putDouble("displayCompensation", display_compensation_g);
            preferences.end();

            scaleStatus = STATUS_IN_MENU;
            currentSetting = -1;
            break;
        }
        case 2: // Shot Offset Menu
        {
            preferences.begin("scale", false);
            preferences.putDouble("shotOffset", shotOffset);
            preferences.end();
            scaleStatus = STATUS_IN_MENU;
            currentSetting = -1;
            break;
        }
        case 9: // Compensation Menu - persist and exit
        {
            preferences.begin("scale", false);
            preferences.putDouble("displayCompensation", display_compensation_g);
            preferences.end();
            Serial.printf("Compensation saved: %.1fg\n", display_compensation_g);
            scaleStatus = STATUS_IN_MENU;
            currentSetting = -1;
            break;
        }
        case 3: // Scale Mode Menu
        {
            preferences.begin("scale", false);
            preferences.putBool("scaleMode", scaleMode);
            preferences.end();
            scaleStatus = STATUS_IN_MENU;
            currentSetting = -1;
            break;
        }
        case 4: // Grinding Mode Menu
        {
            preferences.begin("scale", false);
            preferences.putBool("grindMode", grindMode);
            preferences.end();
            scaleStatus = STATUS_IN_MENU;
            currentSetting = -1;
            break;
        }
        case 5: // Info Menu
        {
            displayLock = true;
            showInfoMenu(); // Display info menu
            delay(3000);
            displayLock = false;
            exitToMenu();
            break;
        }
        case 6: // Reset Menu
        {
            if (greset)
            {
                preferences.begin("scale", false);
                preferences.putDouble("calibration", (double)LOADCELL_SCALE_FACTOR);
                setWeight = (double)COFFEE_DOSE_WEIGHT;
                preferences.putDouble("setWeight", (double)COFFEE_DOSE_WEIGHT);
                   shotOffset = (double)COFFEE_DOSE_OFFSET;
                   preferences.putDouble("shotOffset", (double)COFFEE_DOSE_OFFSET);
                setCupWeight = (double)CUP_WEIGHT;
                preferences.putDouble("cup", (double)CUP_WEIGHT);
                scaleMode = false;
                preferences.putBool("scaleMode", false);
                grindMode = false;
                preferences.putBool("grindMode", false);
                preferences.putUInt("shotCount", 0);
                loadcell.set_scale((double)LOADCELL_SCALE_FACTOR);
                preferences.end();
            }
            scaleStatus = STATUS_IN_MENU;
            currentSetting = -1;
            break;
        }
        case 8: // Grind Trigger Menu
        {
            // Save the current selection and exit
            preferences.begin("scale", false);
            preferences.putBool("grindTrigger", useButtonToGrind);
            preferences.end();
            Serial.print("Grind Trigger Mode set to: ");
            Serial.println(useButtonToGrind ? "Button" : "Cup");
            scaleStatus = STATUS_IN_MENU;
            currentSetting = -1;
            break;
        }
        }
    }
}

// Handles rotary encoder input for menu navigation and adjustments
void rotary_loop()
{
    // Long press detection using direct GPIO reading
    static unsigned long buttonPressStartTime = 0;
    static bool longPressProcessed = false;
    const unsigned long longPressThreshold = 3000; // 3 seconds for long press
    
    bool buttonCurrentlyPressed = (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW); // Assuming active LOW
    unsigned long currentTime = millis();
    
    if (buttonCurrentlyPressed) {
        if (buttonPressStartTime == 0) {
            // Button just pressed
            buttonPressStartTime = currentTime;
            longPressProcessed = false;
        } else if (currentTime - buttonPressStartTime >= longPressThreshold && !longPressProcessed && !displayLock) {
            // Long press detected
            longPressProcessed = true;
            manualGrindMode = !manualGrindMode;
            
            Serial.print("Long press detected - Manual Grind Mode: ");
            Serial.println(manualGrindMode ? "ENABLED" : "DISABLED");
            
            // Save the setting
            preferences.begin("scale", false);
            preferences.putBool("manualGrindMode", manualGrindMode);
            preferences.end();
            
            // Show mode change on display briefly
            displayLock = true;
            if (manualGrindMode) {
                showModeChangeMessage("Manual Mode", "Enabled");
            } else {
                showModeChangeMessage("GBW Mode", "Enabled");
            }
            
            // Create task to unlock display after showing message
            xTaskCreatePinnedToCore(
                unlockDisplayTask,
                "ModeChangeDisplayTask",
                1000,
                NULL,
                1,
                NULL,
                1
            );
        }
    } else {
        // Button released
        if (buttonPressStartTime > 0) {
            buttonPressStartTime = 0;
            longPressProcessed = false;
        }
    }
    
    if (rotaryEncoder.encoderChanged())
    {
        // Wake the screen if it's asleep
        if (millis() - lastSignificantWeightChangeAt > sleepTime)
        {
            Serial.println("Screen waking due to rotary movement...");
            wakeScreen();
        }
        switch (scaleStatus)
        {
        case STATUS_EMPTY:
        {
            if (screenJustWoke)
            {
                // Skip modifying the set weight if the screen just woke up
                screenJustWoke = false; // Reset the flag
                break;
            }
            // Adjust weight when in scale mode
            if (setWeight < 0)
            {
                setWeight = 0;
                Serial.println("Grind weight cannot be less than 0. Reset to 0.");
            }
            int newValue = rotaryEncoder.readEncoder();
            int encoderDelta = newValue - encoderValue;
            
            // Process encoder changes - make each detent = 0.1g
            if (encoderDelta != 0) {
                // If 6 detents was giving 0.1g, then each detent was 0.0167g
                // To make each detent = 0.1g, multiply by 6
                float increment = (float)encoderDelta * 0.1 * encoderDir;
                setWeight += increment;
                if (setWeight < 0) setWeight = 0; // Prevent negative values
                
                // Round to nearest 0.1g for display consistency
                setWeight = round(setWeight * 10.0) / 10.0;
                
                encoderValue = newValue;
                preferences.begin("scale", false);
                preferences.putDouble("setWeight", setWeight);
                preferences.end();
                
                Serial.print("Weight: ");
                Serial.print(setWeight, 1);
                Serial.print("g (delta: ");
                Serial.print(encoderDelta);
                Serial.print(", increment: ");
                Serial.print(increment, 3);
                Serial.println(")");
            }
            break;
        }
        case STATUS_IN_MENU:
        {
            // Navigate through menu items based on current submenu
            int newValue = rotaryEncoder.readEncoder();
            int encoderDelta = newValue - encoderValue;
            
            // Only process if there's a significant change
            if (abs(encoderDelta) >= 1) {
                // Apply encoder direction consistently
                int menuDirection = (encoderDelta > 0 ? 1 : -1) * encoderDir;
                
                if (currentSubmenu == 0) {
                    // Main menu navigation
                    currentMenuItem = (currentMenuItem + menuDirection) % menuItemsCount;
                    currentMenuItem = currentMenuItem < 0 ? menuItemsCount + currentMenuItem : currentMenuItem;
                    Serial.print("Main menu item: ");
                    Serial.println(currentMenuItem);
                } else if (currentSubmenu == 1) {
                    // Mode submenu navigation
                    currentSubmenuItem = (currentSubmenuItem + menuDirection) % modeMenuItemsCount;
                    currentSubmenuItem = currentSubmenuItem < 0 ? modeMenuItemsCount + currentSubmenuItem : currentSubmenuItem;
                    Serial.print("Mode submenu item: ");
                    Serial.println(currentSubmenuItem);
                } else if (currentSubmenu == 2) {
                    // Configuration submenu navigation
                    currentSubmenuItem = (currentSubmenuItem + menuDirection) % configMenuItemsCount;
                    currentSubmenuItem = currentSubmenuItem < 0 ? configMenuItemsCount + currentSubmenuItem : currentSubmenuItem;
                    Serial.print("Config submenu item: ");
                    Serial.println(currentSubmenuItem);
                }
                
                encoderValue = newValue;
            }
            break;
        }
        case STATUS_IN_SUBMENU:
        {
            int newValue = rotaryEncoder.readEncoder();
            int encoderDelta = newValue - encoderValue;
            
            // Allow encoder to adjust display compensation when in the
            // Compensation submenu (currentSetting == 9). Each detent = 0.1g.
            if (currentSetting == 9 && encoderDelta != 0) {
                display_compensation_g += (double)encoderDelta * 0.1 * encoderDir;
                encoderValue = newValue;
                // Bound to reasonable limits
                if (display_compensation_g < 0.0) display_compensation_g = 0.0;
                if (display_compensation_g > 20.0) display_compensation_g = 20.0;
                // Round to 0.1g
                display_compensation_g = round(display_compensation_g * 10.0) / 10.0;
                Serial.printf("Display compensation: %.1fg\n", display_compensation_g);
            }

            if (currentSetting == 2 && encoderDelta != 0)
            { // Shot offset menu - make each detent = 0.01g
                shotOffset += (float)encoderDelta * 0.01 * encoderDir;
                encoderValue = newValue;
                if (abs(shotOffset) >= setWeight)
                {
                    shotOffset = setWeight; // Prevent nonsensical offsets
                }
                // Round to nearest 0.01g
                shotOffset = round(shotOffset * 100.0) / 100.0;
                
                Serial.print("ShotOffset: ");
                Serial.print(shotOffset, 2);
                Serial.print("g (delta: ");
                Serial.print(encoderDelta);
                Serial.println(")");
            }
            else if (currentSetting == 3)
            {
                scaleMode = !scaleMode;
            }
            else if (currentSetting == 4)
            {
                grindMode = !grindMode;
            }
            else if (currentSetting == 6)
            {
                greset = !greset;
            }
            else if (currentSetting == 8) // Grind Trigger Menu - selector style
            {
                useButtonToGrind = !useButtonToGrind;
            }
            break;
        }
        case STATUS_GRINDING_FAILED:
        {
            Serial.println("Exiting Grinding Failed state to Main Menu...");
            scaleStatus = STATUS_IN_MENU;
            currentMenuItem = 0; // Reset to the main menu
            return; // Exit early to avoid further processing
        }
        }
    }
    if (rotaryEncoder.isEncoderButtonClicked())
    {
        // Don't process button clicks while display is locked
        if (displayLock) {
            return;
        }
        
        // Wake the screen if it's asleep
        if (millis() - lastSignificantWeightChangeAt > sleepTime)
        {
            Serial.println("Screen waking due to button press...");
            wakeScreen();
            return; // Exit early to prevent other button actions while waking
        }
        
        rotary_onButtonClick(); // Existing button click handling
    }
}

// ISR for reading encoder changes
void readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}