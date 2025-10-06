/*
    PinSim-ESP32 Controller
    Octopilot Electronics

    Based on the excellent PinSim Controller v20200517
    Controller for PC Pinball games
    https://www.youtube.com/watch?v=18EcIxywXHg
    
    Which was based on the excellent MSF_FightStick XINPUT project by Zack "Reaper" Littell
    https://github.com/zlittell/MSF-XINPUT
    
    Uses the ESP32-S3, as built into the PinSim-ESP32 PCB

    IMPORTANT PLUNGER NOTE:
    You MUST calibrate the plunger range at least once by holding down "A"
    when plugging in the USB cable. LED-1 should flash rapidly, and then you should
    pull the plunger all the way out and release it all the way back in. The LED1 should
    flash again, and normal operation resumes. The setting is saved between power cycles.

    If you're not using a plunger, ground the PLUNGE pin.

    Requires installing the following libraries from the Library Manager:
    - Adafruit ADXL345  (pulls in Adafruit BusIO, and Adafruit Unified Sensor)
    - Button2
    - Callback
    - NimBLE-Arduino

    Configuration:
    During Startup Hold Back button to enter Config mode.  Continue holding Back while you press other buttons.
    • While in Config mode:
      - Menu + Guide button LEDs turn on
      - Start button LED flashes fast

    • Press Left Flipper -- Rumble Test (1 blink before starting)
    • Press Right Flipper -- Enable/Disable Solenoids (1 blink for Disabled, 2 for Enabled)
    • Press DPad Down  -- Clear all BLE Pairings (1 blink)
    • Press DPad Up -- Enter pairing mode to add a device (2 blinks)
    • Press DPad Left -- Swap plunger controls (1 blink for Right stick plunger, 2 for Left stick)
    • Press DPad Right -- Prepare for accelerometer calibration (1 blink now to confirm)
      - After exiting config mode, close the lid and press Start to calibrate. (1 blink again)
    • Press Start -- Begin Plunger calibration (1 blink to confirm, 2 blinks after pulling and
      - Reboot with plunger slack/neautral and Back held down
      - Press Start
      - Start LED will immediately blink 1 time
      - Slowly pull plunger all the way out, and then slowly push all the way in.
      - Start will blink 2 times
      - After also releasing Back button, controls will work, but it's still waiting for you to choose the plunger deadzone.
        - For no deadzone, just press Start now, while leaving the plunger alone.
        - Or to add a deadzone, like for Pinball FX VR, pull the plunger out part-way until it just starts to show as moving, and then press Start.
      - Start will now blink 3 times.

    For example:
    - Hold Back while booting
    - Config mode LEDs turn on
    - Press and release DPad Up to allow pairing a new device
    - Start button and Back and Menu blink 2 times
    - Press and release DPad Right to signal that you want to calibrate the accelerometer when you next press Start
    - Buttons blink 1 time
    - Press DPad Left to swap plunger control.
    - Start button blink 2 times, to show that you're now in plunger on Left stick mode
    - Finally Release Back button
    - Config mode LEDs turn off
    - Close LID
    - Gently press Start to calibrate the accelerometer
    - Start button blinks 1 time
    - Done
*/


#include <Arduino.h>
#include <bootloader_random.h>
#include <esp_mac.h>
#include <Preferences.h>
#include <Button2.h>
#include "solenoid.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include "xInput.h"

const char *XB_NAME = "Xbox Wireless Controller";  // "PinSimESP32 Xbox Controller";
const char *XB_MANUFACTURER = "Microsoft";  // "Octopilot Electronics";

// Define this as 2 for v0.2, or 3 for v0.3/v0.4, , or 5 for v0.5 as a few connections have changed
#define PCB_VERSION 5

// LED Brightness for the Red and Green LEDs onboard the PCB.  255 is full, 5 is dim, 0 disables them.
#define PCB_LED_BRIGHTNESS 6

// Button debounce time in milliseconds
#define MILLIDEBOUNCE 20

// Require holding the home button for DELAY_HOME milliseconds to activate it, to avoid disruptive, accidental home button presses.
// Send Start button immediately on press, but after holding for DELAY_L3R3 ms, also start sending L3 and R3 together, to activate the controller on a Quest.
#define DELAY_HOME 5000
#define DELAY_L3R3 2000

#define vTaskDelay_ms(x)  vTaskDelay(pdMS_TO_TICKS(x));


// GLOBAL CONFIGURATION VARIABLES
// configure these
boolean accelerometerEnabled = true;
boolean plungerEnabled = true;
boolean solenoidEnabled = true;

boolean waitingForDeadzoneSetting = false;
boolean waitingForAccelSetting = false;
boolean controlShuffle = false;         // When enabled, move Tilt to D-Pad, and move plunger to Left Stick, to get around a Pinball FX2 VR bug
int16_t nudgeMultiplier = 8000;         // accelerometer multiplier (higher = more sensitive)
int16_t plungeTrigger = 60;             // threshold to trigger a plunge (lower = more sensitive)
int16_t fourButtonModeThreshold = 250;  // ms that pins 13/14 need to close WITHOUT FLIP_L/FLIP_R closing to trigger four flipper button mode.


// Store settings to EEPROM using preferences storage name: PinSimESP32
// Fields: (int)plungerMin, (int)plungerMax, (int)plungerZero,
//         (int)accelZeroX, (int)accelZeroY,
//         (bool)controlShuffle, (bool)solenoidEnabled
// NOTE: field names have max length of 15 chars!!!
static Preferences preferences;

int numSamples = 12;
int plungerAverage = 0;

// Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

XInput gamepad;
boolean handleRumbleCommand(uint8_t command);
void updateTriggerStatus();

TaskHandle_t mainTaskHandle = NULL;
void handle_main_task(void *arg);

int16_t plungerMin = 780;           // default min plunger analog sensor value
int16_t plungerZeroValue = 780;     // zero value for the plunger, to set the deadzone (don't show movement between min and zero)
int16_t plungerMinOffset = 50;      // add this to the measured min value, to give wiggle room for vibrations to not start a pliunge
int16_t plungerMax = 2700;          // default max plunger analog sensor value
int16_t plungerMaxDistance = 0;     // sensor value converted to actual distance
int16_t plungerMinDistance = 0; 
uint32_t tiltEnableTime = 0;
int16_t lastDistance = 0;
int16_t distanceBuffer = 0;
int32_t zeroX = 0;                  // Accelerometer X calibration
int32_t zeroY = 15000;              // Accelerometer Y calibration (non-zero due to tilted cabinet cover)
int32_t accX = 0;
int32_t accY = 0;

// Pin Declarations
#if PCB_VERSION == 2
// PCB Version 0.2
#define pinLEDStart 2   // Start Button LED
#define pinLEDLR    0   // Left/Right Buttons LED
#define pinLEDABC   0   // ABC Buttons LED
#define pinLEDBG    0   // BG Buttons LED
#define pinLEDXYZ   0   // XYZ Buttons LED
#define pinLEDg     6   // PCB LED GREEN
#define pinLEDr     7   // PCB LED RED
#define pinACC_SCL 3    // Accelerometer SCL pin
#define pinRT 4         // Right Flipper
#define pinPlunger 5    // IR distance for plunger
#define rumbleSmall 8   // Large Rumble Motor
#define pinACC_SDA 9    // Accelerometer SDA pin
#define rumbleLarge 10  // Large Rumble Motor
#define pinDpadD 12     // Down on DPAD
#define pinDpadU 14     // Up on DPAD
#define pinB1 15        // Button 1 (A)
#define pinB2 16        // Button 2 (B)
#define pinB3 17        // Button 3 (X)
#define pinB4 18        // Button 4 (Y)
#define pinDpadL 38     // Left on DPAD
#define pinST 39        // Button 8 (Start)
#define pinBK 40        // Button 7 (Back)
#define pinXB 41        // XBOX Guide Button
#define pinLT 42        // Left Flipper
#define pinDpadR 48     // Right on DPAD
#define pinB9 47        // Button 9 (L3)   -- Exposed in GPIO header
#define pinB10 21       // Button 10 (R3)  -- Exposed in GPIO header
#define pinGPIO11 11    // Exposed in GPIO header
#define pinGPIO13 13    // Exposed in GPIO header
#define pinGPIO35 35
#define pinGPIO36 36
#define pinGPIO37 37

#elif PCB_VERSION == 3
// PCB Versions 0.3, 0.4
#define pinLEDStart 42  // Start Button LED
#define pinLEDLR 0      // Left/Right Buttons LED
#define pinLEDABC 0     // ABC Buttons LED
#define pinLEDBG  0     // BG Buttons LED
#define pinLEDXYZ 0     // XYZ Buttons LED
#define pinLEDg 6       // PCB LED GREEN
#define pinLEDr 7       // PCB LED RED
#define pinACC_SCL 3    // Accelerometer SCL pin
#define pinRT 4         // Right Flipper
#define pinPlunger 5    // IR distance for plunger
#define rumbleSmall 8   // Large Rumble Motor
#define pinACC_SDA 9    // Accelerometer SDA pin
#define rumbleLarge 10  // Large Rumble Motor
#define pinDpadD 12     // Down on DPAD
#define pinDpadU 14     // Up on DPAD
#define pinB1 15        // Button 1 (A)
#define pinB2 16        // Button 2 (B)
#define pinB3 17        // Button 3 (X)
#define pinB4 18        // Button 4 (Y)
#define pinDpadL 38     // Left on DPAD
#define pinST 39        // Button 8 (Start)
#define pinBK 40        // Button 7 (Back)
#define pinXB 41        // XBOX Guide Button
#define pinLT 1         // Left Flipper
#define pinDpadR 48     // Right on DPAD
#define pinB9 47        // Button 9 (L3)   -- Exposed in GPIO header
#define pinB10 21       // Button 10 (R3)  -- Exposed in GPIO header
#define pinGPIO11 11    // Exposed in GPIO header
#define pinGPIO13 13    // Exposed in GPIO header
#define pinGPIO35 35
#define pinGPIO36 36
#define pinGPIO37 37

#elif PCB_VERSION == 5
// PCB Version 0.5
#define pinLEDStart 45  // Start Button LED
#define pinLEDLR 2      // Left/Right Buttons LED
#define pinLEDABC 7     // ABC Buttons LED
#define pinLEDBG 42     // BG Buttons LED
#define pinLEDXYZ 46    // XYZ Buttons LED
#define pinLEDg 6       // PCB LED GREEN
#define pinLEDr 0       // PCB LED RED
#define pinACC_SCL 3    // Accelerometer SCL pin
#define pinRT 4         // Right Flipper
#define pinPlunger 5    // IR distance for plunger
#define rumbleSmall 8   // Large Rumble Motor
#define pinACC_SDA 9    // Accelerometer SDA pin
#define rumbleLarge 10  // Large Rumble Motor
#define pinDpadD 12     // Down on DPAD
#define pinDpadU 14     // Up on DPAD
#define pinB1 15        // Button 1 (A)
#define pinB2 16        // Button 2 (B)
#define pinB3 17        // Button 3 (X)
#define pinB4 18        // Button 4 (Y)
#define pinDpadL 38     // Left on DPAD
#define pinST 39        // Button 8 (Start)
#define pinBK 40        // Button 7 (Back)
#define pinXB 41        // XBOX Guide Button
#define pinLT 1         // Left Flipper
#define pinDpadR 48     // Right on DPAD
#define pinB9 47        // Button 9 (L3)
#define pinB10 21       // Button 10 (R3)
#define pinGPIO11 11    // Exposed in GPIO header
#define pinGPIO13 13    // Exposed in GPIO header
#define pinGPIO35 35    // Exposed in GPIO header
#define pinGPIO36 36    // Exposed in GPIO header
#define pinGPIO37 37    // Exposed in GPIO header

#else
#error "Please set PCB_VERSION to a valid value"
#endif


Solenoid solLeft(pinGPIO11, pinGPIO13, pinGPIO37);
Solenoid solRight(pinGPIO35, pinGPIO36, pinGPIO37);


// Position of a button in the button status array
#define POSUP   0
#define POSDN   1
#define POSLT   2
#define POSRT   3
#define POSB1   4
#define POSB2   5
#define POSB3   6
#define POSB4   7
#define POSL1   8
#define POSR1   9
#define POSST  10
#define POSBK  11
#define POSXB  12
#define POSB9  13
#define POSB10 14
#define NUMBUTTONS 15   // Total number of buttons


uint8_t buttonStatus[NUMBUTTONS];  // array Holds a "Snapshot" of the button status to parse and manipulate

// Setup Button Debouncing
Button2 dpadUP = Button2(pinDpadU);
Button2 dpadDOWN = Button2(pinDpadD);
Button2 dpadLEFT = Button2(pinDpadL);
Button2 dpadRIGHT = Button2(pinDpadR);
Button2 button1 = Button2(pinB1);
Button2 button2 = Button2(pinB2);
Button2 button3 = Button2(pinB3);
Button2 button4 = Button2(pinB4);
Button2 buttonLT = Button2(pinLT);
Button2 buttonRT = Button2(pinRT);
Button2 buttonSTART = Button2(pinST);
Button2 buttonBACK = Button2(pinBK);
Button2 buttonXBOX = Button2(pinXB);
Button2 button9 = Button2(pinB9);
Button2 button10 = Button2(pinB10);


bool LED_states[50];

// Use LED_Set() and LED_SetAnalog() to control LEDS.
// These will ignore LEDs with pin numbers set to 0 (not used on the current board revision)
void LED_Set(int pin, bool state, bool skipSetState=0) {
  if (pin) {
    digitalWrite(pin, state);
  }
  if (!skipSetState) LED_states[pin] = state;
}
void LED_SetAnalog(int pin, int value) {
  if (pin) {
    analogWrite(pin, value);
  }
}


// Configure Inputs and Outputs
void setupPins()
{
  // Configure the direction of the pins
  // All inputs with internal pullups enabled
  pinMode(pinDpadU, INPUT_PULLUP);
  pinMode(pinDpadD, INPUT_PULLUP);
  pinMode(pinDpadL, INPUT_PULLUP);
  pinMode(pinDpadR, INPUT_PULLUP);
  pinMode(pinB1, INPUT_PULLUP);
  pinMode(pinB2, INPUT_PULLUP);
  pinMode(pinB3, INPUT_PULLUP);
  pinMode(pinB4, INPUT_PULLUP);
  pinMode(pinB9, INPUT_PULLUP);
  pinMode(pinB10, INPUT_PULLUP);
  pinMode(pinLT, INPUT_PULLUP);
  pinMode(pinRT, INPUT_PULLUP);
  pinMode(pinST, INPUT_PULLUP);
  pinMode(pinBK, INPUT_PULLUP);
  pinMode(pinXB, INPUT_PULLUP);
  pinMode(rumbleSmall, OUTPUT);
  pinMode(rumbleLarge, OUTPUT);

  if (pinLEDStart) pinMode(pinLEDStart, OUTPUT);
  if (pinLEDLR) pinMode(pinLEDLR, OUTPUT);
  if (pinLEDBG) pinMode(pinLEDBG, OUTPUT);
  if (pinLEDABC) pinMode(pinLEDABC, OUTPUT);
  if (pinLEDXYZ) pinMode(pinLEDXYZ, OUTPUT);
  if (pinLEDg) pinMode(pinLEDg, OUTPUT);
  if (pinLEDr) pinMode(pinLEDr, OUTPUT);

  // Disable bright neopixel on ESP32-S3 Dev Board
  // pinMode(LED_BUILTIN, OUTPUT);
  // LED_Set(LED_BUILTIN, LOW);

  // Set up LED initial states
  LED_SetAnalog(pinLEDr, PCB_LED_BRIGHTNESS);
  LED_SetAnalog(pinLEDg, 0);
  LED_Set(pinLEDStart, LOW);
  LED_Set(pinLEDLR, LOW);
  LED_Set(pinLEDBG, LOW);
  LED_Set(pinLEDABC, LOW);
  LED_Set(pinLEDXYZ, LOW);

  // Set button timeouts
  dpadUP.setDebounceTime(MILLIDEBOUNCE);
  dpadDOWN.setDebounceTime(MILLIDEBOUNCE);
  dpadLEFT.setDebounceTime(MILLIDEBOUNCE);
  dpadRIGHT.setDebounceTime(MILLIDEBOUNCE);
  button1.setDebounceTime(MILLIDEBOUNCE);
  button2.setDebounceTime(MILLIDEBOUNCE);
  button3.setDebounceTime(MILLIDEBOUNCE);
  button4.setDebounceTime(MILLIDEBOUNCE);
  buttonLT.setDebounceTime(MILLIDEBOUNCE);
  buttonRT.setDebounceTime(MILLIDEBOUNCE);
  buttonSTART.setDebounceTime(MILLIDEBOUNCE);
  buttonBACK.setDebounceTime(MILLIDEBOUNCE);
  buttonXBOX.setDebounceTime(MILLIDEBOUNCE);
  button9.setDebounceTime(MILLIDEBOUNCE);
  button10.setDebounceTime(MILLIDEBOUNCE);

  // Set up DATA and CLK pins for the Acceleromter I2C interface
  if (accelerometerEnabled) {
    Wire.setPins(pinACC_SDA, pinACC_SCL);
  }
}


// Blink when the LEDs are already on
void configFeedbackBlinks(int n)
{
  for (int i = 0; i < n-1; i++) {
    LED_Set(pinLEDStart, LOW);
    if (pinLEDBG) LED_Set(pinLEDBG, LOW);
    vTaskDelay_ms(300);
    LED_Set(pinLEDStart, HIGH);
    if (pinLEDBG) LED_Set(pinLEDBG, HIGH);
    vTaskDelay_ms(300);
  }
  LED_Set(pinLEDStart, LOW);
  if (pinLEDBG) LED_Set(pinLEDBG, LOW);
  vTaskDelay_ms(300);
  LED_Set(pinLEDStart, HIGH);
  if (pinLEDBG) LED_Set(pinLEDBG, HIGH);
}


// Blink when the LEDs are already off
void runtimeFeedbackBlinks(int n)
{
  for (int i = 0; i < n; i++) {
    LED_Set(pinLEDStart, HIGH);
    if (pinLEDBG) LED_Set(pinLEDBG, HIGH);
    vTaskDelay_ms(300);
    LED_Set(pinLEDStart, LOW);
    if (pinLEDBG) LED_Set(pinLEDBG, LOW);
    if (i == n-1) break; // Skip the last delay
    vTaskDelay_ms(300);
  }
}


uint16_t readingToDistance(int16_t reading) {
  // The signal from the IR distance detector is curved. Let's linearize. Thanks for the help Twitter!
  float voltage = reading / 310.0f;
  if (voltage == 0) return 0;  // Avoid divide by zero
  float linearDistance = ((0.1621f * voltage) + 1.0f) / (0.1567f * voltage);
  return linearDistance * 100;
}


void getPlungerSamples()
{
  int total = 0;

  for (int i = 0; i < numSamples; i++) {
    total += analogRead(pinPlunger);
  }
  plungerAverage = (total+numSamples/2)/numSamples;
}


void getPlungerMax()
{
  printf("Plunger calibration: starting...\n");

  configFeedbackBlinks(1);
  getPlungerSamples();
  plungerMin = plungerAverage + plungerMinOffset;
  plungerMax = plungerMin + 1;

  printf("Plunger calibration: recorded min.\n");
  printf("Plunger calibration: pull plunger back to record max...\n");

  while (plungerAverage < plungerMin + 100) {
    // wait for the plunger to be pulled
    getPlungerSamples();
    vTaskDelay_ms(16);
  }

  while (plungerAverage > plungerMin) {
    // start recording plungerMax
    getPlungerSamples();
    if (plungerAverage > plungerMax) {
      plungerMax = plungerAverage;
    }
    vTaskDelay_ms(16);
  }

  printf("Plunger calibration: recorded max.\n");

  preferences.putInt("plungerMin", plungerMin);
  preferences.putInt("plungerMax", plungerMax);

  plungerMaxDistance = readingToDistance(plungerMin);
  plungerMinDistance = readingToDistance(plungerMax);
  lastDistance = plungerMaxDistance;

  // Reset plungerZero to make sure we have full plunger movement so we can preperly set a new zero value
  plungerZeroValue = 0;
  preferences.putInt("plungerZero", plungerMin);

  printf("Plunger calibration data stored to flash.\n");
  printf("---- min: %d, max: %d\n", plungerMin, plungerMax);
  printf("Waiting for Dead Zone calibration.  Adjust plunger and press Start to set it...\n");
  waitingForDeadzoneSetting = true;

  configFeedbackBlinks(2);
}


void deadZoneCompensation()
{
  plungerZeroValue = map(distanceBuffer, plungerMaxDistance, plungerMinDistance, 0, XBOX_STICK_MAX) - 10;
  if (plungerZeroValue < 0) plungerZeroValue = 0;
  preferences.putInt("plungerZero", plungerZeroValue);
  printf("Plunger deadzone data stored to flash.\n");
  printf("---- plungerZeroValue: %d\n", plungerZeroValue);
  configFeedbackBlinks(3);
}


void checkForConfigButtonPresses()
{
  // Hold Back and dPad Down on boot to clear BLE paired devices
  if (buttonStatus[POSBK] && buttonStatus[POSDN]) {
    gamepad.clearWhitelist();
    configFeedbackBlinks(1);
    // Wait for button release
    while (buttonStatus[POSDN]) {
      delay(16);
      buttonUpdate();
    }
  }

  // Hold Back and dPad Up on boot to allow pairing a new device
  if (buttonStatus[POSBK] && buttonStatus[POSUP]) {
    printf("Allow new devices to connect...\n");
    gamepad.allowNewConnections(true);
    configFeedbackBlinks(2);
    // Wait for button release
    while (buttonStatus[POSUP]) {
      delay(16);
      buttonUpdate();
    }
  }

  // rumble test (hold Back and Left Flipper on boot)
  if (buttonStatus[POSBK] && buttonStatus[POSL1]) {
    printf("Left Flipper down at start - Rumble Test\n");
    configFeedbackBlinks(1);
    for (int i = 0; i < 256; i++) {
      analogWrite(rumbleSmall, i);
      vTaskDelay_ms(10);
    }
    for (int i = 255; i >= 0; i--) {
      analogWrite(rumbleSmall, i);
      vTaskDelay_ms(10);
    }

    for (int i = 0; i < 256; i++) {
      analogWrite(rumbleLarge, i);
      vTaskDelay_ms(10);
    }
    for (int i = 255; i >= 0; i--) {
      analogWrite(rumbleLarge, i);
      vTaskDelay_ms(10);
    }
    // Wait for button release
    while (buttonStatus[POSL1]) {
      delay(16);
      buttonUpdate();
    }
  }

  if (accelerometerEnabled) {
    if (buttonStatus[POSBK] && buttonStatus[POSRT]) {
      waitingForAccelSetting = true;
      printf("Waiting for Accelerometer calibration.  Close lid and press Start to set it...\n");
      configFeedbackBlinks(1);
      // Wait for button release
      while (buttonStatus[POSRT]) {
        delay(16);
        buttonUpdate();
      }
    }
  }

  // If BACK and Left D-Pad are pressed simultaneously, toggle controlShuffle, and persist that value
  if (buttonStatus[POSBK] && buttonStatus[POSLT]) {
    controlShuffle = !controlShuffle;
    preferences.putBool("controlShuffle", controlShuffle);
    if (controlShuffle) {
      printf("Control Shuffle Enabled: Plunge with Left Stick, Tilt with D-Pad.\n");
      configFeedbackBlinks(2);
    }
    else {
      printf("Control Shuffle Disabled: Plunge with Right Stick, Tilt with Left Stick.\n");
      configFeedbackBlinks(1);
    }
    // Wait for button release
    while (buttonStatus[POSLT]) {
      delay(16);
      buttonUpdate();
    }
  }
  
  if (plungerEnabled) {
    // to calibrate, hold Back and Start
    if (buttonStatus[POSBK] && buttonStatus[POSST]) {
      printf("Begin Plunger Calibration\n");
      getPlungerMax();
      // Wait for button release
      while (buttonStatus[POSST]) {
        delay(16);
        buttonUpdate();
      }
    }
  }

  // Solenoid toggle (hold Back and Right Flipper)
  if (buttonStatus[POSBK] && buttonStatus[POSR1]) {
    solenoidEnabled = !solenoidEnabled;
    preferences.putBool("solenoidEnabled", solenoidEnabled);
    if (solenoidEnabled) {
      printf("Solenoids Enabled\n");
      configFeedbackBlinks(2);
    }
    else {
      printf("Solenoids Disabled\n");
      configFeedbackBlinks(1);
    }
    // Wait for button release
    while (buttonStatus[POSR1]) {
      delay(16);
      buttonUpdate();
    }
  }
}

void setup()
{
  setupPins();
  preferences.begin("PinSimESP32");

  // delay(2000);
  printf("\n\n");
  printf("PinSim ESP32 Starting up\n");

  uint8_t mac_bytes[6];
  esp_efuse_mac_get_default(mac_bytes);
  printf("MAC: %02X%02X %02X%02X %02X%02X\n", mac_bytes[0], mac_bytes[1], mac_bytes[2], mac_bytes[3], mac_bytes[4], mac_bytes[5]);

  // Set up vibration event handler
  FunctionSlot<XboxGamepadOutputReportData> vibrationSlot(OnVibrateEvent);
  gamepad.onVibrate.attach(vibrationSlot);

  gamepad.startServer(XB_NAME, XB_MANUFACTURER);

  for (int i = 0; i < 6; i++) {
    delay(16);
    buttonUpdate();
  }

  /* Initialise the sensor */
  if (!accel.begin()) {
    printf("Accelerometer setup failed\n");
    /* There was a problem detecting the ADXL345 ... check your connections */
    accelerometerEnabled = false;

    accel.setRange(ADXL345_RANGE_2_G);
    delay(100);
    
    zeroX = preferences.getInt("accelZeroX", zeroX);
    zeroY = preferences.getInt("accelZeroY", zeroY);
  }

  if (plungerEnabled) {
    plungerMin = preferences.getInt("plungerMin", plungerMin);  // With default value
    plungerMax = preferences.getInt("plungerMax", plungerMax);  // With default value
    plungerZeroValue  = preferences.getInt("plungerZero", plungerZeroValue);  // With default value
  }

  controlShuffle = preferences.getBool("controlShuffle", controlShuffle);
  solenoidEnabled = preferences.getBool("solenoidEnabled", solenoidEnabled);

  // Hold Back button on boot to enter config mode
  if (buttonStatus[POSBK]) {
    static uint8_t ledON = 0;
    if (pinLEDBG) LED_Set(pinLEDBG, HIGH);
    while (buttonStatus[POSBK]) {
      LED_Set(pinLEDStart, (ledON & 0x02)!=0);
      ledON++;
      checkForConfigButtonPresses();
      delay(16);
      buttonUpdate();
    }
    if (pinLEDBG) LED_Set(pinLEDBG, LOW);
  }

  gamepad.startAdvertising();

  // plunger setup
  if (plungerEnabled) {
    // linear conversions
    plungerMaxDistance = readingToDistance(plungerMin);
    plungerMinDistance = readingToDistance(plungerMax);
    lastDistance = plungerMaxDistance;
  }

  if (solenoidEnabled) {
    solLeft.setup();
    solRight.setup();
  }

  xTaskCreatePinnedToCore(&handle_main_task, "mani_loop", 8192, NULL, 20, &mainTaskHandle, 1);
}

// Nothing to do in loop().  Use handle_main_task() instead.
void loop() {}


// Update the debounced button statuses
void buttonUpdate()
{
  dpadUP.loop();
  dpadDOWN.loop();
  dpadLEFT.loop();
  dpadRIGHT.loop();

  // Disable button LEDs while testing the buttons
  LED_Set(pinLEDABC, LOW, 1);
  button1.loop();
  button2.loop();
  button9.loop();
  LED_Set(pinLEDABC, LED_states[pinLEDABC]);

  // Disable button LEDs while testing the buttons
  LED_Set(pinLEDXYZ, LOW, 1);
  button3.loop();
  button4.loop();
  button10.loop();
  LED_Set(pinLEDXYZ, LED_states[pinLEDXYZ]);

  // Disable button LEDs while testing the buttons
  LED_Set(pinLEDLR, LOW, 1);
  buttonLT.loop();
  buttonRT.loop();
  LED_Set(pinLEDLR, LED_states[pinLEDLR]);

  // Disable button LEDs while testing the buttons
  LED_Set(pinLEDStart, LOW, 1);
  buttonSTART.loop();
  LED_Set(pinLEDStart, LED_states[pinLEDStart]);

  // Disable button LEDs while testing the buttons
  LED_Set(pinLEDBG, LOW, 1);
  buttonBACK.loop();
  buttonXBOX.loop();
  LED_Set(pinLEDBG, LED_states[pinLEDBG]);

  buttonStatus[POSUP] = dpadUP.isPressed();
  buttonStatus[POSDN] = dpadDOWN.isPressed();
  buttonStatus[POSLT] = dpadLEFT.isPressed();
  buttonStatus[POSRT] = dpadRIGHT.isPressed();
  buttonStatus[POSB1] = button1.isPressed();
  buttonStatus[POSB2] = button2.isPressed();
  buttonStatus[POSB3] = button3.isPressed();
  buttonStatus[POSB4] = button4.isPressed();
  buttonStatus[POSL1] = buttonLT.isPressed();
  buttonStatus[POSR1] = buttonRT.isPressed();
  buttonStatus[POSST] = buttonSTART.isPressed();
  buttonStatus[POSBK] = buttonBACK.isPressed();
  buttonStatus[POSXB] = buttonXBOX.isPressed();
  buttonStatus[POSB9] = button9.isPressed();
  buttonStatus[POSB10] = button10.isPressed();
}


// Process Home button press delay
// Wait until DELAY_HOME ms after button inisially pressed to start sending the button press
void pressHome(bool isPressed)
{
  static int wasPressed = 0;  // 0 = was up, 1 = was waiting, 2 = was pressed
  static long lastPress = 0;
  long currentTime = millis();

  if (wasPressed == 0 && isPressed) {
    lastPress = currentTime;
    wasPressed = 1;
  }
  else if (wasPressed == 1 && isPressed) {
    long currentTime = millis();
    if (currentTime > lastPress + DELAY_HOME) {
      gamepad.press(XBOX_BUTTON_HOME);
      wasPressed = 2;
    }
  }
  else if (wasPressed != 0 && !isPressed) {
    gamepad.release(XBOX_BUTTON_HOME);
    wasPressed = 0;
  }
}


// Process Start button press delay
// Send the Start button immediately, but after DELAY_L3R3, also start sending L3 and R3
void pressStart(bool isPressed) {
  static int wasPressed = 0;  // 0 = was up, 1 = was waiting, 2 = was pressed
  static long lastPress = 0;
  long currentTime = millis();

  if (wasPressed == 0 && isPressed) {
    gamepad.press(XBOX_BUTTON_START);
    lastPress = currentTime;
    wasPressed = 1;
  }
  else if (wasPressed == 1 && isPressed) {
    long currentTime = millis();
    if (currentTime > lastPress + DELAY_L3R3) {
      gamepad.press(XBOX_BUTTON_LS);
      gamepad.press(XBOX_BUTTON_RS);
      wasPressed = 2;
    }
  }
  else if (wasPressed != 0 && !isPressed) {
    gamepad.release(XBOX_BUTTON_START);
    gamepad.release(XBOX_BUTTON_LS);
    gamepad.release(XBOX_BUTTON_RS);
    wasPressed = 0;
  }
}


// ProcessInputs
void processInputs()
{
  uint8_t direction = XboxDpadFlags::NONE;
  
  // Use -1 instead of 0 for non-moving thumb/stick axes to fix right stick stuck to top-left
  // Some hosts need to see negative values or else they assume that 0 is the lowest value.
  int8_t center[2];
  center[0] = -1;
  center[1] = -1;

  // Update the DPAD
  if (buttonStatus[POSUP]) direction |= XboxDpadFlags::NORTH;
  if (buttonStatus[POSRT]) direction |= XboxDpadFlags::EAST;
  if (buttonStatus[POSDN]) direction |= XboxDpadFlags::SOUTH;
  if (buttonStatus[POSLT]) direction |= XboxDpadFlags::WEST;

  // Buttons
  if (buttonStatus[POSB1]) gamepad.press(XBOX_BUTTON_A);
  else gamepad.release(XBOX_BUTTON_A);
  if (buttonStatus[POSB2]) gamepad.press(XBOX_BUTTON_B);
  else gamepad.release(XBOX_BUTTON_B);
  if (buttonStatus[POSB3]) gamepad.press(XBOX_BUTTON_X);
  else gamepad.release(XBOX_BUTTON_X);
  if (buttonStatus[POSB4]) gamepad.press(XBOX_BUTTON_Y);
  else gamepad.release(XBOX_BUTTON_Y);

  if (!buttonStatus[POSST]) {
    if (buttonStatus[POSB9]) gamepad.press(XBOX_BUTTON_LS);
    else gamepad.release(XBOX_BUTTON_LS);
    if (buttonStatus[POSB10]) gamepad.press(XBOX_BUTTON_RS);
    else gamepad.release(XBOX_BUTTON_RS);
  }

  // If we're still waiting for DeadZone calibration, and Start is pressed, set new plunger dead zone
  // Compensates for games where the in-game plunger doesn't begin pulling back until
  // the gamepad is pulled back ~half way. Just pull the plunger to the point just before
  // it begins to move in-game, and then press START.
  if (waitingForDeadzoneSetting && buttonStatus[POSST]) {
    deadZoneCompensation();
    printf("Calibrated Plunger Dead Zone\n");
    waitingForDeadzoneSetting = false;
  }

  // Bumpers
  if (buttonStatus[POSL1]) gamepad.press(XBOX_BUTTON_LB);
  else gamepad.release(XBOX_BUTTON_LB);
  if (buttonStatus[POSR1]) gamepad.press(XBOX_BUTTON_RB);
  else gamepad.release(XBOX_BUTTON_RB);

  // Middle Buttons: Start, Select, Home
  if (buttonStatus[POSST] && buttonStatus[POSBK]) {
    pressStart(0);
    gamepad.release(XBOX_BUTTON_SELECT);
    pressHome(1);
  } else if (buttonStatus[POSST]) {
    pressHome(0);
    gamepad.release(XBOX_BUTTON_SELECT);
    pressStart(1);
  } else if (buttonStatus[POSBK]) {
    pressHome(0);
    pressStart(0);
    gamepad.press(XBOX_BUTTON_SELECT);
  } else if (buttonStatus[POSXB]) {
    pressStart(0);
    gamepad.release(XBOX_BUTTON_SELECT);
    pressHome(1);
  } else {
    pressHome(0);
    pressStart(0);
    gamepad.release(XBOX_BUTTON_SELECT);
  }

  // Tilt
  if (accelerometerEnabled) {
    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);

    accX = event.acceleration.x * nudgeMultiplier * -1;
    accY = event.acceleration.y * nudgeMultiplier * -1;

    // Re-calibrate accelerometer if we're waiting for Accel setting, and Start is pressed
    if (waitingForAccelSetting && buttonStatus[POSST]) {
      zeroX = accX;
      zeroY = accY;
      preferences.putInt("accelZeroX", zeroX);
      preferences.putInt("accelZeroY", zeroY);
      printf("Recalibrated accelerometers.\n");
      printf("---- accelZeroX: %d, accelZeroY: %d\n", zeroX, zeroY);
      configFeedbackBlinks(1);
      waitingForAccelSetting = false;
    }

    int32_t accXcon = constrain(accX-zeroX, XBOX_STICK_MIN, XBOX_STICK_MAX);
    int32_t accYcon = constrain(accY-zeroY, XBOX_STICK_MIN, XBOX_STICK_MAX);

    if (millis() > tiltEnableTime) {
      if (controlShuffle) {
        if (accYcon > XBOX_STICK_MAX * 0.6) direction |= XboxDpadFlags::NORTH;
        if (accXcon > XBOX_STICK_MAX * 0.6) direction |= XboxDpadFlags::EAST;
        if (accYcon < XBOX_STICK_MIN * 0.6) direction |= XboxDpadFlags::SOUTH;
        if (accXcon < XBOX_STICK_MIN * 0.6) direction |= XboxDpadFlags::WEST;
      }
      else {
        // Add a big dead zone for Left-stick analog accelerometer movements
        if (abs(accXcon) > XBOX_STICK_MAX * 0.6 || abs(accYcon) > XBOX_STICK_MAX * 0.6) {
          gamepad.setLeftThumb(accXcon, accYcon);
        }
        else {
          gamepad.setLeftThumb(0, 0);
        }
      }
    }
    gamepad.pressDPadDirection(XboxDpadFlags(direction));
  }

  // Plunger
  // This is based on the Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm
  if (plungerEnabled) {
    getPlungerSamples();

    // // Automatically move the plunger, for testing without a plunger
    // static int16_t pCnt = 0;
    // pCnt = (++pCnt)%1024;
    // plungerAverage = (((pCnt >= 512) ? (1024-pCnt) : pCnt)-256)*128;
    // gamepad.setRightThumb(0, plungerAverage);

    int16_t currentDistance = readingToDistance(plungerAverage);
    distanceBuffer = currentDistance;

    if (currentDistance < plungerMaxDistance - 20 && currentDistance > plungerMinDistance + 20) {
      // Attempt to detect plunge
      int16_t adjustedPlungeTrigger = map(currentDistance, plungerMaxDistance, plungerMinDistance, plungeTrigger / 2, plungeTrigger);
      if (currentDistance - lastDistance >= adjustedPlungeTrigger) {
        // we throw STICK_RIGHT to 0 to better simulate the physical behavior of a real analog stick
        if (controlShuffle) {
          gamepad.setLeftThumb(center[0], center[1]);
          gamepad.setRightThumb(center[0], center[1]);
        }
        else {
          gamepad.setRightThumb(center[0], center[1]);
        }
        distanceBuffer = plungerMaxDistance;
        lastDistance = plungerMaxDistance;
        return;
      }
      lastDistance = currentDistance;

      // Disable accelerometer while plunging and for 1 second afterwards.
      if (currentDistance < plungerMaxDistance - 20)
        tiltEnableTime = millis() + 1000;
    } else if (currentDistance <= plungerMinDistance + 20) {
      // cap max
      distanceBuffer = plungerMinDistance;
      tiltEnableTime = millis() + 1000;
    } else if (currentDistance > plungerMaxDistance) {
      // cap min
      distanceBuffer = plungerMaxDistance;
    }

    if (controlShuffle) {
      gamepad.setLeftThumb(center[0], map(distanceBuffer, plungerMaxDistance, plungerMinDistance, plungerZeroValue, XBOX_STICK_MAX));
    }
    else {
      gamepad.setRightThumb(center[0], map(distanceBuffer, plungerMaxDistance, plungerMinDistance, plungerZeroValue, XBOX_STICK_MAX));
    }
  }
  if (controlShuffle) {
    gamepad.setRightThumb(center[0], center[1]);
  }
}


void ledUpdate()
{
  if (!gamepad.isAdvertisingNewDevices()) {
    // Connected!  So LEDs On Solid
    LED_SetAnalog(pinLEDg, PCB_LED_BRIGHTNESS);
    LED_Set(pinLEDStart, HIGH);
  } else {
    // BLE Pairing mode -- So Flash LEDs
    static uint8_t blinkCounter = 0;
    static bool ledOn = 0;
    if (blinkCounter++ >= 30) {
      ledOn = !ledOn;
      blinkCounter = 0;
    }
    LED_SetAnalog(pinLEDg, (ledOn) ? PCB_LED_BRIGHTNESS : 0);
    LED_Set(pinLEDStart, ledOn);
  }
}


void solenoidUpdate()
{
  // Last state is the previous button state, to help notice state changes
  static bool lastStateLeft  = 0;
  static bool lastStateRight = 0;

  if (buttonStatus[POSL1] && !lastStateLeft) {
    // Left was just pressed
    lastStateLeft = 1;
    solLeft.fwd();
    tiltEnableTime = millis() + 100;
  }
  else if (!buttonStatus[POSL1] && lastStateLeft) {
    // Left was just released
    lastStateLeft = 0;
    solLeft.coast();
    tiltEnableTime = millis() + 100;
  }

  if (buttonStatus[POSR1] && !lastStateRight) {
    // Right was just pressed
    lastStateRight = 1;
    solRight.fwd();
    tiltEnableTime = millis() + 100;
  }
  else if (!buttonStatus[POSR1] && lastStateRight) {
    // Right was just released
    lastStateRight = 0;
    solRight.coast();
    tiltEnableTime = millis() + 100;
  }

  if (lastStateLeft == 0 && lastStateRight == 0) {
    solLeft.standby();
  }
}


// Delay until at least ms_since_last_delay ms after the last call of this function
void delay_since_last_delay(uint32_t ms_since_last_delay)
{
    static uint32_t last_target = 0;
    uint32_t now = millis();
    uint32_t new_target = last_target + ms_since_last_delay;
    if (new_target <= now) {
        new_target = now;
    }
    else {
        vTaskDelay_ms(new_target - now);
    }
    last_target = new_target;
}


// Main Task runs forever, yielding during vTaskDelay() calls
void handle_main_task(void *arg)
{
  while (true) {
    delay_since_last_delay(16);

    // Update Solenoids
    // Doing this before buttonUpdate() effectively adds a 16ms delay
    if (solenoidEnabled) {
      solenoidUpdate();
    }

    // Poll Buttons
    buttonUpdate();

    // Update LEDs
    ledUpdate();

    if (gamepad.isConnected()) {
      // Process all inputs and load up the usbData registers correctly
      processInputs();

      // Add status data into trigger values as small movements
      updateTriggerStatus();

      // Send controller data
      gamepad.sendGamepadReport();
    }
  }
}


// This value as the weak magnitude means the strong magnitude is a command
#define RUMBLE_COMMAND_MAGIC 2

// Commands in the strong magnitude field
#define RUMBLE_COMMAND_ACCEL_CAL 3
#define RUMBLE_COMMAND_PLUNGER_SET_MIN 4
#define RUMBLE_COMMAND_PLUNGER_SET_MAX 5
#define RUMBLE_COMMAND_PLUNGER_SET_ZERO 6
#define RUMBLE_COMMAND_TOGGLE_CONTROL_SWAP 7
#define RUMBLE_COMMAND_TOGGLE_SOLENOIDS 8
#define RUMBLE_COMMAND_PAIR_CLEAR 9
#define RUMBLE_COMMAND_PAIR_START 10

#define RUMBLE_COMMAND_STATUS_PLUNGER_CONTROL_RIGHT 2
#define RUMBLE_COMMAND_STATUS_SOLENOIDS_ENABLED 4


// Handle Vibrate/Rumble events
void OnVibrateEvent(XboxGamepadOutputReportData data)
{
  if (data.weakMotorMagnitude == RUMBLE_COMMAND_MAGIC) {
    if (handleRumbleCommand(data.strongMotorMagnitude)) {
      return;
    }
  }
  printf("Rumble: %d, %d\n", data.weakMotorMagnitude, data.strongMotorMagnitude);
  analogWrite(rumbleSmall, data.weakMotorMagnitude);
  analogWrite(rumbleLarge, data.strongMotorMagnitude);
}


void updateTriggerStatus()
{
  int leftStatus = 0;
  if (controlShuffle) leftStatus += RUMBLE_COMMAND_STATUS_PLUNGER_CONTROL_RIGHT;
  if (solenoidEnabled) leftStatus += RUMBLE_COMMAND_STATUS_SOLENOIDS_ENABLED;
  gamepad.setLeftTrigger(leftStatus);

  int rightStatus = gamepad.getPairCount() * 2;
  gamepad.setRightTrigger(rightStatus);
}

// Received a runmble event that encodes a pinsim command
boolean handleRumbleCommand(uint8_t command)
{
  switch (command) {
    case RUMBLE_COMMAND_ACCEL_CAL:
      printf("Rumble Command: Calibrate Accels\n");
      zeroX = accX;
      zeroY = accY;
      preferences.putInt("accelZeroX", zeroX);
      preferences.putInt("accelZeroY", zeroY);
      runtimeFeedbackBlinks(1);
      break;
    
    case RUMBLE_COMMAND_PLUNGER_SET_MIN:
      printf("Rumble Command: Plunger Set Min\n");
      plungerMin = plungerAverage;
      plungerMaxDistance = readingToDistance(plungerMin);
      preferences.putInt("plungerMin", plungerMin);
      runtimeFeedbackBlinks(1);
      break;
    
    case RUMBLE_COMMAND_PLUNGER_SET_MAX:
      printf("Rumble Command: Plunger Set Max\n");
      plungerMax = plungerAverage;
      plungerMinDistance = readingToDistance(plungerMax);
      preferences.putInt("plungerMax", plungerMax);
      runtimeFeedbackBlinks(1);
      break;
    
    case RUMBLE_COMMAND_PLUNGER_SET_ZERO:
      printf("Rumble Command: Plunger Set Zero\n");
      plungerZeroValue = map(distanceBuffer, plungerMaxDistance, plungerMinDistance, 0, XBOX_STICK_MAX) - 10;
      if (plungerZeroValue < 0) plungerZeroValue = 0;
      preferences.putInt("plungerZero", plungerZeroValue);
      runtimeFeedbackBlinks(1);
      break;
    
    case RUMBLE_COMMAND_TOGGLE_CONTROL_SWAP:
      printf("Rumble Command: Control Swap\n");
      controlShuffle = !controlShuffle;
      preferences.putBool("controlShuffle", controlShuffle);
      runtimeFeedbackBlinks(controlShuffle ? 2 : 1);
      break;
    
    case RUMBLE_COMMAND_TOGGLE_SOLENOIDS:
      printf("Rumble Command: Toggle Solenoids\n");
      solenoidEnabled = !solenoidEnabled;
      preferences.putBool("solenoidEnabled", solenoidEnabled);
      runtimeFeedbackBlinks(solenoidEnabled ? 2 : 1);
      break;
    
    case RUMBLE_COMMAND_PAIR_CLEAR:
      printf("Rumble Command: Clear Pairing\n");
      gamepad.clearWhitelist();
      runtimeFeedbackBlinks(1);
      break;
    
    case RUMBLE_COMMAND_PAIR_START:
      printf("Rumble Command: Pairing Mode\n");
      gamepad.allowNewConnections(true);
      gamepad.startAdvertising();
      runtimeFeedbackBlinks(2);
      break;

    default:
      printf("Bad Rumble Command: %d\n", command);
      return false;
  }
  return true;
}
