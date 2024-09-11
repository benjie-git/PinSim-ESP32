/*
    PinSim-ESP32 Controller
    Octopilot Electronics

    Based on the excellent PinSim Controller v20200517
    Controller for PC Pinball games
    https://www.youtube.com/watch?v=18EcIxywXHg
    
    Based on the excellent MSF_FightStick XINPUT project by Zack "Reaper" Littell
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
    
    And also these libraries from downloaded zip files (Sketch menu -> Include Library -> Add .ZIP library)
    - Average - https://github.com/MajenkoLibraries/Average
    - BleCompositeHID - https://github.com/Mystfit/ESP32-BLE-CompositeHID

*/

#include <Arduino.h>
#include <Preferences.h>
#include <Button2.h>
#include <Average.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <BleConnectionStatus.h>
#include <BleCompositeHID.h>
#include <XboxGamepadDevice.h>


// GLOBAL CONFIGURATION VARIABLES
// configure these
boolean flipperL1R1 = true;             //
boolean fourFlipperButtons = false;     // FLIP_L & FLIP_R map to L1/R1 and pins 13 & 14 map to analog L2/R2 100%
boolean doubleContactFlippers = false;  // FLIP_L & FLIP_R map to analog L2/R2 10% and pins 13 & 14 map to L2/R2 100%
boolean analogFlippers = false;         // use analog flipper buttons
boolean leftStickJoy = false;           // joystick moves left analog stick instead of D-pad
boolean accelerometerEnabled = true;
boolean accelerometerCalibrated = false;
boolean plungerEnabled = true;
int16_t nudgeMultiplier = 9000;         // accelerometer multiplier (higher = more sensitive)
int16_t plungeTrigger = 60;             // threshold to trigger a plunge (lower = more sensitive)
int16_t fourButtonModeThreshold = 250;  // ms that pins 13/14 need to close WITHOUT FLIP_L/FLIP_R closing to trigger four flipper button mode.


// Store settings to EEPROM using preferences storage name: PinSimESP32
// Fields: int plungerMin, int plungerMax
Preferences preferences;

int numSamples = 20;
Average<int> ave(numSamples);

// Assign a unique ID to this sensor at the same time
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

BleCompositeHID compositeHID("PinSim ESP32 Controller", "Octopilot Electronics", 100);
XboxGamepadDevice* gamepad;

long fourButtonModeTriggeredLB = 0;  // these two vars are used to check for 4 flipper buttons
long fourButtonModeTriggeredRB = 0;
int16_t zeroValue = 0;            // xbox 360 value for neutral analog stick
int16_t zeroValueBuffer = 0;      // save zero value during plunge
int16_t plungerReportDelay = 17;  // delay in ms between reading ~60hz plunger updates from sensor
int16_t plungerMin = 200;         // min plunger analog sensor value
int16_t plungerMax = 550;         // max plunger analog sensor value
int16_t plungerMaxDistance = 0;   // sensor value converted to actual distance
int16_t plungerMinDistance = 0;
uint32_t plungerReportTime = 0;
boolean currentlyPlunging = false;
uint32_t tiltEnableTime = 0;
int16_t lastReading = 0;
int16_t lastDistance = 0;
int16_t distanceBuffer = 0;
float zeroX = 0;
float zeroY = 0;


// Pin Declarations
#define pinLED1 1       // Onboard LED 1
#define pinLED2 2       // Onboard LED 2
#define pinACC_SCL 3    // Accelerometer SCL pin
#define pinRT 4         // Right Analog Trigger
#define pinPlunger 5    // IR distance for plunger
#define pinLEDg 6       // PCB LED GREEN
#define pinLEDr 7       // PCB LED RED
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
#define pinLT 42        // Left Analog Trigger
#define pinDpadR 48     // Right on DPAD
#define pinB9 47        // Button 9 (L3)   -- Exposed in GPIO header
#define pinB10 21       // Button 10 (R3)  -- Exposed in GPIO header
#define pinLB 13        // Button 5 (LB)   -- Exposed in GPIO header
#define pinRB 11        // Button 6 (RB)   -- Exposed in GPIO header
#define NUMBUTTONS 17   // Total number of buttons


// Position of a button in the button status array
#define POSUP 0
#define POSDN 1
#define POSLT 2
#define POSRT 3
#define POSB1 4
#define POSB2 5
#define POSB3 6
#define POSB4 7
#define POSL1 8
#define POSR1 9
#define POSL2 10
#define POSR2 11
#define POSST 12
#define POSBK 13
#define POSXB 14
#define POSB9 15
#define POSB10 16


// Debounce time in milliseconds
#define MILLIDEBOUNCE 20


// Dual flippers
uint8_t leftTrigger = 0;
uint8_t rightTrigger = 0;

byte buttonStatus[NUMBUTTONS];  // array Holds a "Snapshot" of the button status to parse and manipulate

// LED Toggle Tracking Global Variables
uint8_t LEDState = LOW;   //used to set the pin for the LED
uint32_t previousMS = 0;  //used to store the last time LED was updated
uint8_t LEDtracker = 0;   //used as an index to step through a pattern on interval

// LED Patterns
uint8_t patternAllOff[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t patternBlinkRotate[10] = { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 };
uint8_t patternPlayer1[10] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t patternPlayer2[10] = { 1, 0, 1, 0, 0, 0, 0, 0, 0, 0 };
uint8_t patternPlayer3[10] = { 1, 0, 1, 0, 1, 0, 0, 0, 0, 0 };
uint8_t patternPlayer4[10] = { 1, 0, 1, 0, 1, 0, 1, 0, 0, 0 };

// Variable to hold the current pattern selected by the host
uint8_t patternCurrent[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Setup Button Debouncing
Button2 dpadUP = Button2(pinDpadU);
Button2 dpadDOWN = Button2(pinDpadD);
Button2 dpadLEFT = Button2(pinDpadL);
Button2 dpadRIGHT = Button2(pinDpadR);
Button2 button1 = Button2(pinB1);
Button2 button2 = Button2(pinB2);
Button2 button3 = Button2(pinB3);
Button2 button4 = Button2(pinB4);
Button2 buttonLB = Button2(pinLB);
Button2 buttonRB = Button2(pinRB);
Button2 buttonLT = Button2(pinLT);
Button2 buttonRT = Button2(pinRT);
Button2 buttonSTART = Button2(pinST);
Button2 buttonBACK = Button2(pinBK);
Button2 buttonXBOX = Button2(pinXB);
Button2 button9 = Button2(pinB9);
Button2 button10 = Button2(pinB10);


// Configure Inputs and Outputs
void setupPins()
{
  // Configure the direction of the pins
  // All inputs with internal pullups enabled
  pinMode(pinDpadU, INPUT_PULLUP);
  pinMode(pinDpadD, INPUT_PULLUP);
  pinMode(pinDpadL, INPUT_PULLUP);
  pinMode(pinDpadR, INPUT_PULLDOWN);
  pinMode(pinB1, INPUT_PULLUP);
  pinMode(pinB2, INPUT_PULLUP);
  pinMode(pinB3, INPUT_PULLUP);
  pinMode(pinB4, INPUT_PULLUP);
  pinMode(pinB9, INPUT_PULLUP);
  pinMode(pinB10, INPUT_PULLUP);
  pinMode(pinLB, INPUT_PULLUP);
  pinMode(pinRB, INPUT_PULLUP);
  pinMode(pinLT, INPUT_PULLUP);
  pinMode(pinRT, INPUT_PULLUP);
  pinMode(pinST, INPUT_PULLUP);
  pinMode(pinBK, INPUT_PULLUP);
  pinMode(pinXB, INPUT_PULLUP);
  pinMode(pinLED1, OUTPUT);
  pinMode(pinLED2, OUTPUT);
  pinMode(pinLEDg, OUTPUT);
  pinMode(pinLEDr, OUTPUT);
  pinMode(rumbleSmall, OUTPUT);
  pinMode(rumbleLarge, OUTPUT);

  // Set button timeouts
  dpadUP.setDebounceTime(MILLIDEBOUNCE);
  dpadDOWN.setDebounceTime(MILLIDEBOUNCE);
  dpadLEFT.setDebounceTime(MILLIDEBOUNCE);
  dpadRIGHT.setDebounceTime(MILLIDEBOUNCE);
  button1.setDebounceTime(MILLIDEBOUNCE);
  button2.setDebounceTime(MILLIDEBOUNCE);
  button3.setDebounceTime(MILLIDEBOUNCE);
  button4.setDebounceTime(MILLIDEBOUNCE);
  buttonLB.setDebounceTime(MILLIDEBOUNCE);
  buttonRB.setDebounceTime(MILLIDEBOUNCE);
  buttonLT.setDebounceTime(MILLIDEBOUNCE);
  buttonRT.setDebounceTime(MILLIDEBOUNCE);
  buttonSTART.setDebounceTime(MILLIDEBOUNCE);
  buttonBACK.setDebounceTime(MILLIDEBOUNCE);
  buttonXBOX.setDebounceTime(MILLIDEBOUNCE);
  button9.setDebounceTime(MILLIDEBOUNCE);
  button10.setDebounceTime(MILLIDEBOUNCE);

  // Set the LED to low to make sure it is off
  digitalWrite(pinLED1, LOW);
  digitalWrite(pinLEDg, LOW);

  // Disable bright neopixel on ESP32-S3 Dev Board
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, LOW);

  // Set the LED to high to turn it on
  digitalWrite(pinLED2, HIGH);
  digitalWrite(pinLEDr, HIGH);

  // Set up DATA and CLK pins for the Acceleromter I2C interface
  if (accelerometerEnabled) {
    Wire.setPins(pinACC_SDA, pinACC_SCL);
  }
}


void flashStartButton()
{
  for (int i = 0; i < 10; i++) {
    digitalWrite(pinLED1, HIGH);
    delay(50);
    digitalWrite(pinLED1, LOW);
    delay(50);
  }
}


uint16_t readingToDistance(int16_t reading)
{
  // The signal from the IR distance detector is curved. Let's linearize. Thanks for the help Twitter!
  float voltage = reading / 310.0f;
  float linearDistance = ((0.1621f * voltage) + 1.0f) / (0.1567f * voltage);
  return linearDistance * 100;
}


uint16_t getPlungerAverage()
{
  for (int i = 0; i < numSamples; i++) {
    int reading = analogRead(pinPlunger);

    if ((reading - lastReading) > -10 && (reading - lastReading) < 10) {
      ave.push(reading);
    }
    lastReading = reading;
  }
  int averageReading = ave.mean();
  return averageReading;
}


void getPlungerMax()
{
  flashStartButton();
  plungerMin = getPlungerAverage();
  plungerMax = plungerMin + 1;
  int averageReading = ave.mean();
  while (averageReading < plungerMin + 100) {
    // wait for the plunger to be pulled
    int reading = analogRead(pinPlunger);
    if ((reading - lastReading) > -10 && (reading - lastReading) < 10) {
      ave.push(reading);
    }
    lastReading = reading;
    averageReading = ave.mean();
  }

  while (averageReading > plungerMin) {
    // start recording plungerMax
    int reading = analogRead(pinPlunger);
    if ((reading - lastReading) > -10 && (reading - lastReading) < 10) {
      ave.push(reading);
    }
    lastReading = reading;
    averageReading = ave.mean();
    if (averageReading > plungerMax) {
      plungerMax = averageReading;
    }
  }

  preferences.putInt("plungerMin", plungerMin);
  preferences.putInt("plungerMax", plungerMax);
  flashStartButton();
}


// Handle Vibrate/Rumble events
void OnVibrateEvent(XboxGamepadOutputReportData data)
{
  analogWrite(rumbleSmall, data.weakMotorMagnitude);
  analogWrite(rumbleLarge, data.strongMotorMagnitude);
}


void setup()
{
  XboxOneSControllerDeviceConfiguration* config = new XboxOneSControllerDeviceConfiguration();
  BLEHostConfiguration hostConfig = config->getIdealHostConfiguration();
  gamepad = new XboxGamepadDevice(config);
  // Set up vibration event handler
  FunctionSlot<XboxGamepadOutputReportData> vibrationSlot(OnVibrateEvent);
  gamepad->onVibrate.attach(vibrationSlot);
  // Add all child devices to the top-level composite HID device to manage them
  compositeHID.addDevice(gamepad);
  compositeHID.begin(hostConfig);

  preferences.begin("PinSimESP32", false);

  setupPins();
  delay(500);

  // rumble test (hold Left Flipper on boot)
  if (digitalRead(pinLB) == LOW) {
    digitalWrite(rumbleSmall, 1);
    delay(500);
    digitalWrite(rumbleSmall, 0);
    digitalWrite(rumbleLarge, 1);
    delay(500);
    digitalWrite(rumbleLarge, 0);
  }

  // Hold Right Flipper on boot to disable accelerometer
  if (digitalRead(pinRB) == LOW) {
    accelerometerEnabled = false;
    leftStickJoy = true;
  }

  /* Initialise the sensor */
  if (accelerometerEnabled) {
    if (!accel.begin()) {
      /* There was a problem detecting the ADXL345 ... check your connections */
      accelerometerEnabled = false;
      flashStartButton();
    }
  }

  if (accelerometerEnabled) {
    accel.setRange(ADXL345_RANGE_2_G);
    delay(2500);  // time to lower the cabinet lid
    sensors_event_t event;
    accel.getEvent(&event);
    zeroX = event.acceleration.x * nudgeMultiplier * -1;
    zeroY = event.acceleration.y * nudgeMultiplier * -1;
  }

  // plunger setup
  plungerMin = getPlungerAverage();
  if (plungerEnabled) {
    plungerMin = preferences.getInt("plungerMin", 0);  // With default value 0
    plungerMax = preferences.getInt("plungerMax", 0);  // With default value 0
  } else plungerMin = getPlungerAverage();

  // to calibrate, hold A or START when powering up the PinSim ESP32 board
  if (digitalRead(pinB1) == LOW) getPlungerMax();
  else if (digitalRead(pinST) == LOW) getPlungerMax();

  // linear conversions
  if (plungerEnabled) {
    plungerMaxDistance = readingToDistance(plungerMin);
    plungerMinDistance = readingToDistance(plungerMax);
    lastDistance = plungerMaxDistance;
  }
}


// Update the debounced button statuses
void buttonUpdate()
{
  dpadUP.loop();
  dpadDOWN.loop();
  dpadLEFT.loop();
  dpadRIGHT.loop();
  button1.loop();
  button2.loop();
  button3.loop();
  button4.loop();
  buttonLB.loop();
  buttonRB.loop();
  buttonLT.loop();
  buttonRT.loop();
  buttonSTART.loop();
  buttonBACK.loop();
  buttonXBOX.loop();
  button9.loop();
  button10.loop();

  buttonStatus[POSUP] = dpadUP.isPressed();
  buttonStatus[POSDN] = dpadDOWN.isPressed();
  buttonStatus[POSLT] = dpadLEFT.isPressed();
  buttonStatus[POSRT] = dpadRIGHT.isPressed();
  buttonStatus[POSB1] = button1.isPressed();
  buttonStatus[POSB2] = button2.isPressed();
  buttonStatus[POSB3] = button3.isPressed();
  buttonStatus[POSB4] = button4.isPressed();
  buttonStatus[POSL1] = buttonLB.isPressed();
  buttonStatus[POSR1] = buttonRB.isPressed();
  buttonStatus[POSL2] = buttonLT.isPressed();
  buttonStatus[POSR2] = buttonRT.isPressed();
  buttonStatus[POSST] = buttonSTART.isPressed();
  buttonStatus[POSBK] = buttonBACK.isPressed();
  buttonStatus[POSXB] = buttonXBOX.isPressed();
  buttonStatus[POSB9] = button9.isPressed();
  buttonStatus[POSB10] = button10.isPressed();
}


void deadZoneCompensation()
{
  zeroValue = map(distanceBuffer, plungerMaxDistance, plungerMinDistance, 0, -32768) + 10;
  if (zeroValue > 0) zeroValue = 0;
  flashStartButton();
  buttonUpdate();
  // ensure just one calibration per button press
  while (digitalRead(POSBK) == LOW) {
    // wait...
  }
}


// ProcessInputs
void processInputs()
{
  if (leftStickJoy) {
    int leftStickX = buttonStatus[POSLT] * -30000 + buttonStatus[POSRT] * 30000;
    int leftStickY = buttonStatus[POSDN] * -30000 + buttonStatus[POSUP] * 30000;
    gamepad->setLeftThumb(leftStickX, leftStickY);
    gamepad->releaseDPad();
  } else {
    // Update the DPAD
    uint8_t direction = NONE;
    if (buttonStatus[POSUP]) direction |= NORTH;
    if (buttonStatus[POSRT]) direction |= EAST;
    if (buttonStatus[POSDN]) direction |= SOUTH;
    if (buttonStatus[POSLT]) direction |= WEST;
    gamepad->pressDPadDirectionFlag((XboxDpadFlags)direction);
  }

  // If Xbox "Back" and joystick Up pressed simultaneously, map joystick to Xbox Left Stick
  // If Xbox "Back" and joystick Down pressed, map joystick to D-pad
  if (leftStickJoy && buttonStatus[POSDN] && buttonStatus[POSBK]) {
    leftStickJoy = false;
  } else if (!leftStickJoy && buttonStatus[POSUP] && buttonStatus[POSBK]) {
    leftStickJoy = true;
  }

  // If Xbox "Back" and joystick Left pressed simultaneously, use normal L1 & R1
  // If Xbox "Back" and joystick Right pressed, use analog L2 & R2
  if (!flipperL1R1 && buttonStatus[POSLT] && buttonStatus[POSBK]) {
    flipperL1R1 = true;
  } else if (flipperL1R1 && buttonStatus[POSRT] && buttonStatus[POSBK]) {
    flipperL1R1 = false;
  }

  // Buttons
  if (buttonStatus[POSB1]) gamepad->press(XBOX_BUTTON_A);
  else gamepad->release(XBOX_BUTTON_A);
  if (buttonStatus[POSB2]) gamepad->press(XBOX_BUTTON_B);
  else gamepad->release(XBOX_BUTTON_B);
  if (buttonStatus[POSB3]) gamepad->press(XBOX_BUTTON_X);
  else gamepad->release(XBOX_BUTTON_X);
  if (buttonStatus[POSB4]) gamepad->press(XBOX_BUTTON_Y);
  else gamepad->release(XBOX_BUTTON_Y);
  if (buttonStatus[POSB9]) gamepad->press(XBOX_BUTTON_LS);
  else gamepad->release(XBOX_BUTTON_LS);
  if (buttonStatus[POSB10]) gamepad->press(XBOX_BUTTON_RS);
  else gamepad->release(XBOX_BUTTON_RS);

  // If BACK and Left Flipper pressed simultaneously, set new plunger dead zone
  // Compensates for games where the in-game plunger doesn't begin pulling back until
  // the gamepad is pulled back ~half way. Just pull the plunger to the point just before
  // it begins to move in-game, and then press BACK & LB.
  if (buttonStatus[POSBK] && buttonStatus[POSL1]) {
    deadZoneCompensation();
  }

  // detect double contact flipper switches tied to GPIO 13 & 14
  if (!doubleContactFlippers && !fourFlipperButtons) {
    if (buttonStatus[POSL2] && buttonStatus[POSL1]) {
      flipperL1R1 = false;
      doubleContactFlippers = true;
    }
    if (buttonStatus[POSR2] && buttonStatus[POSR1]) {
      flipperL1R1 = false;
      doubleContactFlippers = true;
    }
  }

  // detect four flipper buttons, second pair tied to GPIO 13 & 14
  // detection occurs if GPIO 13 is pressed WITHOUT FLIP_L being pressed
  // or GPIO 14 without FLIP_R being pressed (not possible with double contact switch)
  // 20190511 - Switch must be closed for ~250 ms in order to qualify mode change.
  if (!fourFlipperButtons) {
    if (buttonStatus[POSL2] && !buttonStatus[POSL1]) {
      long currentTime = millis();
      if (fourButtonModeTriggeredLB == 0) {
        fourButtonModeTriggeredLB = currentTime;
      } else if (currentTime > fourButtonModeTriggeredLB + fourButtonModeThreshold) {
        flipperL1R1 = true;
        fourFlipperButtons = true;
        doubleContactFlippers = false;
      }
    }
    // reset check timer if necessary
    else if (fourButtonModeTriggeredLB > 0)
      fourButtonModeTriggeredLB = 0;

    if (buttonStatus[POSR2] && !buttonStatus[POSR1]) {
      long currentTime = millis();
      if (fourButtonModeTriggeredRB == 0) {
        fourButtonModeTriggeredRB = currentTime;
      } else if (currentTime > fourButtonModeTriggeredRB + fourButtonModeThreshold) {
        flipperL1R1 = true;
        fourFlipperButtons = true;
        doubleContactFlippers = false;
      }
    }
    // reset check timer if necessary
    else if (fourButtonModeTriggeredRB > 0)
      fourButtonModeTriggeredRB = 0;
  }

  // Bumpers
  // Standard mode: FLIP_L and FLIP_R map to L1/R1, and optionally GPIO 13 & 14 map to L2/R2
  if (flipperL1R1) {
    uint8_t leftTrigger = 0;
    uint8_t rightTrigger = 0;
    if (buttonStatus[POSL1]) gamepad->press(XBOX_BUTTON_LB);
    else gamepad->release(XBOX_BUTTON_LB);
    if (buttonStatus[POSR1]) gamepad->press(XBOX_BUTTON_RB);
    else gamepad->release(XBOX_BUTTON_RB);

    if (buttonStatus[POSL2]) leftTrigger = 255;
    else leftTrigger = 0;
    if (buttonStatus[POSR2]) rightTrigger = 255;
    else rightTrigger = 0;
    gamepad->setLeftTrigger(leftTrigger);
    gamepad->setRightTrigger(rightTrigger);
  }

  // L2/R2 Flippers (standard mode swapped)
  else if (!flipperL1R1 && !doubleContactFlippers) {
    uint8_t leftTrigger = 0;
    uint8_t rightTrigger = 0;
    if (buttonStatus[POSL2]) gamepad->press(XBOX_BUTTON_LB);
    else gamepad->release(XBOX_BUTTON_LB);
    if (buttonStatus[POSR2]) gamepad->press(XBOX_BUTTON_RB);
    else gamepad->release(XBOX_BUTTON_RB);

    if (buttonStatus[POSL1]) leftTrigger = 255;
    else leftTrigger = 0;
    if (buttonStatus[POSR1]) rightTrigger = 255;
    else rightTrigger = 0;
    gamepad->setLeftTrigger(leftTrigger);
    gamepad->setRightTrigger(rightTrigger);
  }

  // Double Contact Flippers
  else if (!flipperL1R1 && doubleContactFlippers) {
    uint8_t leftTrigger = 0;
    uint8_t rightTrigger = 0;
    if (buttonStatus[POSL1] && buttonStatus[POSL2]) {
      leftTrigger = 255;
    } else if (buttonStatus[POSL1] && !buttonStatus[POSL2]) {
      leftTrigger = 25;
    } else if (!buttonStatus[POSL1] && !buttonStatus[POSL2]) {
      leftTrigger = 0;
    }
    if (buttonStatus[POSR1] && buttonStatus[POSR2]) {
      rightTrigger = 255;
    } else if (buttonStatus[POSR1] && !buttonStatus[POSR2]) {
      rightTrigger = 25;
    } else if (!buttonStatus[POSR1] && !buttonStatus[POSR2]) {
      rightTrigger = 0;
    }
    gamepad->setLeftTrigger(leftTrigger);
    gamepad->setRightTrigger(rightTrigger);
  }

  // Middle Buttons
  if (buttonStatus[POSL2]) gamepad->press(XBOX_BUTTON_LB);
  else gamepad->release(XBOX_BUTTON_LB);

  if (buttonStatus[POSST] && buttonStatus[POSBK]) {
    gamepad->press(XBOX_BUTTON_HOME);
  } else if (buttonStatus[POSST]) {
    gamepad->press(XBOX_BUTTON_START);
  } else if (buttonStatus[POSBK]) {
    gamepad->press(XBOX_BUTTON_SELECT);
  } else if (buttonStatus[POSXB]) {
    gamepad->press(XBOX_BUTTON_HOME);
  } else {
    gamepad->release(XBOX_BUTTON_HOME);
    gamepad->release(XBOX_BUTTON_START);
    gamepad->release(XBOX_BUTTON_SELECT);
  }

  // Experimental Analog Input
  // Analog flippers
  if (analogFlippers) {
    uint8_t leftTrigger = map(analogRead(pinLT), 0, 512, 0, 255);
    uint8_t rightTrigger = map(analogRead(pinRT), 0, 512, 0, 255);
    gamepad->setLeftTrigger(leftTrigger);
    gamepad->setRightTrigger(rightTrigger);
  }

  // Tilt
  if (accelerometerEnabled && !leftStickJoy) {
    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);

    // Zero accelerometer when START is first pressed (PinSim Yellow Start Button)
    if (buttonStatus[POSST] && !accelerometerCalibrated) {
      accelerometerCalibrated = true;
      zeroX = event.acceleration.x * nudgeMultiplier * -1;
      zeroY = event.acceleration.y * nudgeMultiplier * -1;
    }

    // Re-calibrate accelerometer if both BACK and RIGHT FLIPPER pressed
    if (buttonStatus[POSBK] && buttonStatus[POSR1]) {
      accelerometerCalibrated = true;
      zeroX = event.acceleration.x * nudgeMultiplier * -1;
      zeroY = event.acceleration.y * nudgeMultiplier * -1;
    }

    int leftStickX = zeroX + (event.acceleration.x * nudgeMultiplier);
    int leftStickY = zeroY + (event.acceleration.y * nudgeMultiplier);
    if (millis() > tiltEnableTime) {
      gamepad->setLeftThumb(leftStickX, leftStickY);
    }
  }

  // Plunger
  // This is based on the Sharp GP2Y0A51SK0F Analog Distance Sensor 2-15cm
  if (plungerEnabled) {
    int reading = analogRead(pinPlunger);

    if ((reading - lastReading) > -10 && (reading - lastReading) < 10 || (reading - lastReading > 75) || (reading - lastReading < -75)) {
      ave.push(reading);
    }
    lastReading = reading;
    int16_t averageReading = ave.mean();

    // it appears the distance sensor updates at about 60hz, no point in checking more often than that
    if (millis() > plungerReportTime) {
      // restore zero value after a plunge
      if (zeroValueBuffer) {
        zeroValue = zeroValueBuffer;
        zeroValueBuffer = 0;
      }
      plungerReportTime = millis() + plungerReportDelay;
      int16_t currentDistance = readingToDistance(averageReading);
      distanceBuffer = currentDistance;

      // if plunger is pulled
      if (currentDistance + 50 < plungerMaxDistance && currentDistance > plungerMinDistance + 50) {
        currentlyPlunging = true;
        // Attempt to detect plunge
        int16_t adjustedPlungeTrigger = map(currentDistance, plungerMaxDistance, plungerMinDistance, plungeTrigger / 2, plungeTrigger);
        if (currentDistance - lastDistance >= adjustedPlungeTrigger) {
          // we throw STICK_RIGHT to 0 to better simulate the physical behavior of a real analog stick
          gamepad->setRightThumb(0, 0);
          // disable plunger momentarily to compensate for spring bounce
          plungerReportTime = millis() + 1000;
          distanceBuffer = plungerMaxDistance;
          lastDistance = plungerMaxDistance;
          if (zeroValue) {
            zeroValueBuffer = zeroValue;
            zeroValue = 0;
          }
          return;
        }
        lastDistance = currentDistance;

        // Disable accelerometer while plunging and for 1 second afterwards.
        if (currentDistance < plungerMaxDistance - 50) tiltEnableTime = millis() + 1000;
      }

      // cap max
      else if (currentDistance <= plungerMinDistance + 50) {
        currentlyPlunging = true;
        gamepad->setRightThumb(0, -32760);
        distanceBuffer = plungerMinDistance;
        tiltEnableTime = millis() + 1000;
      }

      // cap min
      else if (currentDistance > plungerMaxDistance) {
        currentlyPlunging = false;
        distanceBuffer = plungerMaxDistance;
      }

      else if (currentlyPlunging) currentlyPlunging = false;
    }

    if (currentlyPlunging) {
      gamepad->setRightThumb(0, map(distanceBuffer, plungerMaxDistance, plungerMinDistance, zeroValue, -32760));
    } else {
      gamepad->setRightThumb(0, map(distanceBuffer, plungerMaxDistance, plungerMinDistance, 0, -32760));
    }
  }
}


void loop()
{
  // Poll Buttons
  buttonUpdate();

  if (compositeHID.isConnected()) {
    // Update the LED display
    digitalWrite(pinLEDg, HIGH);

    // Process all inputs and load up the usbData registers correctly
    processInputs();

    // Send controller data
    gamepad->sendGamepadReport();
  }
  else {
    digitalWrite(pinLEDg, LOW);
  }
  delay(2);
}
