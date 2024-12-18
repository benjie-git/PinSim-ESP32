# PinSim

## ESP32-based port of PinSim

I created this port due to the lack of availability of the Teensy 3.1 dev boards required for the old PinSim project.
I have also designed an updated, drop-in replacement for the old PCB.  The new one comes assembled with surface mount 
parts including an ESP32-S3, a USB-C connector, and the accelerometer.


## XInput Game Controller for PC Pinball Games

Based on the excellent [MSF_FightStick XINPUT](https://github.com/zlittell/MSF-XINPUT) project by Zack "Reaper" Littell

#### The complete project build guide, along with PCB links, wiring schematics, and 3D STL files, is [hosted here](https://github.com/Jerware/PinSim/blob/master/pinsim_instructions.pdf) and backed up on [Archive.org](https://web.archive.org/web/20180209074729/https://www.tested.com/tech/gaming/569647-how-build-pinsim-virtual-reality-pinball-machine/).

This code is designed to run on Teensy LC and requires the following libraries:

- [MSF-XInput](https://github.com/zlittell/MSF-XINPUT/tree/master/MSF_XINPUT)
- [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [Adafruit ADXL345](https://github.com/adafruit/Adafruit_ADXL345)
- [Bounce](https://www.pjrc.com/teensy/td_libs_Bounce.html)
- [Average](https://github.com/MajenkoLibraries/Average)
- [EEPROMex](https://github.com/thijse/Arduino-EEPROMEx)

Please note the instructions for the MSF-XInput library, which requires some Teensyduino files to be overwritten in order to add support for a new USB device type.

I have included compiled code that can be installed on a Teensy LC using the [Teensy Loader](https://www.pjrc.com/teensy/loader.html) application. The code will compile for Teensy 3.2 but, besides being more expensive, that board lacks sufficient current output on the LED pins.

#### OPTIONS OVERVIEW:

* BACK + LEFT FLIPPER: Home analog plunger (if installed)
* BACK + RIGHT FLIPPER: Home/Zero accelerometer
* BACK + JOYSTICK DOWN: Joystick controls D-PAD (default)
* BACK + JOYSTICK UP: Joystick controls left analog stick
* BACK + JOYSTICK LEFT: L1 & R1 flippers (default)
* BACK + JOYSTICK RIGHT: L2 & R2 flippers

Note: If installed, you MUST calibrate the analog plunger range at least once by holding down "A" when plugging in the USB cable. LED-1 should flash rapidly, and then you should pull the plunger all the way out and release it all the way back in. The LED1 should flash again, and normal operation resumes. The setting is saved between power cycles.

#### New features added 05/17/2020:

* GPIO 21 & 23 connected to L3 & R3 (analog stick buttons). These GPIO pins were previously unused.

#### New features added 05/16/2019:

* Accelerometer is only zeroed the first time START is pressed, not every time. To zero it again, use the new BACK + RIGHT FLIPPER button combo.
* Plunger base position is now stored between power cycles. You will need to recalibrate the plunger once if upgrading to this firmware by holding A when plugging in the USB cable.
* Auto-detection of four button mode has been improved (avoid false positives).

#### New features added 12/19/2018:

* Four flipper button support. You can connect a second pair of flipper buttons to GPIO 13 & 14 (Left and Right, respectively) which will map to analog L2/R2 100%. The system will try to autodetect if GPIO 13/14 are connected to double contact or individual switches based on whether GPIO 13/14 are ever pressed *without* FLIP_L or FLIP_R being pressed (not possible on double contact switches).
* Minor bug fixes.

#### New features added 11/15/2018:

* Hold Back and press Right on the joystick to map flippers to L2 & R2 analog triggers. This is required by some pinball games.
* Hold Back and press Left on the joystick to map flippers to L1 & R1 buttons (default).

Optionally: You can connect GPIO 13 or 14 to the second stage of double contact switches so FLIP_L & FLIP_R depress the analog triggers by 10% and GPIO 13 & 14 depress them fully. This allows you to control lower and upper flippers independently using a single button (when supported by the game).

(Huge thanks to James Ricalde for his great suggestions and thorough beta testing!)

#### New feature added 5/13/2018:

* Hold Back and press Up on the joystick to map joystick to left analog stick. This is required by Oculus Arcade.
* Hold Back and press Down on the joystick to map joystick to D-pad (default).

#### New features & improvements added 6/20/2016:

* Pressing LB & Back simultaneously sets plunger dead zone. This compensates for PC pinball games that have a huge plunger dead zone that works on Gamepads but is accentuated on real hardware. Just pull the plunger until just before the one in-game starts to move and then hit LB & Back with your other hand. This setting is not saved between power cycles.
* The accelerometer is zeroed out every time Start is pressed.
* Hold left flipper on boot to perform a rumble test.
* Hold right flipper on boot to disable accelerometer.
* Better plunge detection code that scales based on how far back the plunger is pulled.
