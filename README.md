# Welcome to FabiWare
FabiWare is the official firmware for different alternative computer input devices including [FABI](https://github.com/asterics/FABI), 
[FlipMouse](https://github.com/asterics/FLipMouse) and [FlipPad](https://github.com/asterics/FLipPad). 
(The FabiWare firmware is compatible with these devices since HW-version 3, using the Raspberry Pi Pico familiy of microcontrollers. 
Prior HW versions use a different firmware which is still available in the respective repositories.)

## Configuration Manager and User manuals
Devices running FabiWare can be configured using a [Web-based configuration editor](https://fabi.asterics.eu/). This allows storing multiple configuration 
settings and changing them "on-the-fly". Find more information how to configure the individual button functions and system options in the user manuals for the FABI/FlipMouse devices:

* [FlipMouse user manual](https://github.com/asterics/FLipMouse/tree/master/Documentation/UserManual)
* [FABI user manual](https://github.com/asterics/FABI/tree/master/Documentation/UserManual)

(TBD: update manuals for new FabiWare functions)

## Building and Installing the Firmware

* Clone this repository to your local file system
* Install [PlatformIO with VSCode Integration](https://platformio.org/install/ide?install=vscode)
* Add the repository folder to the PlatformIO workspace (File -> Add Folder to Workspace)
* Select the desired target (build environment) for Raspberry Pi Pico/PicoW (FABI_RP2040), Raspberry Pi Pico 2/2W (FABI_RP2350), or Arduino Nano 2040 Connect (FLIPMOUSE).
* Build (and optionally upload) the firmware (via the PlatformIO GUI buttons or terminal commands, e.g. `pio run -e FABI_RP2350 -t upload`)

After the build process, you can find the .uf2 files in the newly created folder `build/`.

### Utilized Resources and Libraries
The necessary dependencies should be installed automatically in course of the PlatformIO build process. 
Many thanks to the people behind the following projects:

* the [RP Pico core](https://github.com/earlephilhower/arduino-pico) by Earle Philhower
* the [RP Pico PlatformIO support](https://github.com/maxgerhardt/platform-raspberrypi) by Max Gerhardt
* the [Adafruit Neopixel](https://github.com/adafruit/Adafruit_NeoPixel) and [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO) libraries
* the [SSD1306 Ascii](https://github.com/greiman/SSD1306Ascii) library by Bill Greiman
* our own libraries/library adatations: [LoadCellSensor](https://github.com/ChrisVeigl/LoadcellSensor) and [NAU7802-DualChannel](https://github.com/benjaminaigner/NAU7802-DualChannel)

## Example setups and applications
Have a look at the [AsTeRICS Foundation homepage](https://www.asterics-foundation.org) for applications and our other Open Source projects:

* [FABI - the Flexible Assistive Button Interface](https://github.com/asterics/FABI) - an open source switch interface for USB HID and Bluetooth.
* [The FLipMouse controller](https://github.com/asterics/FLipMouse) - an open source alternative input device for controlling computers and mobile devices with minimal muscle movement.
* [The FLipPad controller](https://github.com/asterics/FLipMouse) - a flexible touchpad for controlling computers and mobile devices with minimal muscle movement.
* [Asterics Grid Open Source AAC](https://grid.asterics.eu) - an open source, cross plattform communicator / talker for Augmented and Alternative Communication (AAC).
* [The AsTeRICS framework](https://github.com/asterics/AsTeRICS) - provides high flexibility for building Assistive Technology solutions. 

# Links and Credits
Most of this work has been accomplished at the UAS Technikum Wien in course of the R&D-projects *ToRaDes* (MA23 project 18-04), *WBT* (MA23 project 26-02) and Indiko (MA23 project 38-09), which have been supported by the [City of Vienna](https://www.wien.gv.at/kontakte/ma23/index.html),
see: [ToRaDes Project Information](https://embsys.technikum-wien.at/projects/torades/index.php), [Webpage WBT project](https://wbt.wien), [Webpage InDiKo project](https://www.technikum-wien.at/en/research-projects/indiko/).

# Support us
Please support the development of Open Source Assistive Technology projects by donating to the AsTeRICS Foundation:

<div>
<a title="Donate with PayPal" href="https://www.paypal.com/donate/?hosted_button_id=38AJJNS427MJ2" target="_blank" style="margin-right:3em">
<img src="https://github.com/asterics/AsTeRICS-Grid/raw/master/app/img/donate-paypal.png" width=300/></a>
<span>&nbsp;&nbsp;&nbsp;</span>
<a title="Donate at opencollective.com" href="https://opencollective.com/asterics-foundation" target="_blank">
<img src="https://github.com/asterics/AsTeRICS-Grid/raw/master/app/img/donate-open-collective.png" width=300/></a>
</div>
