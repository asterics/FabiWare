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

# Complex Trigger System (AT TG Command)

The FabiWare trigger system assigns actions to button gestures:
- Long-press: hold a button for a configured time
- Double-press: press twice inside the multi-press window
- Triple-press: press three times inside the multi-press window

## Important Prerequisite: Enable Thresholds

Triggers are disabled while thresholds are `0`.

```
AT LP 1500    # long-press threshold in ms
AT MP 400     # multi-press threshold in ms
```

- `AT LP <ms>`: long-press threshold (`0` disables long-press triggers)
- `AT MP <ms>`: multi-press threshold (`0` disables double/triple triggers)

## Basic Trigger Assignment (Two-Step)

`AT TG` works in two steps (similar to `AT BM`):

1. Send trigger specification
2. Send the action command to store

The action is stored, not executed during assignment.

### Basic Examples

```
# Long-press on B1
AT LP 1500
AT TG long(B1)
AT KP KEY_A

# Double-press on sip
AT MP 400
AT TG double(sip)
AT CR

# Triple-press on puff
AT MP 400
AT TG triple(puff)
AT CL
```

## Current Trigger Behavior

### Hierarchical multi-press resolution

If a button has double/triple triggers configured, the normal single action is deferred until the multi-press window closes.

- 1 press and timeout: normal single action executes
- 2 presses in window: double trigger executes
- 3 presses in window: triple trigger executes

Only one of these outcomes is selected for that press sequence.

### Hold behavior and safety

Hold-style normal actions (`KH`, `HL/HR/HM`, `PL/PR/PM`, joystick hold-style actions, etc.) are handled safely when double/triple triggers exist:

- The normal hold action is delayed until it is clear no double/triple gesture will happen
- If a long/double/triple trigger takes over, active hold state is released before the trigger action runs
- Deferred hold actions are never left sticky; release is always enforced when transitioning between actions

## Supported Button Names for `AT TG`

You can assign triggers to:

- Physical buttons: `B1`, `B2`, `B3`, `B4`, `B5`
- Cursor buttons: `up`, `down`, `left`, `right`
- Sip/Puff buttons: `sip`, `puff`, `strongsip`, `strongpuff`

The names below are still recognized, but trigger assignment is currently disabled for them:

- `ssup`, `ssdown`, `ssleft`, `ssright`
- `spup`, `spdown`, `spleft`, `spright`

## Trigger Management Commands

```
AT TG list
AT TG clear
AT TG clear(B1)
```

## Notes on Saving/Loading Slots

Triggers are saved and restored with slots.

`AT TG clear` is intentionally written before saved trigger definitions so loading a slot first removes old triggers from the previously active slot.

## Minimal Troubleshooting

- Triggers do not fire: ensure `AT LP`/`AT MP` are set to values greater than `0`
- Trigger missing or wrong action: verify with `AT TG list` and reassign if needed
- Multi-press not detected: increase `AT MP` if your press sequence is slower

# Links and Credits
Most of this work has been accomplished at the UAS Technikum Wien in course of the R&D-projects [ToRaDes](https://embsys.technikum-wien.at/projects/torades/index.php) (MA23 project 18-04),
[WBT](https://wbt.wien) (MA23 project 26-02) and [InDiKo](https://www.technikum-wien.at/en/research-projects/indiko/) (MA23 project 38-09), which have been supported by the [City of Vienna](https://www.wien.gv.at/kontakte/ma23/index.html).

# Support us
Please support the development of Open Source Assistive Technology projects by donating to the AsTeRICS Foundation:

<div>
<a title="Donate with PayPal" href="https://www.paypal.com/donate/?hosted_button_id=38AJJNS427MJ2" target="_blank" style="margin-right:3em">
<img src="https://github.com/asterics/AsTeRICS-Grid/raw/master/app/img/donate-paypal.png" width=300/></a>
<span>&nbsp;&nbsp;&nbsp;</span>
<a title="Donate at opencollective.com" href="https://opencollective.com/asterics-foundation" target="_blank">
<img src="https://github.com/asterics/AsTeRICS-Grid/raw/master/app/img/donate-open-collective.png" width=300/></a>
</div>
