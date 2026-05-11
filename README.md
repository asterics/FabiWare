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

The FabiWare trigger system lets you assign actions to rich button gestures — from a simple press or release, through multi-tap counts, to composite sequences that chain multiple buttons together.

## Trigger Types

There are four trigger types:

| Type | Syntax | Fires when... |
|------|--------|---------------|
| `press` | `press(button)` | Button is pressed down (immediate) |
| `release` | `release(button)` | Button is released (immediate) |
| `tap` | `tap(button)` or `tap(button, N)` | Button is tapped N times within the multi-press window |
| `long` | `long(button)` or `long(button, ms)` | Button is held for at least `ms` milliseconds (or `AT LP` threshold if omitted) |

- N for `tap` can be 1–10. `tap(B1)` is equivalent to `tap(B1,1)`.
- A `long` trigger with an explicit duration overrides the global `AT LP` threshold for that tier.

---

## Enabling Thresholds

Triggers that rely on timing are disabled while the relevant threshold is `0`:

```
AT LP 1000    # long-press threshold in ms (default startup value)
AT MP 400     # multi-press window in ms (required for tap(btn, N≥2) triggers)
```

- `AT LP <ms>`: long-press threshold — `0` disables long-press triggers
- `AT MP <ms>`: multi-press window — `0` disables tap-count triggers and composite tap sequencing
- Fresh default initialization uses `AT LP 1000` and `AT MP 400`.

---

## Assigning Triggers

Specify the trigger condition and action on a single line, separated by a comma:

```
AT TG <trigger_expression>, <AT command>
```

Examples:

```
AT TG press(B1), KP KEY_A              # Press B1 → type KEY_A (fires on press, immediately)
AT TG release(sip), CR                 # Release sip → right-click
AT TG tap(B1), CL                      # Single tap on B1 → left-click
AT TG tap(B1,2), KP KEY_ESC            # Double-tap B1 → ESC
AT TG tap(sip,5), KW hello             # 5 rapid sips → type "hello"
AT TG long(B1), NE                     # Hold B1 → next slot
AT TG long(B1,2000), RA                # Hold B1 for 2 s → release all keys
```

The action after the comma follows the same syntax as standalone `AT` commands (`KP`, `KW`, `KH`, `KR`, `CL`, `CR`, `CM`, `NE`, `RA`, `WA`, `MA`, `IR`, `WS`, `MX`, `MY`, etc.). `AT BM` is removed and cannot be used inside trigger actions.

---

## Multi-Tier Long-Press

You can define multiple long-press tiers on the same button with different durations. Each tier fires exactly once as the button is held longer and longer:

```
AT LP 800

AT TG long(B1,800),  KP KEY_A       # fires at 800 ms
AT TG long(B1,1500), KP KEY_B       # fires at 1500 ms
AT TG long(B1,3000), KP KEY_C       # fires at 3000 ms
```

- Shorter tiers fire as each threshold is crossed during the hold.
- If the button is released before the next tier is crossed, the last-exceeded tier fires on release.

---

## Composite Trigger Sequences

Use `+` to chain multiple terms into a sequence. All terms must be satisfied in order within the multi-press window:

```
AT TG term1+term2[+term3...], <action>
```

Examples:

```
AT MP 400
AT LP 1500

# Double-tap B1, then single-tap sip (within 400 ms of releasing B1)
AT TG tap(B1,2)+tap(sip), KP KEY_A

# Double-tap B1, then hold B3 for 1500 ms
AT TG tap(B1,2)+long(B3), KP KEY_ENTER

# Press B1, then release B2 (immediate event types)
AT TG press(B1)+release(B2), CL

# Single up, double right, single sip (three-term chain)
AT TG tap(up)+tap(right,2)+tap(sip), CR
```

### Timing Rules for Sequences

- The multi-press window (`AT MP`) is measured **from when the preceding button is released**, giving natural reaction time.
- `long()` terms are exempt from the MP timeout — they fire after a timed hold, then the next term window opens.
- `press()` and `release()` terms are instantaneous and do not consume the MP window.

---

## Supported Button Names

| Category | Names |
|----------|-------|
| Physical buttons | `B1` … `B9` |
| Stick/cursor | `up`, `down`, `left`, `right` |
| Sip & Puff | `sip`, `puff`, `strongsip`, `strongpuff` |

> **Note:** Directional gesture buttons (`ssup`, `ssdown`, `ssleft`, `ssright`, `spup`, `spdown`, `spleft`, `spright`) have been removed. Strong sip/puff is now a single event (`strongsip`, `strongpuff`) without gesture direction detection.

---

## Trigger Management

```
AT TG list           # Print all active triggers for the current slot
AT TG clear          # Remove all triggers for the current slot
AT TG clear(B1)      # Remove all triggers involving button B1
AT TG clear(3)       # Remove the 3rd trigger by list position (1-based)
```

---

## Hierarchical Press Resolution

When a button has multiple tap-count triggers defined, resolution is deferred until the multi-press window closes so the most specific matching action is selected:

- 1 press + timeout → `tap(btn)` / `tap(btn,1)` trigger
- 2 presses in window → `tap(btn,2)` trigger
- 3 presses in window → `tap(btn,3)` trigger

Hold-style actions (`KH`, `HL`/`HR`/`HM`, joystick hold, etc.) are always cleanly released before a trigger fires.

`press()` and `release()` triggers bypass this deferral and fire immediately.

---

## Saving and Loading

Triggers are saved and restored with slots. `AT TG clear` is written before trigger definitions when a slot is saved, so loading a slot cleanly replaces the previous slot's triggers.

> **Breaking change (settings revision 3):** Slots saved with earlier firmware that still relied on `AT BM`, or were saved during the intermediate broken trigger-serialization revision, will not be loaded by this firmware. Re-configure and re-save slots using `AT TG` triggers.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| Triggers never fire | `AT LP` or `AT MP` is `0` — set them to nonzero values |
| Wrong trigger fires | Check with `AT TG list`; reassign if needed |
| Multi-tap not detected | Increase `AT MP` — your press sequence may be slower than the window |
| Long-press fires too early / too late | Adjust `AT LP` or the explicit duration in `long(btn, ms)` |
| Composite sequence misses a step | Increase `AT MP`; ensure you release the prior button before starting the next |


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
