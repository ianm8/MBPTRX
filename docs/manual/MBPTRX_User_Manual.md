# MBPTRX User Manual

Firmware version 9.5.240

Copyright 2026 Ian Mitchell VK7IAN. The firmware is licensed under the GNU GPL Version 3.

---

## Contents

1. Introduction
2. Specifications at a glance
3. Controls and connections
4. Switching on
5. The main screen
6. Tuning, bands and modes
7. Using the menu
8. Menu reference
9. Operating SSB
10. Operating CW
11. Operating the digital modes (DGL and DGU)
12. Receiving AM and shortwave (SWL band)
13. Operating FT8
14. Hidden features and tips
15. Settings that are remembered
16. Messages and fault codes
17. Appendix A - Band table
18. Appendix B - Signal connections
19. Appendix C - Building the firmware

---

## 1. Introduction

MBPTRX is a multi-band HF phasing transceiver built around a Raspberry Pi Pico 2 (RP2350). It covers the amateur bands from 80 m to 10 m, plus a general coverage shortwave receive band.

The receiver uses a Tayloe quadrature sampling detector. The I and Q signals are digitised by an audio ADC and everything after that is done in software: sideband selection by the phasing method, filtering, AGC, noise blanking, notch filtering, noise reduction and the spectrum display. The transmitter works the same way in reverse, generating I and Q in software and driving a quadrature sampling exciter. The local oscillator is an Si5351 with a TCXO reference.

One processor core runs the signal processing and the other runs the display and controls, so the audio is not interrupted by the user interface.

Main features:

- SSB (LSB and USB), CW, AM receive, VOX-keyed digital modes, and a complete built-in FT8 mode that needs no computer
- 240 x 135 colour display with spectrum and waterfall
- Automatic mode selection by frequency
- Selectable receive bandwidth, noise blanker, tunable notch filter and noise reduction
- Iambic A and B keyer, straight key input, and two CW decoders
- Microphone processor and CESSB speech processing
- Forward power and SWR metering
- Everything is controlled with one rotary encoder and its push button

---

## 2. Specifications at a glance

| Item | Value |
|---|---|
| Amateur bands | 80, 40, 30, 20, 17, 15, 12 and 10 m |
| Shortwave band (SWL) | 3.5 to 30 MHz, receive only |
| Modes | LSB, USB, CWL, CWU, DGL, DGU, FT8, AM (receive only) |
| Tuning steps | 10 Hz, 100 Hz, 500 Hz, 1 kHz, 5 kHz, 10 kHz, 100 kHz |
| SSB receive bandwidth | 2000, 2200, 2400, 2600 or 2800 Hz |
| CW receive filter | 600 to 800 Hz (200 Hz wide, centred on 700 Hz) |
| CW pitch and sidetone | 600 to 800 Hz in 50 Hz steps |
| CW keying | Gaussian shaped, about 5 ms rise and fall |
| Digital and FT8 bandwidth | 3000 Hz |
| SSB transmit bandwidth | 2600 Hz |
| Spectrum display | About 14 kHz either side of the tuned frequency, with waterfall |
| CW keyer speed | 10, 15, 20, 25 or 30 WPM |
| CW decoder range | About 8 to 34 WPM |
| Notch filter | 300 to 2600 Hz, about 30 dB deep |
| AGC | Automatic. Fast attack, about 10 dB per second decay |
| Power meter | 0 to 10 W bar graph, or a numeric reading |
| Sample rate | 31250 Hz |
| Processor | Raspberry Pi Pico 2 at 240 MHz |

---

## 3. Controls and connections

### 3.1 Rotary encoder

The encoder does almost everything. What it does depends on what the radio is doing at the time.

| Situation | Turning the encoder |
|---|---|
| Normal receive | Tunes the radio |
| Menu open | Moves the highlight |
| Setting the spectrum level | Changes the level |
| Adjusting the notch | Moves the notch frequency |
| Transmitting (SSB, CW, DGL, DGU) | Sets the transmit power level |
| FT8 mode | Scrolls through decoded messages |

### 3.2 Encoder push button

| Situation | Short press | Long press |
|---|---|---|
| Normal receive | Changes the tuning step | Opens the menu (hold for just under a second) |
| Menu open | Selects the highlighted item | - |
| Setting spectrum level or notch | Finishes the adjustment | - |
| FT8 mode | Calls CQ, or answers the highlighted message | Stops the current activity, or exits FT8 |

### 3.3 PTT

The PTT line keys the transmitter in LSB and USB. In CW modes it works as a **straight key** input. It does nothing in the digital modes, FT8 or AM.

### 3.4 Paddles

Paddle A sends dits and paddle B sends dahs. They are only active in CWL and CWU.

### 3.5 Microphone input

The microphone input is used for speech in LSB and USB. In DGL and DGU it is the audio input from a computer or other digital mode source, and it also drives the VOX.

### 3.6 LED

The LED on the Pico lights while the radio is transmitting in SSB, the digital modes and FT8, and while the key is down in CW. It also flashes fault codes if the radio fails its power-on self-test (see section 16.3).

---

## 4. Switching on

### 4.1 Start-up sequence

1. The LED lights for one second.
2. The radio checks its clock and its hardware. If something is missing the LED flashes a fault code and the radio stops (see section 16.3).
3. The MBPTRX splash screen is shown, with the firmware version in the bottom left corner.
4. Your callsign is shown for two seconds.
5. The audio is unmuted and the radio starts receiving on 7.100 MHz.

The version is shown as, for example, `V9.5.240`. The last number is the measured processor clock in MHz and should always read 240.

### 4.2 First power-on: entering your callsign and grid

If no valid callsign and grid locator are stored, the radio asks for them before it will start. They are needed for FT8.

The callsign screen has six character cells and two buttons, BACK and SAVE.

| When the cursor is on | Turn | Push |
|---|---|---|
| A character cell | Change the character | Move to the next cell |
| The last character cell | Change the character | Jump to SAVE |
| BACK or SAVE | Move between the buttons | Do it |

- The characters available are A to Z, 0 to 9, `/` and blank. Leave unused cells blank.
- BACK returns the cursor to the first character so you can fix a mistake.
- SAVE checks the entry. A callsign must be 3 to 6 characters long and contain at least one letter and one digit. If it is rejected, a red message is shown for a moment and you can correct it.

The grid screen works the same way, with four cells: two letters (A to R) followed by two digits, for example `QE37`.

When both are accepted the radio shows SAVED with your callsign and grid, stores them, and carries on starting up.

To change your callsign later, use **Callsign** in the menu (section 8.19).

### 4.3 Transmit lock-out at start-up

If the PTT or either paddle is closed when the radio is switched on, the PTT and paddles are locked out and will not key the transmitter. This protects against a stuck key or a shorted lead. The frequency is displayed in **yellow** while the lock-out is active.

To clear it, switch off, fix the cause, and switch on again with the PTT and paddles released.

Notes:

- The lock-out covers the PTT and paddles only. FT8 mode transmits regardless.
- Selecting DGL or DGU also clears the lock-out for the rest of the session.

---

## 5. The main screen

```
+------------------------------------------+
| TX RX        7.100.000                   |
| [LSB] [40M]  1 3 5 7 9 +20    [T]  [61%] |
|              STEP 1000                   |
| CESSB:Off Mic:100% BW:2400Hz NB0  JNR:1  |
| -14KHz         spectrum          +14KHz  |
|                waterfall                 |
|                waterfall                 |
+------------------------------------------+
```

| Item | Meaning |
|---|---|
| TX / RX | The active one is white and the other is blue. On the SWL band or in AM both are green, because transmit is not available |
| Frequency | White on receive, red on transmit, yellow if the PTT and paddles are locked out |
| Mode box | LSB, USB, CWL, CWU, DGL, DGU or AM |
| Band box | Purple for the amateur bands, red for SWL |
| S-meter | Scale `1 3 5 7 9 +20`. Ten blocks is S9, and up to five more show stronger signals. On transmit it shows the microphone or audio input level |
| T | Shown when the notch filter is on |
| Percentage box | Signal processing load. It should always be below 100% |
| STEP | The tuning step in Hz. Replaced by SWR and power on transmit |
| Status line | Settings for the current mode (see below) |
| JNR:n | Noise reduction level, when it is on |
| Spectrum | About 14 kHz either side of the tuned frequency |
| Shaded strip | Shows where the receive passband sits within the spectrum |
| Waterfall | History of the spectrum, newest at the top |

### 5.1 The status line

The status line changes with the mode.

| Mode | Status line shows |
|---|---|
| LSB, USB | `CESSB:On` or `Off`, `Mic:100%`, `BW:2400Hz`, `NB0` to `NB5` |
| CWL, CWU | `WPM:20 Level:MED Sidetone:700`, and a boxed `A` or `S` if a CW decoder is on |
| DGL, DGU | `Vox:On BW:3000Hz CESSB:Off NB:Off` |
| AM | Nothing |

In SSB, the microphone item changes to red, for example `Mic:3P`, when the microphone processor is on. The number is the processor level.

While you are setting the spectrum level the line reads `Spectrum Level: 6dB`. While you are adjusting the notch it reads `Notch: 1000Hz`.

### 5.2 Popup messages

Some actions show a message in a dark red box in the middle of the screen for five seconds, under a small heading such as `ATTENTION:`. In FT8 mode the box is blue. The messages are listed in section 16.

---

## 6. Tuning, bands and modes

### 6.1 Tuning

Turn the encoder to tune. Each click moves by one tuning step, and the frequency always lands on a multiple of the step. Tuning stops at the band edges.

### 6.2 Tuning step

A **short press** of the button multiplies the step by ten, so it cycles through 100 Hz, 1 kHz and 10 kHz. The other steps (10 Hz, 500 Hz, 5 kHz and 100 kHz) are chosen from the **Step** menu. The radio starts with a 1 kHz step.

### 6.3 Bands

Bands are chosen from the **Band** menu. Each band remembers the frequency you last used on it until the radio is switched off. The band-pass and low-pass filters are switched automatically.

The band limits are listed in Appendix A.

### 6.4 Modes and automatic mode selection

The radio starts in **AUTO** mode, where the mode follows the frequency:

| Frequency | Mode chosen |
|---|---|
| SWL band | AM |
| 7.000 MHz up to, but not including, 7.060 MHz | CWL |
| 14.000 MHz up to, but not including, 14.060 MHz | CWU |
| Exactly 3.573 MHz or 7.074 MHz (FT8 frequencies) | USB |
| Anywhere else below 10 MHz | LSB |
| Anywhere else at 10 MHz and above | USB |

Choosing a specific mode from the **Mode** menu turns AUTO off, and that mode then stays selected wherever you tune and whichever band you choose. Choose **AUTO** from the Mode menu to turn it back on. Leaving FT8 mode also turns AUTO back on.

| Mode | Use |
|---|---|
| LSB, USB | Speech |
| CWL, CWU | CW, received on the lower or upper side. Swap between them to move interference to the other side of the filter |
| DGL, DGU | Digital modes with a computer, keyed by VOX |
| FT8 | Built-in FT8 (section 13) |
| AM | AM broadcast reception. Receive only |

---

## 7. Using the menu

1. **Hold the button** for just under a second. The menu opens showing four items at a time.
2. **Turn** to move the highlight. The list scrolls.
3. **Press** to open the highlighted item. Its name is shown with its options listed under it, three at a time.
4. **Turn** to highlight an option and **press** to select it. The menu closes.

Other points:

- A press takes effect when the button is released.
- Every list ends with **Exit**, which closes the menu without changing anything.
- The menu closes by itself after five seconds without any input.
- Pressing the PTT or a paddle closes the menu.
- The menu cannot be opened while transmitting, or from inside FT8 mode.

---

## 8. Menu reference

The menus are listed here in the order they appear on the radio.

### 8.1 Band

| Option | Effect |
|---|---|
| 80M, 40M, 30M, 20M, 17M, 15M, 12M, 10M | Selects an amateur band and returns to the last frequency used on it |
| SWL | Selects the 3.5 to 30 MHz receive-only band |

### 8.2 Step

| Option | Effect |
|---|---|
| 10, 100, 500, 1000, 5000, 10000, 100000 | Sets the tuning step in Hz |

### 8.3 Mode

| Option | Effect |
|---|---|
| AUTO | The mode follows the frequency (section 6.4) |
| LSB, USB | Sideband speech |
| CWL, CWU | CW on the lower or upper side |
| DGL, DGU | VOX-keyed digital modes |
| FT8 | Enters the built-in FT8 mode. Tune to an FT8 frequency on an amateur band first |
| AM | AM receive |

### 8.4 JNR (noise reduction)

| Option | Effect |
|---|---|
| Level 1, Level 2, Level 3 | Progressively stronger smoothing of the audio |
| Off | No noise reduction |

JNR is a simple fixed audio smoothing filter. It removes high-pitched hiss, and higher levels also take more of the treble off the wanted signal. It works in LSB, USB and AM. The level is shown as `JNR:n` on the screen.

### 8.5 Blanker (noise blanker)

| Option | Effect |
|---|---|
| Level 1 | Blanks only very large impulses |
| Level 2 | Conservative |
| Level 3 | Moderate |
| Level 4 | Fairly aggressive |
| Level 5 | Maximum blanking |
| Off | No blanking |

The blanker removes short impulses such as ignition noise and electric fence clicks. It works in LSB and USB only. Use the lowest level that does the job, because high levels can also react to strong speech peaks. The level is shown as `NB0` to `NB5` on the status line.

### 8.6 Bandwidth

| Option | Effect |
|---|---|
| 2000 Hz, 2200 Hz, 2400 Hz, 2600 Hz, 2800 Hz | Sets the SSB receive bandwidth |

This applies to LSB and USB receive only. The other modes use fixed filters.

### 8.7 Notch Filter

| Option | Effect |
|---|---|
| Notch On | Turns the notch on and lets you tune it |
| Notch Off | Turns the notch off |

The notch removes a steady tone, such as a carrier or someone tuning up, from SSB audio. It is placed before the AGC, so a strong carrier does not reduce the receiver gain once it is notched.

To use it:

1. Select **Notch On**. The status line shows `Notch: 1000Hz`.
2. Turn the encoder to move the notch until the tone disappears. It moves 10 Hz per click between 300 and 2600 Hz.
3. Press the button to finish. The notch stays on, a boxed `T` is shown, and the encoder tunes the radio again.

To move the notch later, select **Notch Off** and then **Notch On** again. The notch frequency is kept while the radio is on.

The notch can only be switched on or off while the radio is in LSB or USB. In other modes the radio shows `Notch: SSB only` and leaves it as it was. The notch only acts on SSB reception.

### 8.8 FT8 Options

These options are set **before** entering FT8 mode. The menu is not available inside FT8 mode.

| Option | Effect |
|---|---|
| CQ | CQ calls are sent as a plain `CQ` |
| CQ DX | CQ calls are sent as `CQ DX` |
| CQ WWFF | CQ calls are sent as `CQ WWFF` |
| CQ POTA | CQ calls are sent as `CQ POTA` |
| CQ SOTA | CQ calls are sent as `CQ SOTA` |
| Reset Cal | Discards the FT8 timing calibration and selects manual calibration |
| Auto Cal | Selects automatic calibration from received signals, the next time FT8 is running |
| Monitor On | FT8 receive audio is heard |
| Monitor Off | FT8 mode is silent |

See section 13 for how these are used.

### 8.9 Spectrum

| Option | Effect |
|---|---|
| Wind | Spectrum drawn as a line |
| Grass | Spectrum drawn as a filled, coloured area |
| Set Level | Adjusts the spectrum gain for this band and saves it |
| Adj Level | Adjusts the spectrum gain for this band temporarily |

To set the level:

1. Select **Set Level** or **Adj Level**. The status line shows `Spectrum Level:` and the gain in dB.
2. Turn the encoder. The gain moves in 6 dB steps from -24 dB to +24 dB.
3. Press the button to finish.

Each band has its own level. **Set Level** starts from the saved value and stores the new one, so it is used every time the radio is switched on. **Adj Level** is for a quick change during a session and is not stored.

If you press the PTT or a paddle while adjusting, the adjustment is cancelled and the previous level is restored.

On frequencies above 15 MHz the radio automatically flattens the small hump of noise that would otherwise appear at the centre of the spectrum.

### 8.10 Mic Gain

| Option | Effect |
|---|---|
| Gain 25% to Gain 200%, in steps of 25% | Sets the microphone gain |

The gain is shown on the SSB status line. Set it so that speech peaks move the microphone level meter well up the scale on transmit without holding it at the top. It applies to LSB and USB only.

### 8.11 Mic Proc (microphone processor)

| Option | Effect |
|---|---|
| Level 1 | Subtle |
| Level 2 | Light processing |
| Level 3 | Typical SSB setting, a good starting point |
| Level 4 | Aggressive |
| Level 5 | Heavy, contest style |
| Off | No processing |

The processor is a soft clipper that raises the average level of your speech. Higher levels give more punch and more distortion. When it is on, the status line shows the level in red, for example `Mic:3P`. It applies to LSB and USB transmit.

### 8.12 CW Decode

| Option | Effect |
|---|---|
| Adaptive | Decoder using an adaptive threshold |
| Schmitt | Decoder using two thresholds. More resistant to noise and fading |
| Off | Decoder off |

When a decoder is on in CWL or CWU, decoded text scrolls along the bottom of the screen. A `*` at the end of the line shows when the decoder hears the key down, which helps with tuning. The decoder follows sending speeds from about 8 to 34 WPM. A boxed `A` or `S` on the status line shows which decoder is running.

### 8.13 CW Speed

| Option | Effect |
|---|---|
| 10, 15, 20, 25, 30 WPM | Sets the keyer speed for the paddles |

### 8.14 CW Tone

| Option | Effect |
|---|---|
| 600 Hz to 800 Hz, in steps of 50 Hz | Sets the CW pitch |

This sets two things together: the pitch of the transmit sidetone, and the pitch at which a station exactly on the displayed frequency is received. Matching a station's pitch to your sidetone puts you on its frequency.

The CW receive filter is fixed at 600 to 800 Hz, so 700 Hz puts an on-frequency station in the centre of the filter. At 650 or 750 Hz it is about 1 dB down, and at 600 or 800 Hz it is at the edge of the filter and about 3 dB down.

### 8.15 CW Level

| Option | Effect |
|---|---|
| Low, Medium, High | Sets the sidetone volume |

Each step is 6 dB. The setting is shown on the CW status line.

### 8.16 CW Keyer

| Option | Effect |
|---|---|
| Iambic A | Releasing a squeeze finishes the current element and stops |
| Iambic B | Releasing a squeeze sends one more, opposite, element |

In both modes a quick tap on the other paddle during an element is remembered and sent next.

### 8.17 Graph SWR

| Option | Effect |
|---|---|
| Yes | Power and SWR are shown as bar graphs on transmit |
| No | Power and SWR are shown as numbers on transmit |

With **Yes**, the S-meter is replaced on transmit by a scale marked `1 2 3 5 10` with two bars under it. The upper bar is forward power in watts. The lower bar is SWR, and it turns red above about 1.9:1.

With **No**, the display shows, for example, `SWR 1.25 PO 5.0` where the tuning step is normally shown. PO is forward power in watts.

The power reading holds its peak and decays slowly, so it is easy to read on speech.

### 8.18 CESSB

| Option | Effect |
|---|---|
| On | Controlled envelope SSB processing on transmit |
| Off | Normal SSB |

CESSB limits the peaks of the transmitted envelope and then filters the result, which raises average power without splatter. It applies to LSB and USB transmit and can be used together with the microphone processor.

### 8.19 Callsign

| Option | Effect |
|---|---|
| Clear | Erases the stored callsign and grid. The radio keeps running |
| ..Reboot | Erases the stored callsign and grid and restarts the radio straight away |

To change your callsign or grid, choose **..Reboot**. The radio restarts and asks for the new details (section 4.2).

After **Clear**, FT8 mode is not available until the radio is next switched on and the details are entered again. Selecting FT8 shows `No callsign`.

### 8.20 Exit

Closes the menu.

---

## 9. Operating SSB

### 9.1 Receiving

Select LSB or USB, or leave the mode on AUTO. Tune the station in, then use these as needed:

- **Bandwidth** to narrow the filter against interference
- **Blanker** for impulse noise
- **Notch Filter** for carriers and tuning signals
- **JNR** for hiss

The AGC is automatic, with a fast attack and a slow release.

### 9.2 Transmitting

Press and hold the PTT and speak. On transmit:

- The frequency turns red and TX is highlighted
- The S-meter shows the microphone level
- The spectrum shows your transmitted signal
- SWR and forward power are displayed

Release the PTT to return to receive. The AGC and S-meter are restored to where they were before you transmitted, so the receiver does not need to recover.

### 9.3 Setting transmit power

**While transmitting, turn the encoder to set the power level.** Each click changes the drive by 1%, from 0% to 100%. Turn anticlockwise to reduce power and clockwise to increase it.

- There is no number shown for the level. Watch the power reading while you turn.
- The setting is a drive level, so output power falls faster than the percentage. Half drive gives roughly a quarter of the power.
- The setting stays until the radio is switched off, and it applies to every mode, including FT8.
- The radio always starts at 100%.

### 9.4 Getting the best transmit audio

1. Start with Mic Proc off and CESSB off.
2. Set **Mic Gain** so that speech peaks reach the upper part of the microphone level meter.
3. Add **CESSB** for more average power.
4. Add **Mic Proc** if you want more punch. Level 3 is a good starting point.

---

## 10. Operating CW

### 10.1 Receiving

Select CWL or CWU. On 40 m and 20 m, AUTO mode selects CW for you in the bottom 60 kHz of the band.

The CW filter passes 600 to 800 Hz. Tune so that the station is heard at the same pitch as your sidetone (700 Hz unless you have changed **CW Tone**) and it will be on your transmit frequency. A 10 Hz or 100 Hz tuning step is best for CW.

If there is interference on one side, switch between CWL and CWU and retune.

The noise blanker, notch and JNR do not act on CW.

### 10.2 Transmitting with paddles

Touch either paddle to start. The radio switches to transmit, waits one dit length for the power amplifier to come up, and then sends. The paddle that started the transmission is remembered, so the first element is not lost.

The radio returns to receive 0.8 seconds after the last element (semi break-in).

Set the speed with **CW Speed** and the keyer type with **CW Keyer**.

### 10.3 Transmitting with a straight key

In CWL and CWU the **PTT line is a straight key input**. Connect a straight key, or an external keyer, across it. The transmitter follows the key directly, with the same shaped edges and the same 0.8 second return to receive.

### 10.4 On transmit

- The S-meter goes to full scale while the key is down
- A marker appears at the centre of the spectrum
- The sidetone is heard at the CW Tone pitch and CW Level volume, shaped the same way as the transmitted signal
- The keying is Gaussian shaped, with a rise and fall time of about 5 ms
- SWR and power are displayed
- Turning the encoder sets the power level, as in SSB (section 9.3). This works while keying and during the 0.8 second hang time

### 10.5 CW decoder

Turn on **CW Decode** to read CW on the screen. Try **Schmitt** first on noisy or fading signals. The decoder needs a few characters to lock on to the sending speed. Tune carefully so the `*` flashes cleanly in time with the signal.

---

## 11. Operating the digital modes (DGL and DGU)

DGL and DGU are for digital modes generated by a computer or other device, such as PSK31, RTTY and FT8 from a PC. DGU is the usual choice.

- Feed the receiver audio to the computer.
- Feed the computer's transmit audio to the microphone input.
- **The transmitter is keyed by VOX.** When audio appears at the microphone input the radio transmits. It returns to receive a quarter of a second after the audio stops.
- The PTT and paddles do nothing in these modes.

Settings in these modes are fixed and shown on the status line: 3000 Hz bandwidth on receive and transmit, no noise blanker, no CESSB and no microphone processing. The Mic Gain setting is not applied either, so set the drive with the computer's audio output level and with the encoder while transmitting (section 9.3).

On transmit the S-meter shows the audio input level. Set the computer's output level just high enough to key the VOX reliably and give the power you want.

---

## 12. Receiving AM and shortwave (SWL band)

Select **SWL** from the Band menu. The radio tunes continuously from 3.5 to 30 MHz and, in AUTO mode, selects AM.

- A 5 kHz tuning step suits the broadcast bands.
- JNR can be used to take the edge off noisy signals.
- The band box turns red and TX and RX are both shown in green as a reminder that this band is receive only.
- You can select another mode from the Mode menu, for example USB to listen to utility stations.

**The PTT, paddles and VOX will not transmit on the SWL band or in AM mode.** If you press the PTT or a paddle the radio shows `TX unavaliable` and stays on receive.

FT8 mode can only be entered when the frequency is inside one of the amateur bands. Anywhere else the radio shows `Choose Ham Band`.

AM mode can also be selected on the amateur bands.

---

## 13. Operating FT8

MBPTRX can receive, decode and transmit FT8 by itself. It decodes every 15 second slot, shows the messages, and runs a complete contact for you once you choose a station.

### 13.1 Before you start

1. Make sure your callsign and grid are entered (section 4.2).
2. Select an amateur band and **tune to the FT8 frequency**. The encoder does not tune once you are in FT8 mode. If the frequency is not inside an amateur band the radio shows `Choose Ham Band` and stays out of FT8.
3. Set the power level you want by transmitting briefly in SSB or CW and turning the encoder (section 9.3). It cannot be changed inside FT8 mode. FT8 is a continuous carrier, so the firmware runs it at about a third of the CW drive level.
4. Choose any **FT8 Options** you want (CQ type, Monitor). These cannot be changed inside FT8 mode.
5. Select **Mode** then **FT8**.

Commonly used FT8 frequencies:

| Band | Frequency (MHz) |
|---|---|
| 80 m | 3.573 |
| 40 m | 7.074 |
| 30 m | 10.136 |
| 20 m | 14.074 |
| 17 m | 18.100 |
| 15 m | 21.074 |
| 12 m | 24.915 |
| 10 m | 28.074 |

### 13.2 Timing calibration

FT8 stations all transmit in 15 second slots that start at 0, 15, 30 and 45 seconds past each minute. The radio has no clock, so it has to find out, or be told, where the slots start. This is the calibration. It is kept until the radio is switched off, even if you leave FT8 mode and come back.

**Automatic calibration** is selected at switch-on, so it is used the first time you enter FT8. It needs FT8 signals on the band.

1. The radio shows `Searching 1/5` and listens for one slot.
2. If it decodes any signals it adjusts its timing to match them and shows, for example, `Locked +0.12s`. Calibration is complete.
3. If it decodes nothing it shifts its timing by four seconds and listens again.
4. After five slots with nothing decoded it shows `No signals found`, and carries on searching.

**The radio will not transmit until it has locked.** On a dead band you can leave it searching, move to a band with activity, or use manual calibration.

**Trimming the timing.** If the time offsets (DT) of the stations you decode drift away from zero after a long session, leave FT8, select **FT8 Options** then **Auto Cal** (the radio shows `Auto cal enabled`), and enter FT8 again.

- The radio trims its timing from the first slot that decodes and shows `Locked`. It does not transmit while it is waiting for that slot.
- If nothing decodes in five slots it shows `No signals found` and leaves the timing as it was.

**Manual calibration** can be used on a quiet band. You need an accurate clock.

1. Select **FT8 Options** then **Reset Cal**. The radio shows `Calibration Reset`.
2. Select **Mode** then **FT8**. The screen fills with `Calibrate:` and a running count.
3. Press the button at the exact moment the clock shows 0, 15, 30 or 45 seconds.

If you do not press the button within a minute the radio leaves FT8 mode.

After **Reset Cal** the radio stays on manual calibration. To go back to automatic, select **Auto Cal** before entering FT8.

### 13.3 The FT8 screen

```
+------------------------------------------+
| *     7.074.000                 RX E     |
|                                 1250     |
| ====== signal level ======               |
| +02.5 +0.12  998 CQ VK3ABC QF22          |
| -11.0 +0.31 1503 VK2XYZ ZL1AAA RF82      |
| -06.5 -0.05 2210 W1XYZ JA1BBB PM95       |
|                                          |
| ====== slot progress ======              |
+------------------------------------------+
```

| Item | Meaning |
|---|---|
| `*` | Flashes once a second. Turns red if the station you are working is answering someone else |
| Frequency | The dial frequency. White on receive, red on transmit |
| RX / TX | Green RX while receiving, red TX while transmitting |
| E / O | Whether the current slot is even or odd |
| CQ | Flashes while you are calling CQ |
| Number under RX | Your transmit audio frequency in Hz, shown during a CQ or a contact |
| Signal level bar | Yellow bar showing the received signal level |
| Message list | The twelve most recent decodes, newest at the bottom |
| Slot progress bar | Green while receiving, red while transmitting |
| SWR and power | Shown as two numbers at the top left while transmitting, SWR above power |

Until the first signals are decoded the list reads `Waiting for FT8 signals...`.

Each decoded line reads: signal to noise ratio in dB, time offset (DT) in seconds, audio frequency in Hz, then the message.

Colours in the list:

| Colour | Meaning |
|---|---|
| White on red | A message containing your callsign |
| White on blue | A CQ call |
| White on black | Everything else |

The radio listens for the first 12.6 seconds of each slot and then decodes, which takes a couple of seconds. Signals between 200 and 3000 Hz are decoded, up to 50 messages per slot, and the last 200 messages are kept.

### 13.4 Controls in FT8 mode

| Action | What it does |
|---|---|
| Turn the encoder | Freezes the list and shows a cursor so you can scroll back through the last 200 messages |
| Short press, list not frozen | Starts calling CQ |
| Short press, cursor on a message | Answers that message |
| Long press | Stops the contact or CQ in progress, even in the middle of a transmission |
| Long press, twice within 5 seconds | Leaves FT8 mode |

A long press is one second or more, and acts when you release the button. The radio is busy for a couple of seconds at the end of each slot while it decodes. If a press seems to have been missed, press again.

### 13.5 Browsing the messages

Turn the encoder and the list stops updating so you can read it. The cursor starts on the newest message, and the line under it is shown in black on white.

- Each line gains a prefix of `E` or `O` for the slot it was received in.
- Your position is shown at the top right, for example `47/200`.
- `NEW` flashes when more messages have arrived since the list was frozen.
- Messages more than four slots old are shown in grey. The station may have moved on.
- After ten seconds without input the radio goes back to the live list.

### 13.6 Answering a CQ

1. Turn the encoder to put the cursor on a CQ call.
2. Short press. The screen changes to show the contact.
3. The radio does the rest. It transmits on the other station's audio frequency, in the opposite slot to the one they use.

The contact is shown as six rows. With your callsign as VK2XYZ:

| Row | Direction | Example |
|---|---|---|
| 1 | Heard | `CQ VK3ABC QF22` |
| 2 | T (transmit) | `VK3ABC VK2XYZ QF56` |
| 3 | R (receive) | `VK2XYZ VK3ABC -10` |
| 4 | T | `VK3ABC VK2XYZ R-10` |
| 5 | R | `VK2XYZ VK3ABC RR73` |
| 6 | T | `VK3ABC VK2XYZ 73` |

Rows that have not happened yet show `???` in place of the report, and are filled in as the contact proceeds.

Row colours show progress:

| Colour | Meaning |
|---|---|
| Grey | Not reached yet |
| Black on white | The current step |
| Yellow | Sent, waiting for the reply |
| Pink | No reply yet, second try |
| Red | No reply after three tries |
| Green | Done |

If the other station does not reply, the radio repeats your last message in each of your slots until they do. After three tries the row turns red and `Hold to abort` is shown. The radio keeps trying until you stop it with a long press.

When the contact is complete the radio shows `COOL:` and the other callsign, then returns to the message list.

Notes:

- The radio does not measure the signal report it sends. When you answer a CQ, the report in row 4 is the one the other station gave you, sent back with an R.
- The radio does not keep a log. Write the contact down.

### 13.7 Calling CQ

1. With the live list showing, short press the button. Do this while the green progress bar is moving, so the radio has heard some of the current slot.
2. The radio looks for a clear frequency between 300 and 2500 Hz and starts calling CQ in the next slot. `CQ` flashes at the top of the screen. For CQ types other than plain CQ, a popup confirms what is being sent.
3. The CQ is repeated every second slot until you stop it with a long press.

If the band is too full, or the slot has only just started, the radio shows `No free freq`. Try again a little later.

When someone answers, the radio shows `RESPONSE:` with their callsign. Their message also appears in the list in red. To work them:

1. Turn the encoder to put the cursor on their message.
2. Short press.

The CQ stops and the contact runs automatically. This kind of contact has five rows:

| Row | Direction | Example |
|---|---|---|
| 1 | Heard | `VK2XYZ W1XYZ FN42` |
| 2 | T | `W1XYZ VK2XYZ -07` |
| 3 | R | `VK2XYZ W1XYZ R-09` |
| 4 | T | `W1XYZ VK2XYZ RRR` |
| 5 | R | `VK2XYZ W1XYZ 73` |

Notes:

- The report sent in row 2 is always `-07`.
- If the other station ends the contact early with `RR73`, the radio sends `73` in row 4.
- The contact is treated as complete if the final 73 has not been heard after three slots.
- The CQ does not start again by itself after a contact. Short press to call CQ again.

You can answer anyone who calls you in the same way at any time, whether or not you were calling CQ.

### 13.8 If the other station works someone else

If the station you are calling answers somebody else, the radio shows `WORKING OTHER:` and the `*` turns red. The message is either `Lost to` and the other callsign, or the other callsign and its audio frequency. `CLASH` is added if that station is transmitting within 60 Hz of you.

You can wait, since the radio keeps calling, or stop with a long press.

### 13.9 Selecting a message that cannot be answered

Only CQ calls and messages addressed to you can be answered. For anything else the radio shows `Not CQ/Your Call`.

### 13.10 Monitoring the audio

With **Monitor Off** (the setting at switch-on) FT8 mode is silent. With **Monitor On** you hear the band while receiving. Set this from **FT8 Options** before entering FT8 mode.

### 13.11 Leaving FT8 mode

Long press the button. The radio shows `Hold to exit`. Long press again within five seconds, without turning the encoder in between.

The radio returns to the normal screen with the mode on AUTO. The message history is cleared. The timing calibration is kept.

---

## 14. Hidden features and tips

- **Transmit power.** Turn the encoder while transmitting to set the power level (section 9.3). It works in SSB, CW, DGL and DGU, and the level carries over to FT8.
- **Straight key.** In CW modes the PTT line is a straight key input.
- **Zero beat.** CW Tone sets the sidetone and the receive pitch together, so matching a station to your sidetone puts you on its frequency.
- **Quick step change.** A short press on the button cycles the tuning step through 100 Hz, 1 kHz and 10 kHz without opening the menu.
- **Tuning snaps to the step.** After changing to a larger step, the first click of the encoder moves to the nearest multiple of that step.
- **FT8 frequencies in AUTO.** On 3.573 and 7.074 MHz, AUTO mode selects USB instead of LSB.
- **Band memory.** Each band returns to the frequency you last used on it during the session.
- **Yellow frequency.** A yellow frequency display means the PTT and paddles were locked out at switch-on because one of them was closed.
- **Abandoning an adjustment.** Pressing the PTT or a paddle closes the menu. During a spectrum level adjustment it also puts the level back to what it was.
- **No AGC recovery after transmit.** The AGC level and S-meter reading are saved when you transmit and restored afterwards.
- **Moving the notch.** Select Notch Off, then Notch On, to tune the notch again.
- **Setting digital drive.** In DGL and DGU the S-meter shows the audio input level on transmit.
- **FT8 options first.** CQ type, Monitor, power level and calibration options can only be changed before entering FT8 mode.
- **FT8 calibration persists.** You can leave FT8 to change band and come back without calibrating again.
- **Stopping an FT8 transmission.** A long press stops a contact or CQ at once, even part way through a transmission.
- **Changing your callsign.** Menu, Callsign, ..Reboot.
- **Clock check.** The last number of the version on the splash screen is the measured processor clock in MHz.
- **Processor load.** The percentage box on the main screen shows the signal processing load. It varies with mode and with the filters in use.

---

## 15. Settings that are remembered

These settings are stored when they are changed and restored each time the radio is switched on:

| Setting | Menu |
|---|---|
| Callsign and grid | Callsign |
| Noise reduction level | JNR |
| SSB bandwidth | Bandwidth |
| Spectrum style | Spectrum |
| Spectrum level for each band | Spectrum, Set Level |
| Microphone gain | Mic Gain |
| CESSB on or off | CESSB |
| CW speed | CW Speed |
| CW tone | CW Tone |
| CW level | CW Level |
| Keyer type | CW Keyer |
| SWR display style | Graph SWR |

These always start from the same state at switch-on:

| Setting | State at switch-on |
|---|---|
| Band and frequency | 40 m, 7.100 MHz |
| Mode | AUTO |
| Tuning step | 1 kHz |
| Noise blanker | Off |
| Notch filter | Off, 1000 Hz |
| Microphone processor | Off |
| CW decoder | Off |
| Transmit power level | 100% |
| FT8 CQ type | CQ |
| FT8 calibration method | Automatic |
| FT8 Monitor | Off |
| FT8 timing calibration | None |

The audio is muted for a moment while settings are being stored.

---

## 16. Messages and fault codes

### 16.1 Messages on the main screen

| Heading | Message | Meaning |
|---|---|---|
| `ATTENTION:` | `TX unavaliable` | The PTT or a paddle was pressed on the SWL band or in AM mode |
| `ATTENTION:` | `Notch: SSB only` | The notch was switched in a mode other than LSB or USB |
| `ATTENTION:` | `Graph SWR` or `Numeric SWR` | The SWR display style was changed |
| `CW KEYER:` | `Iambic A` or `Iambic B` | The keyer type was changed |
| `FT8 INFO:` | `Auto cal enabled` | Automatic FT8 calibration was selected |
| `FT8 INFO:` | `Monitor On` or `Monitor Off` | The FT8 monitor setting was changed |
| `FT8 INFO:` | `Calibration Reset` | The FT8 timing calibration was discarded |
| `ERROR:` | `No callsign` | FT8, or Reset Cal, was selected with no callsign and grid stored |
| `FT8 ERROR:` | `Choose Ham Band` | FT8 was selected on a frequency outside the amateur bands |

### 16.2 Messages in FT8 mode

| Heading | Message | Meaning |
|---|---|---|
| `FT8 AUTOCAL:` | `Searching n/5` | Automatic calibration is listening, slot n of 5 |
| `FT8 AUTOCAL:` | `Locked +0.12s` | Automatic calibration succeeded, with the correction applied |
| `FT8 AUTOCAL:` | `No signals found` | Nothing was decoded in five slots. A first calibration carries on searching. A trim leaves the timing as it was |
| `CALLING:` | `CQ DX`, `CQ WWFF`, `CQ POTA`, `CQ SOTA` | The type of CQ being called |
| `INFO:` | `No free freq` | No clear frequency was found for a CQ |
| `RESPONSE:` | A callsign and grid | Someone has answered your CQ |
| `INFO` | `Not CQ/Your Call` | The selected message cannot be answered |
| `QSO MESSAGE:` | `Hold to abort` | No reply after three tries. Long press to give up |
| `CONFIRM EXIT:` | `Hold to exit` | Long press again within five seconds to leave FT8 |
| `COOL:` | A callsign | The contact is complete |
| `WORKING OTHER:` | `Lost to` and a callsign, or a callsign and frequency | The station you are calling is answering someone else |
| `ERROR:` | `Cannot encode` | The reply could not be built. The contact was not started |
| `ERROR:` | `Encode failed` | A message could not be built. The contact was stopped |
| `ERROR:` | `QSO error` | An internal error. The contact was stopped |

### 16.3 Fault codes

If the power-on self-test fails, the radio stops and the LED flashes a code: a group of short flashes, a one second pause, then the group again.

| Flashes | Fault |
|---|---|
| 3 | The processor clock is not 240 MHz. The firmware was built with the wrong settings |
| 4 | The Si5351 oscillator was not found |
| 5 | The low-pass filter control port was not found |
| 6 | The band-pass filter control port was not found |
| 7 | The forward power ADC was not found |
| 8 | The reflected power ADC was not found |

Codes 4 to 8 point to a fault on the I2C bus or with the named part.

### 16.4 Other symptoms

| Symptom | Likely cause |
|---|---|
| Frequency shown in yellow, PTT and paddles do nothing | PTT or a paddle was closed at switch-on (section 4.3) |
| Will not transmit, `TX unavaliable` shown | SWL band or AM mode is selected |
| PTT does nothing | DGL, DGU or FT8 mode is selected |
| Paddles do nothing | The mode is not CWL or CWU |
| Low power in every mode | The transmit power level has been turned down (section 9.3) |
| Encoder will not tune and the menu will not open | The radio is waiting for you to finish a spectrum level or notch adjustment. Press the button |
| CW station on frequency is a little weak | CW Tone is set to 600 or 800 Hz, at the edge of the CW filter (section 8.14) |
| FT8 will not start, `Choose Ham Band` shown | The frequency is outside the amateur bands |
| FT8 will not call CQ or answer | Automatic calibration has not locked yet (section 13.2) |
| No audio in FT8 mode | Monitor is off (section 13.10) |
| FT8 decodes nothing on a busy band | The manual calibration was mistimed. Reset Cal and calibrate again, or select Auto Cal (section 13.2) |
| FT8 stations show a large DT | Timing has drifted. Run Auto Cal again |
| FT8 shows `No callsign` | The callsign was cleared. Switch off and on, and enter it again |
| Notch option shows `Notch: SSB only` | Select LSB or USB first |

---

## 17. Appendix A - Band table

| Band | Tuning range (MHz) | Frequency at switch-on (MHz) |
|---|---|---|
| 80M | 3.500 to 3.800 | 3.600 |
| 40M | 7.000 to 7.300 | 7.100 |
| 30M | 10.100 to 10.150 | 10.120 |
| 20M | 14.000 to 14.350 | 14.200 |
| 17M | 18.068 to 18.168 | 18.100 |
| 15M | 21.000 to 21.450 | 21.200 |
| 12M | 24.890 to 24.990 | 24.900 |
| 10M | 28.000 to 29.700 | 28.500 |
| SWL | 3.500 to 30.000 | 5.000 |

Check the band limits against your own licence conditions before transmitting.

Filters are selected automatically from the frequency:

| Band-pass filter | Used for |
|---|---|
| 3 to 5 MHz | Below 5 MHz |
| 4 to 8 MHz | 5 to 8 MHz |
| 8 to 16 MHz | 8 to 16 MHz |
| 16 to 30 MHz | 16 MHz and above |

| Low-pass filter | Used for |
|---|---|
| LPF 1 | Below 6 MHz |
| LPF 2 | 6 to 11 MHz |
| LPF 3 | 11 to 20 MHz |
| LPF 4 | 20 MHz and above |

---

## 18. Appendix B - Signal connections

Pico 2 pin assignments used by the firmware.

| GPIO | Function |
|---|---|
| 0 | PTT, active low. Straight key in CW modes |
| 1 | Encoder push button, active low |
| 2, 3 | Encoder A and B |
| 4, 5 | I2C SDA and SCL (oscillator, filter control ports, power and SWR ADCs) |
| 6, 7, 8, 9 | Transmit I and Q outputs (0, 180, 90 and 270 degrees) |
| 10, 11, 12, 13 | I and Q audio ADC (data, bit clock, word clock, master clock) |
| 14, 15 | Receiver audio output |
| 16 to 21 | Display |
| 22 | Transmit bias enable |
| 23 | Pico regulator mode |
| 26 | Paddle A (dit), active low |
| 27 | Paddle B (dah), active low |
| 28 | Microphone input |

I2C addresses: low-pass filter control port 0x20, band-pass filter control port 0x21.

---

## 19. Appendix C - Building the firmware

The firmware is built with the Arduino IDE and the Raspberry Pi Pico core, with these settings:

| Setting | Value |
|---|---|
| Board | Raspberry Pi Pico 2 |
| CPU Speed | 240 MHz |
| Optimize | -O3 |
| USB Stack | No USB |
| Flash Size | 4MB (no FS) |

The firmware will not compile if the board, CPU speed, optimisation or USB stack setting is wrong.

Libraries:

- The display uses the TFT_eSPI2 library supplied with the project. Copy `User_Setup.h` into the library folder each time the library is installed.
- In `TFT_eSPI_RP2040.h`, the `spi_set_format(...)` call in the `SET_BUS_READ_MODE` definition must be commented out. See the note at the top of the sketch.
- In `si5351.h`, `SI5351_PLL_VCO_MIN` must be set to 440000000.

Options that can be changed in the source before building:

| Option | Purpose |
|---|---|
| `PEDESTAL_FILTER` | Flattens the noise hump at the centre of the spectrum above 15 MHz. Remove the line to disable it |
| `FT8_TX_MAX` | FT8 transmit drive. 127 is the default, 511 is the maximum |
| `TCXO_FREQ` | The measured frequency of the reference oscillator, for frequency calibration |
