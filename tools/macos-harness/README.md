# ozzy-harness (macOS)

Runs Ozzy's engine (`engine/`) against a device from user space on macOS, through IOUSBLib, as a command-line program. No kext, SIP stays on, no signing identity needed. It is a tool for protocol work and for testing an experimental device without installing a driver. It is also the measurement for a userspace macOS backend. The macOS driver itself is the kext in [`macos/`](../../macos/). A port of the [WinUSB harness](../winusb-harness/), and like it, it plays the CoreAudio HAL's part of the shared memory:

- a test tone on outputs 1/2 at -18 dBFS (`-q` for silence)
- the input to a 24-bit WAV file (`-r capture.wav`)
- MIDI in to the console

Every second it prints the device clock measured against `mach_absolute_time`, transfers per second, errors, and the engine's *late* counts (completions after a gap longer than the queued transfers cover: the USB side's xrun). At the end it prints the engine's stats (`engine/shared/ozzy_stats.h`).

## Build

```
make -C tools/macos-harness
```

Needs the Xcode command line tools.

## Use

Unplug nothing, install nothing: the device must just not be claimed by another driver. If Ozzy's kext or a manufacturer's driver is loaded for it, unload that first.

```
tools/macos-harness/ozzy-harness -l                        list supported devices
tools/macos-harness/ozzy-harness -q -s 10 -r in.wav        10 s, silence out, input to in.wav
tools/macos-harness/ozzy-harness -R 48000 -s 30            30 s at 48 kHz with the test tone
```

The test tone goes to the device's outputs 1/2. Turn the monitoring down before running without `-q`.

## ozzyd-client

The same HAL part (tone, input to WAV, `hal_role.c`) against [`ozzyd`](../../macos/Backends/OzzyDaemon/), the userspace backend, instead of an engine in its own process, with the harness's options. Settings reach the daemon as OzzyHAL's would (`OZZY_METHOD_SET_RATE` / `_APPLY_SETTING`), and `-X` asks the daemon for the stall (it takes test requests only with `-T`). The daemon logs the per-second line; the client prints the reader's side and, at the end, what the engine counted while it ran.

```
make -C macos/Backends/OzzyDaemon && macos/Backends/OzzyDaemon/ozzyd-agent.sh load
tools/macos-harness/ozzyd-client -l
tools/macos-harness/ozzyd-client -R 96000 -F 200 -Q 8 -O 8 -s 120 -r in.wav -c 15-16
macos/Backends/OzzyDaemon/ozzyd-agent.sh unload
```

## How it works

`iousb_bus.c` (in `macos/Backends/OzzyDaemon/`, which ozzyd runs on too) fills in `struct ozzy_bus_ops` (`engine/core/ozzy_bus.h`):

- configuration 1 (with a 500 ms wait, as OzzyKext), then the profile's interfaces at its alternate setting, retrying for up to 2 s while their nubs appear
- bulk and interrupt transfers as `ReadPipeAsync` / `WritePipeAsync`. Every completion runs on one thread, under the time-constraint policy, that runs the interfaces' async event sources, so engine callbacks never run concurrently with each other
- control requests as `DeviceRequestTO`, and clear halt as `ClearPipeStallBothEnds`
- the timer on its own real-time thread with absolute deadlines (`mach_wait_until`)

No isochronous transfers yet, so the Ploytec isochronous family (Akai EIE Pro and others) is refused.
