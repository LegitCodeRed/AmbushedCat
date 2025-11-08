# Building UsbSync Module - Link to MIDI Clock Bridge

## Overview

UsbSync has been completely rewritten to bridge Ableton Link sessions to hardware MIDI Clock (24 PPQN) using:

- **Ableton Link SDK** (C++) - join Link sessions, read tempo/phase, handle Start/Stop
- **RtMidi** - cross-platform MIDI output (WinMM/Windows, CoreMIDI/macOS, ALSA/Linux)
- **C++20** standard
- **Dedicated high-priority clock thread** with absolute scheduling and PLL phase correction

## Features

### Core Functionality
- Joins Ableton Link session and outputs tight MIDI Clock (24 PPQN)
- MIDI Start/Stop/Continue messages with optional SPP (Song Position Pointer)
- High-precision clock thread using `std::chrono::steady_clock` with `sleep_until`
- PLL (Phase-Locked Loop) correction toward Link phase (±0.2ms max per tick)
- Platform-specific thread priority boost:
  - Windows: `SetThreadPriority` (THREAD_PRIORITY_TIME_CRITICAL)
  - macOS: `pthread_setschedparam` with SCHED_FIFO
  - Linux: `pthread_setschedparam` with SCHED_FIFO

### User Controls
- **Enable Link**: Join/leave Link session
- **Follow Transport**: Enable/disable Start/Stop sync
- **Quantum**: Bar length (1, 2, 4, 8, or 16 beats)
- **MIDI Port Selector**: Choose MIDI output device
- **Offset**: Adjustable timing offset (±10ms)

### Diagnostics Display
- Current BPM from Link session
- Number of connected Link peers
- Tick count since start
- Jitter statistics (average and 95th percentile over last 96 ticks)

## Build Requirements

### Windows
1. Install MSYS2 (https://www.msys2.org/)
2. Open MSYS2 MinGW 64-bit terminal
3. Install build tools:
   ```bash
   pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-make
   ```
4. Add to PATH: `C:\msys64\mingw64\bin`

### macOS
1. Install Xcode Command Line Tools:
   ```bash
   xcode-select --install
   ```

### Linux
1. Install build essentials:
   ```bash
   sudo apt-get install build-essential libasound2-dev
   ```

## Building

### IMPORTANT: Initialize Submodules First

The Link library requires the ASIO standalone headers. Before building, run:

```bash
cd dep/link
git submodule update --init --recursive
cd ../..
```

This will download the ASIO standalone library that Link depends on.

### Build Commands

From the `TuringMaschine` directory:

#### Windows (MSYS2)
```bash
mingw32-make clean
mingw32-make -j4
```

#### macOS / Linux
```bash
make clean
make -j4
```

## Dependencies (Already Included)

The following libraries are already in the `dep/` directory:

- **Ableton Link**: `dep/link/`
  - Header-only library
  - Includes ASIO standalone for network I/O

- **RtMidi**: `dep/rtmidi/`
  - Single source file: `RtMidi.cpp`
  - Single header: `RtMidi.h`

## Compilation Flags

### C++ Standard
- Uses **C++20** (upgraded from C++17)

### Platform-Specific MIDI API Defines
- **Windows**:
  - MIDI: `-D__WINDOWS_MM__` + link `-lwinmm`
  - Link: `-DLINK_PLATFORM_WINDOWS` + link `-lws2_32 -liphlpapi` (for networking)
- **macOS**:
  - MIDI: `-D__MACOSX_CORE__` + frameworks: `-framework CoreMIDI -framework CoreAudio -framework CoreFoundation`
  - Link: `-DLINK_PLATFORM_MACOSX`
- **Linux**:
  - MIDI: `-D__LINUX_ALSA__` + link `-lasound`
  - Link: `-DLINK_PLATFORM_LINUX`

### Additional Flags
- Link SDK headers: `-Idep/link/include`
- ASIO standalone headers: `-Idep/link/modules/asio-standalone/asio/include`
- RtMidi headers: `-Idep/rtmidi`
- **Note**: ASIO standalone is configured automatically by Link's `AsioWrapper.hpp`

## Architecture

### Thread Model
```
Main Audio Thread (VCV Rack)
  │
  ├─ process(): Handle UI buttons, update lights
  │
  └─ Link instance: Manage network discovery/sync

Clock Thread (High Priority)
  │
  ├─ Read Link session state (tempo, phase, transport)
  ├─ Calculate next tick time using Link timeline
  ├─ Apply PLL correction (±0.2ms max)
  ├─ Sleep until absolute wake time (std::chrono::sleep_until)
  ├─ Send MIDI Clock (0xF8)
  └─ Update jitter statistics
```

### Clock Scheduling Algorithm

```cpp
// Every tick:
interval = 60.0 / (tempo * 24.0)  // seconds per tick

// Read Link timeline
phase = sessionState.phaseAtTime(linkTime, quantum)
beat = sessionState.beatAtTime(linkTime, quantum)

// Calculate next tick
idealTick = beat * 24
nextTick = floor(idealTick) + 1
beatsUntilNextTick = (nextTick / 24) - beat
usUntilNextTick = beatsUntilNextTick * (60e6 / tempo)

// Apply PLL correction (proportional control)
usUntilNextTick += clamp(pllError, -200, +200)

// Sleep until absolute time
sleep_until(now + usUntilNextTick)

// Send MIDI Clock
sendMessage(0xF8)

// Update PLL error for next iteration
actualBeat = sessionState.beatAtTime(actualTime, quantum)
tickError = (actualBeat * 24) - nextTick
pllError = -tickError * tickInterval * 0.1  // 10% proportional gain
```

## Elektron Hardware Setup

On your Elektron device (Digitakt, Digitone, Octatrack, etc.):

1. **Settings > MIDI Config > SYNC**
   - Clock Receive: **On**
   - Transport Receive: **On**

2. Connect MIDI cable from computer to Elektron MIDI IN

3. In VCV Rack:
   - Click MIDI port selector
   - Choose your MIDI interface
   - Enable Link
   - Start playing in Ableton Link session

## Troubleshooting

### Build Error: "asio.hpp: No such file or directory"

**Problem:** The Link library depends on ASIO standalone which is a git submodule.

**Solution:**
```bash
cd dep/link
git submodule update --init --recursive
cd ../..
```

Then rebuild:
```bash
mingw32-make clean  # or 'make clean' on macOS/Linux
mingw32-make -j4    # or 'make -j4' on macOS/Linux
```

### Build Error: "'class ableton::Link' has no member named 'enable'"

**Problem:** Link requires platform-specific macros to be defined.

**Solution:** The Makefile now automatically defines the correct platform macro:
- Windows: `-DLINK_PLATFORM_WINDOWS`
- macOS: `-DLINK_PLATFORM_MACOSX`
- Linux: `-DLINK_PLATFORM_LINUX`

If you still see this error, ensure you're building with the updated Makefile. Run:
```bash
mingw32-make clean  # or 'make clean' on macOS/Linux
mingw32-make -j4    # or 'make -j4' on macOS/Linux
```

### No MIDI devices shown
- Ensure MIDI interface is connected
- Restart VCV Rack
- On Linux, ensure user is in `audio` group: `sudo usermod -a -G audio $USER`

### High jitter values
- Check system load (CPU usage)
- Close unnecessary applications
- Ensure Link peers are on stable network
- Try adjusting offset value (±10ms range)

### Link not connecting to peers
- Ensure all devices on same network/VLAN
- Check firewall allows UDP port 20808
- Verify Link is enabled in all applications

### Build errors
- Ensure C++20 compiler support:
  - GCC 10+ or Clang 10+
  - MSVC 2019 16.11+ (Windows)
- Check all dependencies are present in `dep/` directory
- Verify RACK_DIR environment variable points to Rack SDK

## License

This module uses:
- **Ableton Link**: GPL v2+ (or proprietary license for commercial use)
- **RtMidi**: MIT License
- **VCV Rack SDK**: GPL v3+

Ensure compliance with all licenses when distributing.

## Technical Details

### MIDI Clock Specification
- **MIDI Clock (0xF8)**: Sent 24 times per quarter note (24 PPQN)
- **MIDI Start (0xFA)**: Sent when transport starts from beginning
- **MIDI Continue (0xFB)**: Sent when transport resumes (optional, reserved for future use)
- **MIDI Stop (0xFC)**: Sent when transport stops
- **Song Position Pointer (0xF2)**: Optional, sent before Continue (reserved for future use)

### Timing Precision
- Target: Sub-millisecond jitter
- PLL correction range: ±0.2ms per tick
- Jitter monitoring: Rolling 96-tick window (4 beats @ 24 PPQN)
- Thread scheduling: Absolute time using `std::chrono::sleep_until`

### Thread Safety
- All configuration atomics: `std::atomic<T>`
- MIDI output: Protected by `std::mutex`
- Jitter stats: Protected by `std::mutex`
- Link session state: Captured per-frame (thread-safe by design)

## Future Enhancements

Possible improvements:
- [ ] SPP (Song Position Pointer) support for Continue messages
- [ ] MIDI Clock input mode (sync Link to external hardware)
- [ ] Tap tempo input
- [ ] MIDI Clock output filtering (e.g., only send every Nth tick)
- [ ] Multiple MIDI output ports simultaneously
- [ ] MIDI Time Code (MTC) output option
- [ ] Save/recall presets for different hardware setups
