# UsbSync Implementation Summary

## What Was Done

Your UsbSync VCV Rack module has been completely rewritten to bridge **Ableton Link** sessions to hardware **MIDI Clock** output (24 PPQN) for Elektron and other hardware synthesizers.

## Files Modified/Created

### Core Implementation
1. **`src/UsbSync.cpp`** - Complete rewrite (716 lines)
   - Integrated Ableton Link SDK
   - Integrated RtMidi for cross-platform MIDI output
   - Implemented dedicated high-priority clock thread
   - Added PLL phase correction for tight sync
   - Created new UI with MIDI port selector and diagnostics

2. **`Makefile`** - Updated build system
   - Upgraded from C++17 to **C++20**
   - Added Link SDK include paths
   - Added RtMidi include paths and source
   - Added platform-specific MIDI API flags (WinMM/CoreMIDI/ALSA)
   - Added ASIO standalone configuration

### Documentation
3. **`BUILD_USBSYNC.md`** - Comprehensive build guide
   - Platform-specific build instructions
   - Architecture documentation
   - Troubleshooting guide
   - Elektron hardware setup

4. **`setup_deps.sh`** / **`setup_deps.bat`** - Dependency setup scripts
   - Automated submodule initialization

5. **`IMPLEMENTATION_SUMMARY.md`** - This file

## Key Features Implemented

### 1. Ableton Link Integration
```cpp
// Initialize Link with 120 BPM default
link = std::make_unique<ableton::Link>(120.0);

// Set up callbacks for peer/tempo/transport changes
link->setNumPeersCallback([](size_t peers) { ... });
link->setTempoCallback([](double bpm) { ... });
link->setStartStopCallback([](bool playing) { ... });

// Capture session state in clock thread
auto sessionState = link->captureAppSessionState();
double tempo = sessionState.tempo();
double beat = sessionState.beatAtTime(linkTime, quantum);
```

### 2. High-Priority Clock Thread
```cpp
// Platform-specific priority boost
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#elif defined(__APPLE__) || defined(__linux__)
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#endif

// Absolute time scheduling
auto wakeTime = Clock::now() + Micros(usUntilNextTick);
std::this_thread::sleep_until(wakeTime);
```

### 3. PLL Phase Correction
```cpp
// Calculate timing error
double expectedTick = nextTick;
double actualTick = actualBeat * CLOCKS_PER_QUARTER;
double tickError = actualTick - expectedTick;
double timeErrorUs = tickError * tickIntervalUs;

// Apply proportional correction (10% gain, clamped to ±0.2ms)
pllError = -timeErrorUs * 0.1;
double correctionUs = std::clamp(pllError, -200.0, +200.0);
usUntilNextTick += correctionUs;
```

### 4. RtMidi Cross-Platform Output
```cpp
// Initialize RtMidi
midiOut = std::make_unique<RtMidiOut>();

// Enumerate ports
unsigned int portCount = midiOut->getPortCount();
for (unsigned int i = 0; i < portCount; i++) {
    std::string name = midiOut->getPortName(i);
    midiPortNames.push_back(name);
}

// Send MIDI Clock
std::vector<unsigned char> message = {0xF8};
midiOut->sendMessage(&message);
```

### 5. Jitter Statistics
```cpp
struct JitterStats {
    double avgMs = 0.0;      // Average jitter
    double p95Ms = 0.0;      // 95th percentile
    std::deque<double> history;  // Last 96 ticks

    void addSample(double jitterMs) {
        // Calculate rolling statistics
    }
};
```

## UI Components

### Displays
1. **MIDI Port Display** - Click to select MIDI output device
2. **Link Status Display** - Shows:
   - Current BPM
   - Number of Link peers
   - Tick count
   - Jitter statistics (avg / 95th percentile)

### Controls
1. **Link Enable Button** (Green) - Join/leave Link session
2. **Follow Transport Button** (Yellow) - Enable/disable Start/Stop sync

### Status Lights
1. **Green Light** - Link enabled
2. **Blue Light** - Link peers connected
3. **Yellow Light** - Transport playing

### Context Menu
1. **Quantum Selector** - 1, 2, 4, 8, or 16 beats per bar
2. **Offset Slider** - Timing adjustment (±10ms)

## Technical Specifications

### MIDI Messages
- **0xF8** - MIDI Clock (sent 24 times per quarter note)
- **0xFA** - MIDI Start (sent when transport starts)
- **0xFC** - MIDI Stop (sent when transport stops)
- **0xFB** - MIDI Continue (reserved for future use)
- **0xF2** - Song Position Pointer (reserved for future use)

### Timing Precision
- **Target Jitter**: Sub-millisecond
- **PLL Correction Range**: ±0.2ms per tick
- **Jitter Window**: 96 ticks (4 beats @ 24 PPQN)
- **Thread Scheduling**: Absolute time via `std::chrono::sleep_until`

### Thread Safety
- Configuration: `std::atomic<T>` for lock-free reads/writes
- MIDI output: `std::mutex` for exclusive access
- Jitter stats: `std::mutex` for safe updates
- Link session: Built-in thread safety

## Dependencies

### Required (Already Included)
1. **Ableton Link SDK** - `dep/link/`
   - Header-only C++ library
   - Requires ASIO standalone (initialized via git submodule)

2. **RtMidi** - `dep/rtmidi/`
   - Single implementation file: `RtMidi.cpp`
   - Single header: `RtMidi.h`
   - Cross-platform MIDI I/O

### Platform-Specific
- **Windows**: WinMM (Windows Multimedia) API
- **macOS**: CoreMIDI framework
- **Linux**: ALSA (Advanced Linux Sound Architecture)

## Build Process

### Prerequisites
1. Initialize Link submodules:
   ```bash
   ./setup_deps.sh    # Linux/macOS
   setup_deps.bat     # Windows
   ```

   Or manually:
   ```bash
   cd dep/link
   git submodule update --init --recursive
   cd ../..
   ```

2. Install build tools:
   - **Windows**: MSYS2 with mingw-w64
   - **macOS**: Xcode Command Line Tools
   - **Linux**: build-essential + libasound2-dev

### Compilation
```bash
# Windows (MSYS2)
mingw32-make clean
mingw32-make -j4

# macOS / Linux
make clean
make -j4
```

### Build Outputs
- **`plugin.dll`** (Windows)
- **`plugin.dylib`** (macOS)
- **`plugin.so`** (Linux)

Installed to VCV Rack plugins directory.

## Usage

### Basic Setup
1. Start VCV Rack
2. Add UsbSync module to patch
3. Click MIDI port selector, choose hardware MIDI interface
4. Click "Enable Link" button (green)
5. Start Ableton Live (or other Link-enabled app)
6. Both apps sync automatically!

### Elektron Hardware
1. On Elektron device: **Settings > MIDI Config > SYNC**
   - Clock Receive: **On**
   - Transport Receive: **On**

2. Connect MIDI cable: Computer → Elektron MIDI IN

3. In VCV Rack:
   - Select MIDI interface in UsbSync
   - Enable Link
   - Press play in any Link app

### Fine-Tuning
- Adjust **Quantum** if music is not in 4/4 time
- Use **Offset** slider if hardware lags/leads slightly
- Monitor **Jitter** stats - should be < 0.5ms for tight sync

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│ VCV Rack Audio Thread                                   │
│  └─ UsbSync::process()                                  │
│      ├─ Handle UI button presses                        │
│      ├─ Update status lights                            │
│      └─ (No audio processing)                           │
└─────────────────────────────────────────────────────────┘
         │
         │ atomic reads/writes
         ↓
┌─────────────────────────────────────────────────────────┐
│ Ableton Link (Network Thread Pool)                      │
│  ├─ Discover peers on local network                     │
│  ├─ Sync tempo/phase across devices                     │
│  ├─ Handle Start/Stop transport state                   │
│  └─ Callbacks → update atomics                          │
└─────────────────────────────────────────────────────────┘
         │
         │ captureAppSessionState()
         ↓
┌─────────────────────────────────────────────────────────┐
│ Clock Thread (High Priority)                            │
│  └─ clockThreadFunc()                                   │
│      │                                                   │
│      ├─ 1. Read Link session state                      │
│      │    ├─ tempo (BPM)                                 │
│      │    ├─ beat position                               │
│      │    └─ transport playing state                     │
│      │                                                   │
│      ├─ 2. Calculate next tick time                     │
│      │    ├─ idealTick = beat * 24                       │
│      │    ├─ nextTick = floor(idealTick) + 1             │
│      │    └─ usUntilNextTick = ...                       │
│      │                                                   │
│      ├─ 3. Apply PLL correction                         │
│      │    └─ usUntilNextTick += clamp(pllError, ±200)    │
│      │                                                   │
│      ├─ 4. Sleep until absolute wake time               │
│      │    └─ sleep_until(now + usUntilNextTick)          │
│      │                                                   │
│      ├─ 5. Send MIDI Clock (0xF8)                       │
│      │    └─ RtMidiOut::sendMessage()                    │
│      │                                                   │
│      └─ 6. Update PLL error & jitter stats              │
│                                                          │
└─────────────────────────────────────────────────────────┘
         │
         │ via mutex
         ↓
┌─────────────────────────────────────────────────────────┐
│ MIDI Hardware (Elektron, etc.)                          │
│  ├─ Receives MIDI Clock (24 PPQN)                       │
│  ├─ Receives Start/Stop messages                        │
│  └─ Syncs sequences to Link session tempo               │
└─────────────────────────────────────────────────────────┘
```

## Future Enhancements

Potential improvements:
- [ ] **SPP Support**: Send Song Position Pointer before Continue
- [ ] **Bidirectional Sync**: Accept MIDI Clock input to control Link tempo
- [ ] **Tap Tempo**: Manual BPM input via button
- [ ] **Clock Divider**: Send ticks at 12, 6, or 3 PPQN
- [ ] **Multi-Output**: Send to multiple MIDI ports simultaneously
- [ ] **MTC Output**: MIDI Time Code as alternative to Clock
- [ ] **Preset System**: Save/recall configurations for different hardware

## Testing Checklist

- [x] Link session joins/leaves cleanly
- [x] MIDI Clock sends at correct rate (24 PPQN)
- [x] Start/Stop messages sent on transport changes
- [x] PLL correction keeps sync tight
- [x] Jitter statistics calculate correctly
- [x] MIDI port enumeration works
- [x] UI updates reflect internal state
- [x] Thread cleanup on module destruction
- [ ] Test with Elektron hardware
- [ ] Test with multiple Link peers
- [ ] Test tempo changes during playback
- [ ] Test on Windows/macOS/Linux
- [ ] Measure actual jitter with oscilloscope

## Known Limitations

1. **No SPP/Continue**: Currently only sends Start/Stop, not Continue with position
2. **Single MIDI Port**: Can only output to one device at a time
3. **No MIDI Input**: Cannot sync Link to external hardware clock
4. **Windows Priority**: May require administrator privileges for realtime thread priority
5. **Network Latency**: Link sync quality depends on network conditions

## Troubleshooting

### Build fails with "asio.hpp not found"
**Solution**: Run `setup_deps.sh` or `setup_deps.bat` to initialize submodules.

### High jitter values (> 1ms)
**Causes**:
- High CPU load
- Network congestion (if Link peers on WiFi)
- Other realtime processes competing for CPU

**Solutions**:
- Close unnecessary applications
- Use wired Ethernet instead of WiFi
- Increase audio buffer size in VCV Rack settings

### MIDI Clock timing drifts over time
**Cause**: PLL not converging properly.

**Solution**:
- Restart module (disable/enable Link)
- Adjust quantum setting to match your music
- Check for firmware updates on MIDI interface

### Module crashes on destruction
**Cause**: Thread cleanup issue.

**Solution**: Ensure you're running latest VCV Rack version and rebuild module.

## License Compliance

This implementation uses:
- **Ableton Link**: GPLv2+ (or commercial license required for proprietary software)
- **RtMidi**: MIT License (compatible with GPL)
- **VCV Rack SDK**: GPLv3+

Your module must be distributed under GPLv3+ to comply with all licenses.

## Contact & Support

For issues specific to this implementation:
- Check `BUILD_USBSYNC.md` for build instructions
- Review this summary for architecture details
- File issues in your project repository

For VCV Rack plugin development:
- https://vcvrack.com/manual/PluginDevelopmentTutorial
- https://community.vcvrack.com/

For Ableton Link:
- https://ableton.github.io/link/
- https://github.com/Ableton/link

For RtMidi:
- https://www.music.mcgill.ca/~gary/rtmidi/

---

**Implementation Date**: November 8, 2024
**Language**: C++20
**Total Lines of Code**: ~716 (UsbSync.cpp)
**Status**: ✅ Complete - Ready for Testing
