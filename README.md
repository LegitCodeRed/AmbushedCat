# AmbushedCat modules for VCV Rack 2
Heavy hitting with love <3

Dependencies:
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [NeuralAmpModelerCore](https://github.com/sdatkinson/NeuralAmpModelerCore)
- [SabnockOTT/vitOTT](https://github.com/LegitCodeRed/SabnockOTT)

## UsbSync quick start

1. Create or enable a virtual/USB MIDI port (IAC on macOS, loopMIDI on Windows, a2jmidid on Linux).
2. In your DAW (Ableton, Logic, Bitwig, etc.), enable **Sync** on that MIDI output.
3. Add **UsbSync** in Rack, open the MIDI IN dropdown, and choose the same port.
4. Press play in the DAW. Rack clock, run, and reset outputs now follow the transport.
5. Tweak **SMOOTH** to taste for visual stability, **OFFSET** to nudge timing, and enable **Follow SPP** if you want Rack to chase DAW playhead jumps.
