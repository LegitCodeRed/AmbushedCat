# Sitri Algorithm Development Guide

Welcome to the Sitri algorithm development guide! This document will show you how to create and add your own generative algorithms to the Sitri sequencer module.

## Table of Contents
1. [Overview](#overview)
2. [Algorithm Structure](#algorithm-structure)
3. [Creating Your First Algorithm](#creating-your-first-algorithm)
4. [Understanding the AlgoContext](#understanding-the-algocontext)
5. [Understanding StepEvent](#understanding-stepevent)
6. [Registering Your Algorithm](#registering-your-algorithm)
7. [Advanced Features](#advanced-features)
8. [Examples](#examples)

---

## Overview

Sitri is a generative sequencer that uses interchangeable algorithms to create musical patterns. Each algorithm generates notes based on various parameters like density, accent, and randomness.

**Key Features:**
- Deterministic sequences (same seed = same pattern)
- Varied gate lengths per step
- Multiple playback directions
- Quantization support
- Full 10V range (±5V)

---

## Algorithm Structure

Every algorithm must inherit from `IAlgorithm` and implement these methods:

```cpp
struct IAlgorithm {
    virtual ~IAlgorithm() = default;
    virtual const char* id() const = 0;              // Unique ID (e.g., "myalgo")
    virtual const char* displayName() const = 0;     // Display name (e.g., "My Algorithm")
    virtual int paramCount() const { return 0; }     // Optional: number of parameters
    virtual const char* paramName(int) const { return ""; }  // Optional: parameter names
    virtual void setParam(int, float) {}             // Optional: set parameter values
    virtual void reset(uint64_t) {}                  // Optional: reset algorithm state
    virtual StepEvent generate(const AlgoContext& ctx) = 0;  // Generate next step
};
```

---

## Creating Your First Algorithm

Here's a simple example algorithm that creates a bouncing ball pattern:

```cpp
struct AlgoBounce : public IAlgorithm {
    const char* id() const override { return "bounce"; }
    const char* displayName() const override { return "BOUNCE"; }

    StepEvent generate(const AlgoContext& ctx) override {
        StepEvent e;

        // Active based on density
        e.active = rand01(ctx.prngState) < ctx.density;

        // Create a bouncing pattern using sine wave
        float phase = (float)ctx.stepIndex / (float)ctx.steps;
        float bounce = std::abs(std::sin(phase * 3.14159f * 2.0f));

        // Pitch bounces between -2V and +2V
        e.pitch = bounce * 4.0f - 2.0f;

        // Velocity follows the bounce
        e.vel = 0.5f + 0.5f * bounce;

        // Varied gate lengths
        float gateRoll = rand01(ctx.prngState);
        if (gateRoll < 0.3f) {
            e.gateFrac = 0.2f + 0.2f * rand01(ctx.prngState); // Short
        } else if (gateRoll < 0.7f) {
            e.gateFrac = 0.45f + 0.25f * rand01(ctx.prngState); // Medium
        } else {
            e.gateFrac = 0.7f + 0.3f * rand01(ctx.prngState); // Long
        }

        return e;
    }
};

// Register the algorithm (see below)
REGISTER_ALGO("bounce", AlgoBounce);
```

---

## Understanding the AlgoContext

The `AlgoContext` provides information about the current step:

```cpp
struct AlgoContext {
    int stepIndex = 0;           // Current step (0 to steps-1)
    int steps = 16;              // Total number of steps in sequence
    float density = 0.5f;        // Note density (0.0 to 1.0)
    float accent = 0.5f;         // Accent amount (0.0 to 1.0)
    mutable uint64_t prngState = 1;  // Random number generator state
    float lastPitch = 0.f;       // Previous step's pitch
    float lastVel = 0.8f;        // Previous step's velocity
    float phase01 = 0.f;         // Phase within step (0.0 to 1.0)
    float divHz = 1.f;           // Clock division frequency
};
```

**Important:**
- Use `ctx.prngState` for all random number generation to maintain determinism
- `stepIndex` wraps around (0 to steps-1)
- Modify `ctx.prngState` using the provided random functions

---

## Understanding StepEvent

Your algorithm returns a `StepEvent` describing what should happen on this step:

```cpp
struct StepEvent {
    bool active = true;          // Should this step play a note?
    float pitch = 0.f;           // Pitch in V/Oct (-5V to +5V for full range)
    float prob = 1.f;            // Probability multiplier (0.0 to 1.0)
    float vel = 0.8f;            // Velocity (0.0 to 1.0)
    float gateFrac = 0.5f;       // Gate length as fraction of step (0.0 to 1.0)
};
```

**Field Details:**

- **active**: Set to `false` to skip this step entirely
- **pitch**: V/Oct pitch value
  - 0V = C4
  - ±5V gives full 10-octave range
  - Will be quantized to the selected scale
- **prob**: Additional probability (multiplied with density)
- **vel**: Note dynamics (0-1), scaled to 0-10V output
- **gateFrac**: Gate length
  - 0.1 = very short staccato
  - 0.5 = half the step duration
  - 1.0 = full step duration (legato)
  - **Vary this per step for musical interest!**

---

## Registering Your Algorithm

After defining your algorithm class, register it using the `REGISTER_ALGO` macro:

```cpp
REGISTER_ALGO("bounce", AlgoBounce);
```

**Parameters:**
- First parameter: unique ID string (lowercase, no spaces)
- Second parameter: your algorithm class name

**That's it!** Your algorithm will automatically appear in Sitri's algorithm selector.

---

## Advanced Features

### Using the PRNG

Always use the provided random functions with `ctx.prngState`:

```cpp
// Random float between 0.0 and 1.0
float r = rand01(ctx.prngState);

// Random integer from 0 to maxExclusive-1
int i = randRange(ctx.prngState, 10);  // 0-9

// Random choice from list
int note = randChoice({0, 2, 4, 7, 9}, ctx.prngState);  // Pentatonic scale
```

### Stateful Algorithms

If your algorithm needs to maintain state between steps, use member variables:

```cpp
struct AlgoWalker : public IAlgorithm {
    int position = 0;  // State variable

    const char* id() const override { return "walker"; }
    const char* displayName() const override { return "WALKER"; }

    void reset(uint64_t seed) override {
        position = 0;  // Reset state on sequence reset
    }

    StepEvent generate(const AlgoContext& ctx) override {
        StepEvent e;
        e.active = rand01(ctx.prngState) < ctx.density;

        // Random walk
        position += randChoice({-1, 0, 1}, ctx.prngState);
        position = clamp(position, -60, 60);  // ±5 octaves

        e.pitch = degToVolts(position);
        e.vel = 0.7f;
        e.gateFrac = 0.5f;

        return e;
    }
};
REGISTER_ALGO("walker", AlgoWalker);
```

### Using Euclidean Rhythms

Helper function for creating euclidean patterns:

```cpp
bool euclidHit(int step, int steps, int pulses);

// Example: 5 hits in 8 steps
bool hit = euclidHit(ctx.stepIndex, 8, 5);
```

### Degree to Volts Conversion

Convert semitone degrees to V/Oct:

```cpp
float degToVolts(int deg);

// Example: major chord
int degree = randChoice({0, 4, 7}, ctx.prngState);  // Root, M3, P5
e.pitch = degToVolts(degree);
```

---

## Examples

### Example 1: Simple Random Algorithm

```cpp
struct AlgoSimpleRandom : public IAlgorithm {
    const char* id() const override { return "simprnd"; }
    const char* displayName() const override { return "Simple Random"; }

    StepEvent generate(const AlgoContext& ctx) override {
        StepEvent e;
        e.active = rand01(ctx.prngState) < ctx.density;
        e.pitch = (rand01(ctx.prngState) - 0.5f) * 4.0f;  // ±2V range
        e.vel = 0.5f + 0.5f * rand01(ctx.prngState);
        e.gateFrac = 0.3f + 0.4f * rand01(ctx.prngState);
        return e;
    }
};
REGISTER_ALGO("simprnd", AlgoSimpleRandom);
```

### Example 2: Pattern-Based Algorithm

```cpp
struct AlgoPattern : public IAlgorithm {
    const char* id() const override { return "pattern"; }
    const char* displayName() const override { return "PATTERN"; }

    StepEvent generate(const AlgoContext& ctx) override {
        static constexpr std::array<int, 8> pattern = {0, 3, 5, 7, 5, 3, 0, -5};

        StepEvent e;
        e.active = rand01(ctx.prngState) < (0.7f + 0.3f * ctx.density);

        int idx = ctx.stepIndex % 8;
        e.pitch = degToVolts(pattern[idx]);

        // Accent every 4th step
        bool accent = (ctx.stepIndex % 4) == 0;
        e.vel = accent ? (0.8f + 0.2f * ctx.accent) : 0.6f;

        // Varied gates
        e.gateFrac = accent ? 0.7f : 0.4f;

        return e;
    }
};
REGISTER_ALGO("pattern", AlgoPattern);
```

### Example 3: Density-Responsive Algorithm

```cpp
struct AlgoDensity : public IAlgorithm {
    const char* id() const override { return "densalg"; }
    const char* displayName() const override { return "DENSITY"; }

    StepEvent generate(const AlgoContext& ctx) override {
        StepEvent e;

        // More density = more active steps
        e.active = rand01(ctx.prngState) < ctx.density;

        // Higher density = higher notes
        float densityInfluence = ctx.density * 2.0f - 1.0f;  // -1 to +1
        e.pitch = densityInfluence + (rand01(ctx.prngState) - 0.5f) * 0.5f;

        // Higher density = more velocity variation
        e.vel = 0.5f + ctx.density * 0.5f * rand01(ctx.prngState);

        // Varied gates based on accent
        float gateRoll = rand01(ctx.prngState);
        if (gateRoll < ctx.accent) {
            e.gateFrac = 0.7f + 0.3f * rand01(ctx.prngState);  // Long
        } else {
            e.gateFrac = 0.2f + 0.3f * rand01(ctx.prngState);  // Short
        }

        return e;
    }
};
REGISTER_ALGO("densalg", AlgoDensity);
```

---

## Tips for Great Algorithms

1. **Vary Gate Lengths**: Don't use a fixed gate length! Create musical rhythm by varying gateFrac
2. **Use Density**: Respect the density parameter for note triggering
3. **Use Accent**: Use accent to add dynamics and variation
4. **Stay Deterministic**: Always use ctx.prngState for randomness
5. **Test Edge Cases**: Test with steps=1, steps=64, density=0, density=1
6. **Musical Range**: Keep pitch variations musical (don't jump too wildly unless intentional)
7. **Document Your Algorithm**: Add comments explaining the musical concept

---

## File Location

Add your algorithm to: `src/Sitri.cpp`

Place your algorithm code in the "Algorithms" section (around line 160), before the quantizer code.

---

## Contributing

When contributing your algorithm:

1. Test thoroughly with different step counts, densities, and directions
2. Use descriptive names for your algorithm
3. Add comments explaining the musical concept
4. Follow the existing code style
5. Make sure your algorithm compiles without warnings

---

## Questions or Help?

If you need help developing your algorithm or have questions about the API, please open an issue on the GitHub repository!

Happy algorithm creating! 🎵
