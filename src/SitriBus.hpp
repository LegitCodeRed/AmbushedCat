#pragma once

#include <cstdint>

namespace SitriBus {

static const int MAGIC = 0x53545249; // 'STRI'

enum GateMode : uint8_t { EXPAND = 0, MUTE = 1, TRIGGER = 2 };

struct StepData {
        float pitch = 0.f;
        uint8_t gate = 0;
        uint8_t newNote = 0;
        uint8_t valid = 0;  // 1 if this step has been written by Sitri, 0 if empty slot
        uint8_t reserved = 0;  // Padding for alignment
};

struct MasterToExpander {
        int32_t magic = MAGIC;
        uint8_t version = 1;
        uint8_t running = 0;
        uint8_t reservedA = 0;
        uint8_t reservedB = 0;

        uint8_t stepIndex = 1; // 1-based
        uint8_t numSteps = 1;
        uint8_t resetEdge = 0;
        uint8_t clockEdge = 0;
        uint8_t eocPulse = 0;      // End of cycle pulse - triggers snapshot capture
        uint8_t reseedEdge = 0;    // Reseed button pressed - triggers recapture
        uint8_t stepsAdvanced = 1; // Number of steps that advanced this frame (for high speed capture)
        uint8_t reserved2 = 0;

        // Current step's output values from Sitri (for compatibility)
        float currentPitch = 0.f;  // Current step's pitch CV (V/Oct)
        uint8_t currentGate = 0;   // Current step's gate state (0 or 1)
        uint8_t newNote = 0;       // 1 if this is a new note trigger, 0 if extending previous note
        uint8_t reserved = 0;      // Padding for alignment

        // Global parameters from Sitri
        float gateLength = 0.5f;   // Gate length parameter (0.05-1.0 = 5%-100%)

        // Step history buffer - last 8 steps that advanced (for high speed capture)
        StepData stepHistory[8] = {};
};

struct ExpanderToMaster {
        int32_t magic = MAGIC;
        uint8_t version = 1;
        uint8_t reserved[3] = {};

        GateMode gateMode[8] = {};
        float stepCV[8] = {};
};

} // namespace SitriBus

