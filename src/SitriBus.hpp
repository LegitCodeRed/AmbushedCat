#pragma once

#include <cstdint>

namespace SitriBus {

static const int MAGIC = 0x53545249; // 'STRI'

enum GateMode : uint8_t { EXPAND = 0, MUTE = 1, TRIGGER = 2 };

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

        // Current step's output values from Sitri
        float currentPitch = 0.f;  // Current step's pitch CV (V/Oct)
        uint8_t currentGate = 0;   // Current step's gate state (0 or 1)
        uint8_t newNote = 0;       // 1 if this is a new note trigger, 0 if extending previous note
        uint8_t reserved[2] = {};  // Padding for alignment
};

struct ExpanderToMaster {
        int32_t magic = MAGIC;
        uint8_t version = 1;
        uint8_t reserved[3] = {};

        GateMode gateMode[8] = {};
        float stepCV[8] = {};
};

} // namespace SitriBus

