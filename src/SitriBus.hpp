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
};

struct ExpanderToMaster {
        int32_t magic = MAGIC;
        uint8_t version = 1;
        uint8_t reserved[3] = {};

        GateMode gateMode[8] = {};
        float stepCV[8] = {};
};

} // namespace SitriBus

