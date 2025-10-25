#pragma once

#include <cstdint>

namespace BuerBus {

static const int32_t MAGIC = 0x42555245; // 'BURE'

struct ToLilith {
        int32_t magic = MAGIC;
        uint8_t version = 1;
        uint8_t connected = 0;
        uint8_t reserved[2] = {};
        float cvMod[16] = {};
        float modeMod[16] = {};
};

struct FromLilith {
        int32_t magic = MAGIC;
        uint8_t version = 1;
        uint8_t numSteps = 0;
        uint8_t activeSteps = 0;
        uint8_t reserved = 0;
        float baseCv[16] = {};
        float baseMode[16] = {};
};

} // namespace BuerBus

