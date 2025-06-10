#pragma once

float lin_to_log(float lin);

struct Biquad {
    float b0 = 1.f, b1 = 0.f, b2 = 0.f;
    float a1 = 0.f, a2 = 0.f;
    float z1 = 0.f, z2 = 0.f;

    void reset() {
        z1 = z2 = 0.f;
    }

    float process(float in) {
        float out = b0 * in + z1;
        z1 = b1 * in + z2 - a1 * out;
        z2 = b2 * in - a2 * out;
        return out;
    }

    void setLowShelf(float sampleRate, float freq, float gainDb, float slope = 1.f);
    void setHighShelf(float sampleRate, float freq, float gainDb, float slope = 1.f);
};
