#include <cmath>
#include "dsp.hpp"

float lin_to_log(float lin) {
    const float minLin = 1e-20f; // clip floor to avoid -inf
    float clamped = lin < minLin ? minLin : lin;
    return 20.f * std::log10(clamped);
}

static void calcShelf(float sampleRate, float freq, float gainDb, float slope,
                      bool high, float& b0, float& b1, float& b2,
                      float& a0, float& a1, float& a2) {
    float A = std::pow(10.f, gainDb / 40.f);
    float w0 = 2.f * (float)M_PI * freq / sampleRate;
    float cs = std::cos(w0);
    float sn = std::sin(w0);
    float alpha = sn / 2.f * std::sqrt((A + 1.f / A) * (1.f / slope - 1.f) + 2.f);
    float sqrtA = std::sqrt(A);

    if (high) {
        b0 =    A * ((A + 1.f) + (A - 1.f) * cs + 2.f * sqrtA * alpha);
        b1 = -2.f * A * ((A - 1.f) + (A + 1.f) * cs);
        b2 =    A * ((A + 1.f) + (A - 1.f) * cs - 2.f * sqrtA * alpha);
        a0 =        (A + 1.f) - (A - 1.f) * cs + 2.f * sqrtA * alpha;
        a1 =    2.f * ((A - 1.f) - (A + 1.f) * cs);
        a2 =        (A + 1.f) - (A - 1.f) * cs - 2.f * sqrtA * alpha;
    } else {
        b0 =    A * ((A + 1.f) - (A - 1.f) * cs + 2.f * sqrtA * alpha);
        b1 =  2.f * A * ((A - 1.f) - (A + 1.f) * cs);
        b2 =    A * ((A + 1.f) - (A - 1.f) * cs - 2.f * sqrtA * alpha);
        a0 =        (A + 1.f) + (A - 1.f) * cs + 2.f * sqrtA * alpha;
        a1 =   -2.f * ((A - 1.f) + (A + 1.f) * cs);
        a2 =        (A + 1.f) + (A - 1.f) * cs - 2.f * sqrtA * alpha;
    }
}

void Biquad::setLowShelf(float sampleRate, float freq, float gainDb, float slope) {
    float bb0, bb1, bb2, aa0, aa1, aa2;
    calcShelf(sampleRate, freq, gainDb, slope, false, bb0, bb1, bb2, aa0, aa1, aa2);
    b0 = bb0 / aa0;
    b1 = bb1 / aa0;
    b2 = bb2 / aa0;
    a1 = aa1 / aa0;
    a2 = aa2 / aa0;
}

void Biquad::setHighShelf(float sampleRate, float freq, float gainDb, float slope) {
    float bb0, bb1, bb2, aa0, aa1, aa2;
    calcShelf(sampleRate, freq, gainDb, slope, true, bb0, bb1, bb2, aa0, aa1, aa2);
    b0 = bb0 / aa0;
    b1 = bb1 / aa0;
    b2 = bb2 / aa0;
    a1 = aa1 / aa0;
    a2 = aa2 / aa0;
}
