#pragma once
#include <cmath>

// Simple diode ladder lowpass filter approximation
// Provides a resonant 4-pole sound inspired by the TB-303
struct DiodeLadder {
    float sampleRate = 44100.f;
    float cutoff = 1000.f;
    float resonance = 0.f;

    float stage[4] = {};
    float g = 0.f;
    float k = 0.f;

    void setSampleRate(float sr) {
        sampleRate = sr;
        updateCoeffs();
    }

    void setCutoff(float freq) {
        cutoff = freq;
        updateCoeffs();
    }

    void setResonance(float r) {
        resonance = r;
        updateCoeffs();
    }

    void reset() {
        for (int i = 0; i < 4; ++i) {
            stage[i] = 0.f;
        }
    }

    float process(float in) {
        float feedback = k * stage[3];
        float x = std::tanh(in - feedback);
        for (int i = 0; i < 4; ++i) {
            stage[i] += g * (x - stage[i]);
            x = std::tanh(stage[i]);
        }
        return stage[3];
    }

private:
    void updateCoeffs() {
        float wc = 2.f * (float)M_PI * cutoff;
        g = wc / sampleRate;
        g = g / (1.f + g); // bilinear transform approximation
        k = 4.f * resonance; // resonance scaling
    }
};
