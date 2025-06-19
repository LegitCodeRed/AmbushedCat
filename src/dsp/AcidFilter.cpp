#pragma once
#include <cmath>

// 3-pole diode ladder style low-pass filter
// with non-linear feedback for classic 303 tone
struct AcidFilter {
    float sampleRate = 44100.f;
    float cutoff = 1000.f;
    float resonance = 0.f; // 0..1
    float drive = 0.f;

    float stage[3] = {};
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

    void setDrive(float d) {
        drive = d;
    }

    void reset() {
        for (int i = 0; i < 3; ++i)
            stage[i] = 0.f;
    }

    float process(float in) {
        float input = std::tanh(in * (1.f + drive));
        float feedback = std::tanh(k * stage[2]);
        float x = input - feedback;
        for (int i = 0; i < 3; ++i) {
            stage[i] += g * (x - stage[i]);
            x = std::tanh(stage[i]);
        }
        return stage[2];
    }

private:
    void updateCoeffs() {
        float wc = 2.f * (float)M_PI * cutoff;
        g = wc / sampleRate;
        g = g / (1.f + g); // bilinear transform approx
        // scale resonance below self-oscillation
        k = 2.5f * resonance;
    }
};
