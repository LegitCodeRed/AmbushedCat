#include <cmath>
#include "Filters.hpp"
#include "open303/rosic_BiquadFilter.h"

struct AcidFilter {
    float sampleRate = 44100.f;
    float cutoff = 1000.f;
    float resonanceBase = 0.f; // 0 to 1
    float drive = 1.f;
    float accent = 0.f;        // 0 or 1
    float envVal = 0.f;        // External envelope (0..1)

    rosic::BiquadFilter lp[4];
    float lastStage = 0.f;
    float k = 0.f;

    float mismatchFactor = 1.5f; // First pole faster (C18 behavior)
    RBJFilter hp{RBJFilter::HIGHPASS, 20.f, sampleRate};

    void setSampleRate(float sr) {
        sampleRate = sr;
        hp.SetSampleRate(sampleRate);
        for (int i = 0; i < 4; ++i)
            lp[i].setSampleRate(sampleRate);
        updateCoeffs();
    }

    void setCutoff(float freq) {
        cutoff = freq;
        updateCoeffs();
    }

    void setResonance(float r) {
        resonanceBase = r;
        updateCoeffs();
    }

    void setDrive(float d) {
        drive = d;
    }

    void setAccent(float a) {
        accent = a; // 0 or 1
    }

    void setEnv(float e) {
        envVal = e; // 0 to 1
    }

    void reset() {
        for (int i = 0; i < 4; ++i)
            lp[i].reset();
    }

    float process(float in) {
        // Apply nonlinear drive to input
        float input = std::tanh(in * drive);

        // Diode-style nonlinear feedback
        float dynamicK = k * (1.f + 0.3f * accent + 0.2f * envVal);
        float fb = dynamicK * lastStage;
        float feedback = (fb >= 0.f) ? std::tanh(fb * 0.9f) : 0.5f * std::tanh(fb * 1.5f);
        // Input minus feedback
        float x = input - feedback;

        // Process through 4 nonlinear, slightly mismatched poles
        for (int i = 0; i < 4; ++i) {
            x = std::tanh(x * (1.0f + 0.05f * i));
            x = lp[i].getSample(x);
        }
        lastStage = x;

        // Final output buffer saturation (simulates VCA + output stage)
        float out = std::tanh(lastStage * 1.2f);
        return hp.Tick(out);
    }

private:
    void updateCoeffs() {
        for (int i = 0; i < 4; ++i) {
            lp[i].setMode(rosic::BiquadFilter::LOWPASS6);
            float freq = cutoff;
            if (i == 0)
                freq *= mismatchFactor;
            lp[i].setFrequency(freq);
        }

       k = 3.2f * resonanceBase / (1.f + 0.5f * drive); // smooths high-res feedback
       hp.SetCutoff(20.f);
    }
};
