#include <cmath>
#include "Filters.hpp"

struct AcidFilter {
    float sampleRate = 44100.f;
    float cutoff = 1000.f;
    float resonanceBase = 0.f; // 0 to 1
    float drive = 1.f;
    float accent = 0.f;        // 0 or 1
    float envVal = 0.f;        // External envelope (0..1)

    float stage[4] = {};
    float g[4] = {};
    float k = 0.f;

    float mismatchFactor = 1.5f; // First pole faster (C18 behavior)
    RBJFilter hp{RBJFilter::HIGHPASS, 20.f, sampleRate};

    void setSampleRate(float sr) {
        sampleRate = sr;
        hp.SetSampleRate(sampleRate);
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
            stage[i] = 0.f;
    }

    float process(float in) {
        // Apply nonlinear drive to input
        float input = std::tanh(in * drive);

        // Diode-style nonlinear feedback
        float fb = k * stage[3];
        float feedback = (fb >= 0.f) ? std::tanh(fb * 0.9f) : 0.5f * std::tanh(fb * 1.5f);
        // Input minus feedback
        float x = input - feedback;

        // Process through 4 nonlinear, slightly mismatched poles
        for (int i = 0; i < 4; ++i) {
            float gain = g[i] * (1.f + 0.03f * i); // subtle mismatch
            x = stage[i] + gain * (std::tanh(x * (1.0f + 0.05f * i)) - stage[i]);
            stage[i] = x;
        }

        // Final output buffer saturation (simulates VCA + output stage)
        float out = std::tanh(stage[3] * 1.2f);
        return hp.Tick(out);
    }

private:
    void updateCoeffs() {
        float T = 1.f / sampleRate;
        float wc = 2.f * (float)M_PI * cutoff;
        float wd = 2.f / T * std::tan(wc * T / 2.f);
        float baseG = wd * T / (1.f + wd * T);

        g[0] = baseG * mismatchFactor; // first pole mismatch = C18
        g[1] = baseG;
        g[2] = baseG;
       g[3] = baseG;

       k = 3.2f * resonanceBase / (1.f + 0.5f * drive); // smooths high-res feedback
       hp.SetCutoff(20.f);
    }
};