#pragma once

// Simple transformer emulation for P44 Magnum style circuit

struct P42Circuit {
    float hpState = 0.f;
    float preEQState = 0.f;
    float postEQState = 0.f;
    float lpState = 0.f;

    float process(float in, float drive, float sampleRate) {
        // === Parameters ===
        const float hpCut = 20.f;
        const float lpCut = 14000.f;
        const float preEQCut = 800.f;      // gentle bump before saturation
        const float postEQCut = 6000.f;    // rolloff after saturation
        const float bias = 0.05f;          // bias for asymmetric saturation

        // === High-pass filter (DC Blocker) ===
        float hpA = std::exp(-2.f * M_PI * hpCut / sampleRate);
        hpState = hpA * hpState + (1.f - hpA) * in;
        float hp = in - hpState;

        // === Pre-EQ bump (resonant shelf) ===
        float preA = std::exp(-2.f * M_PI * preEQCut / sampleRate);
        preEQState = preA * preEQState + (1.f - preA) * hp;
        float preBoosted = hp + 0.3f * (hp - preEQState);  // subtle mid-bump

        // === Saturation ===
        float driven = (preBoosted + bias) * drive;
        float sat = std::tanh(driven * 1.4f);  // tanh distortion
        float mixed = 0.6f * sat + 0.4f * driven;

        // === Post-EQ soft lowpass ===
        float postA = std::exp(-2.f * M_PI * postEQCut / sampleRate);
        postEQState = postA * postEQState + (1.f - postA) * mixed;

        // === Mild transformer-style rolloff ===
        float lpA = std::exp(-2.f * M_PI * lpCut / sampleRate);
        lpState = lpA * lpState + (1.f - lpA) * postEQState;

        return lpState;
    }
};

