#pragma once

// Simple transformer emulation for P44 Magnum style circuit

struct P42Circuit {
 float hpState = 0.f, preEQState = 0.f, postEQState = 0.f, lpState = 0.f;
    float fluxMemory = 0.f;
    float slewState = 0.f;
    float prevIn = 0.f;

    float process(float in, float drive, float sampleRate) {
        // === Tuned Parameters (P44 Magnum style) ===
        const float hpCut = 18.f;           // gentle DC rolloff
        const float preEQCut = 720.f;       // mid boost before saturation
        const float postEQCut = 5200.f;     // mellow top-end post-sat
        const float lpCut = 13000.f;        // final transformer rolloff
        const float bias = 0.045f;          // asymmetry
        const float resonanceGain = 0.03f;  // subtle transformer resonance
        const float slewSpeed = 0.75f;      // transient rounding
        const float compThresh = 0.9f;      // soft compression knee

        // === High-pass filter (DC blocking) ===
        float hpA = std::exp(-2.f * M_PI * hpCut / sampleRate);
        hpState = hpA * hpState + (1.f - hpA) * in;
        float hp = in - hpState;

        // === Pre-EQ bump (boost mids) ===
        float preA = std::exp(-2.f * M_PI * preEQCut / sampleRate);
        preEQState = preA * preEQState + (1.f - preA) * hp;
        float midBoost = hp + 0.45f * (hp - preEQState);  // stronger push than before

        // === Flux Memory (soft hysteresis) ===
        fluxMemory = 0.994f * fluxMemory + 0.006f * midBoost;
        float fluxShape = 0.5f * std::tanh(fluxMemory);

        // === Saturation Core ===
        float driven = (midBoost + bias + fluxShape) * drive;

        // Multi-shaper harmonic enrichment
        float harmonics =
            0.55f * std::tanh(1.3f * driven) +
            0.25f * std::tanh(0.5f * driven * driven) +
            0.15f * std::sin(driven * 0.45f) +
            0.05f * std::tanh(drive * (driven - std::sin(driven)));  // asymmetric flavor
        float shaped = 0.6f * harmonics + 0.4f * driven;

        // === Soft Compression / Limiting ===
        float compressed = std::tanh(compThresh * shaped);

        // === Slew Limiting (transient rounding) ===
        slewState += (compressed - slewState) * slewSpeed;
        float slewed = slewState;

        // === Post-EQ filter (gentle lowpass) ===
        float postA = std::exp(-2.f * M_PI * postEQCut / sampleRate);
        postEQState = postA * postEQState + (1.f - postA) * slewed;

        // === Final LP + Resonance ===
        float lpA = std::exp(-2.f * M_PI * lpCut / sampleRate);
        lpState = lpA * lpState + (1.f - lpA) * postEQState;

        float resonated = lpState + resonanceGain * std::sin(postEQState * 0.08f);

        return resonated;
    }

    struct MixTransformer {
        float hpState = 0.f, preEQState = 0.f, postEQState = 0.f, lpState = 0.f;
        float fluxMemory = 0.f;
        float slewState = 0.f;

        float process(float in, float drive, float sampleRate) {
            // --- Mix-friendly transformer tuning ---
            const float hpCut = 10.f;            // subtle DC block
            const float preEQCut = 450.f;        // slight mid control
            const float postEQCut = 9500.f;      // gentle top smoothing
            const float lpCut = 18000.f;         // more open top
            const float bias = 0.01f;            // almost symmetrical
            const float fluxAmount = 0.15f;
            const float slewSpeed = 0.85f;       // smoother than P44
            const float satThreshold = 1.5f;     // cleaner at normal drive

            // --- High-pass filter ---
            float hpA = std::exp(-2.f * M_PI * hpCut / sampleRate);
            hpState = hpA * hpState + (1.f - hpA) * in;
            float hp = in - hpState;

            // --- Pre-EQ subtle shaping ---
            float preA = std::exp(-2.f * M_PI * preEQCut / sampleRate);
            preEQState = preA * preEQState + (1.f - preA) * hp;
            float midClean = hp + 0.1f * (hp - preEQState);

            // --- Minimal flux memory ---
            fluxMemory = 0.996f * fluxMemory + 0.004f * midClean;
            float fluxShape = fluxAmount * std::tanh(fluxMemory);

            // --- Drive input stage ---
            float driven = (midClean + bias + fluxShape) * drive;

            // --- Gentle harmonic shaping ---
            float harmonics =
                0.3f * std::tanh(1.0f * driven) +
                0.1f * std::tanh(0.4f * driven * driven);
            float shaped = 0.7f * harmonics + 0.3f * driven;

            // --- Soft limiting (not compression) ---
            float limited = std::tanh(shaped * satThreshold);

            // --- Slew smoothing ---
            slewState += (limited - slewState) * slewSpeed;
            float slewed = slewState;

            // --- Post-EQ filtering ---
            float postA = std::exp(-2.f * M_PI * postEQCut / sampleRate);
            postEQState = postA * postEQState + (1.f - postA) * slewed;

            // --- Final LP rolloff (open transformer) ---
            float lpA = std::exp(-2.f * M_PI * lpCut / sampleRate);
            lpState = lpA * lpState + (1.f - lpA) * postEQState;

            return lpState;
        }
    };

    struct P42CircuitSimple {
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

};
