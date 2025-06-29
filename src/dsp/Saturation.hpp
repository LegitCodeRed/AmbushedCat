#pragma once
#include <cmath>
#include <dsp/resampler.hpp>
#include "dsp.hpp"

namespace dspext {

template<int OS>
class Saturator {
public:
    dsp::Upsampler<OS, 8> upsampler;
    dsp::Decimator<OS, 8> decimator;
    float hpState = 0.f;
    float preLPState = 0.f;
    float postLPState = 0.f;
    float env = 0.f;
    float mix = 1.f;

    enum Circuit {
        EASY,
        MODERATE,
        HEAVY
    };

    float process(float in, float drive, float sampleRate, Circuit circuit) {
        if (drive <= 0.f)
            return in;
        // Soft limit the input to avoid digital clipping when input and drive are high
        float norm = std::tanh(in);
        float upBuf[OS];
        float satBuf[OS];
        upsampler.process(norm, upBuf);
        for (int i = 0; i < OS; i++) {
            float x = upBuf[i];
            // Pre emphasis
            float hpA = std::exp(-2.f * M_PI * 30.f / (sampleRate * OS));
            hpState = hpA * hpState + (1.f - hpA) * x;
            x -= hpState;
            float preA = std::exp(-2.f * M_PI * 8000.f / (sampleRate * OS));
            preLPState = preA * preLPState + (1.f - preA) * x;
            x = preLPState;
            float rect = std::fabs(x);
            env += 0.002f * (rect - env);
            float dynDrive = drive * (1.f + 0.2f * env);
            float cubic = x - (x * x * x) / 3.f;
            float rational = x * (27.f + x * x) / (27.f + 9.f * x * x);
            float shaped = 0.f;
            switch (circuit) {
                default:
                case HEAVY:
                    shaped = 0.4f * cubic + 0.6f * rational;
                    break;
                case MODERATE:
                    shaped = 0.5f * x + 0.5f * cubic;
                    break;
                case EASY:
                    shaped = 0.8f * x + 0.2f * cubic;
                    break;
            }
            float saturated = shaped * dynDrive;
            // simple soft compression
            float comp = 1.f / (1.f + 0.5f * dynDrive * env);
            float post = saturated * comp;
            float postA = std::exp(-2.f * M_PI * 12000.f / (sampleRate * OS));
            postLPState = postA * postLPState + (1.f - postA) * post;
            satBuf[i] = postLPState;
        }
        float sat = decimator.process(satBuf);
        return sat * mix + in * (1.f - mix);
    }
};

} // namespace dspext
