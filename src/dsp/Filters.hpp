#pragma once
#ifndef FILTERS_H
#define FILTERS_H

#include <cmath>
#include <array>
#include <cstdint>

class BiQuadBase {
public:
    BiQuadBase() {
        bCoef = {0.f, 0.f, 0.f};
        aCoef = {0.f, 0.f};
        w = {0.f, 0.f};
    }

    void Process(float* samples, uint32_t n) {
        float out = 0.f;
        for (uint32_t s = 0; s < n; ++s) {
            out = bCoef[0] * samples[s] + w[0];
            w[0] = bCoef[1] * samples[s] - aCoef[0] * out + w[1];
            w[1] = bCoef[2] * samples[s] - aCoef[1] * out;
            samples[s] = out;
        }
    }

    float Tick(float s) {
        float out = bCoef[0] * s + w[0];
        w[0] = bCoef[1] * s - aCoef[0] * out + w[1];
        w[1] = bCoef[2] * s - aCoef[1] * out;
        return out;
    }

    void SetBiquadCoefs(const std::array<float, 3>& b, const std::array<float, 2>& a) {
        bCoef = b;
        aCoef = a;
    }

protected:
    std::array<float, 3> bCoef;
    std::array<float, 2> aCoef;
    std::array<float, 2> w;
};

class RBJFilter : public BiQuadBase {
public:
    enum FilterType {
        LOWPASS,
        HIGHPASS,
        BANDPASS,
        ALLPASS,
        NOTCH,
        PEAK,
        LOW_SHELF,
        HIGH_SHELF
    };

    RBJFilter(FilterType type = LOWPASS, float cutoff = 1.f, float sr = 44100.f)
        : sampleRate(sr), t(type) {
        Q = 1.f;
        A = 1.f;
        a = {0.f, 0.f, 0.f};
        b = {0.f, 0.f, 0.f};
        SetCutoff(cutoff);
    }

    void SetSampleRate(float sr) {
        sampleRate = sr;
        UpdateCoefficients();
    }

    void UpdateCoefficients() {
        cosOmega = std::cos(omega);
        sinOmega = std::sin(omega);

        switch (t) {
            case LOWPASS: {
                alpha = sinOmega / (2.f * Q);
                b[0] = (1.f - cosOmega) / 2.f;
                b[1] = 1.f - cosOmega;
                b[2] = b[0];
                a[0] = 1.f + alpha;
                a[1] = -2.f * cosOmega;
                a[2] = 1.f - alpha;
            } break;
            case HIGHPASS: {
                alpha = sinOmega / (2.f * Q);
                b[0] = (1.f + cosOmega) / 2.f;
                b[1] = -(1.f + cosOmega);
                b[2] = b[0];
                a[0] = 1.f + alpha;
                a[1] = -2.f * cosOmega;
                a[2] = 1.f - alpha;
            } break;
            case BANDPASS: {
                alpha = sinOmega * std::sinh(std::log(2.f) / 2.f * Q * omega / sinOmega);
                b[0] = sinOmega / 2.f;
                b[1] = 0.f;
                b[2] = -b[0];
                a[0] = 1.f + alpha;
                a[1] = -2.f * cosOmega;
                a[2] = 1.f - alpha;
            } break;
            case ALLPASS: {
                alpha = sinOmega / (2.f * Q);
                b[0] = 1.f - alpha;
                b[1] = -2.f * cosOmega;
                b[2] = 1.f + alpha;
                a[0] = b[2];
                a[1] = b[1];
                a[2] = b[0];
            } break;
            case NOTCH: {
                alpha = sinOmega * std::sinh(std::log(2.f) / 2.f * Q * omega / sinOmega);
                b[0] = 1.f;
                b[1] = -2.f * cosOmega;
                b[2] = 1.f;
                a[0] = 1.f + alpha;
                a[1] = b[1];
                a[2] = 1.f - alpha;
            } break;
            case PEAK: {
                alpha = sinOmega * std::sinh(std::log(2.f) / 2.f * Q * omega / sinOmega);
                b[0] = 1.f + (alpha * A);
                b[1] = -2.f * cosOmega;
                b[2] = 1.f - (alpha * A);
                a[0] = 1.f + (alpha / A);
                a[1] = b[1];
                a[2] = 1.f - (alpha / A);
            } break;
            case LOW_SHELF: {
                alpha = sinOmega / 2.f * std::sqrt((A + 1.f / A) * (1.f / Q - 1.f) + 2.f);
                b[0] = A * ((A + 1.f) - ((A - 1.f) * cosOmega) + (2.f * std::sqrt(A) * alpha));
                b[1] = 2.f * A * ((A - 1.f) - ((A + 1.f) * cosOmega));
                b[2] = A * ((A + 1.f) - ((A - 1.f) * cosOmega) - (2.f * std::sqrt(A) * alpha));
                a[0] = ((A + 1.f) + ((A - 1.f) * cosOmega) + (2.f * std::sqrt(A) * alpha));
                a[1] = -2.f * ((A - 1.f) + ((A + 1.f) * cosOmega));
                a[2] = ((A + 1.f) + ((A - 1.f) * cosOmega) - (2.f * std::sqrt(A) * alpha));
            } break;
            case HIGH_SHELF: {
                alpha = sinOmega / 2.f * std::sqrt((A + 1.f / A) * (1.f / Q - 1.f) + 2.f);
                b[0] = A * ((A + 1.f) + ((A - 1.f) * cosOmega) + (2.f * std::sqrt(A) * alpha));
                b[1] = -2.f * A * ((A - 1.f) + ((A + 1.f) * cosOmega));
                b[2] = A * ((A + 1.f) + ((A - 1.f) * cosOmega) - (2.f * std::sqrt(A) * alpha));
                a[0] = ((A + 1.f) - ((A - 1.f) * cosOmega) + (2.f * std::sqrt(A) * alpha));
                a[1] = 2.f * ((A - 1.f) - ((A + 1.f) * cosOmega));
                a[2] = ((A + 1.f) - ((A - 1.f) * cosOmega) - (2.f * std::sqrt(A) * alpha));
            } break;
        }

        float factor = 1.f / a[0];
        std::array<float, 2> aNorm = {a[1] * factor, a[2] * factor};
        std::array<float, 3> bNorm = {b[0] * factor, b[1] * factor, b[2] * factor};
        SetBiquadCoefs(bNorm, aNorm);
    }

    void SetCutoff(float c) {
        omega = 2.f * (float)M_PI * c / sampleRate;
        UpdateCoefficients();
    }

    void SetQValue(float q) {
        Q = q;
        UpdateCoefficients();
    }

    FilterType GetType() { return t; }

private:
    float sampleRate;
    float omega;
    float cosOmega;
    float sinOmega;
    float Q;
    float alpha;
    float A;
    std::array<float, 3> a;
    std::array<float, 3> b;
    FilterType t;
};

class PinkingFilter {
    double b0, b1, b2, b3, b4, b5, b6;
public:
    PinkingFilter() : b0(0.0), b1(0.0), b2(0.0), b3(0.0), b4(0.0), b5(0.0), b6(0.0) {}
    float process(float s) {
        b0 = 0.99886 * b0 + s * 0.0555179;
        b1 = 0.99332 * b1 + s * 0.0750759;
        b2 = 0.96900 * b2 + s * 0.1538520;
        b3 = 0.86650 * b3 + s * 0.3104856;
        b4 = 0.55000 * b4 + s * 0.5329522;
        b5 = -0.7616 * b5 - s * 0.0168980;
        double pink = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + (s * 0.5362)) * 0.11;
        b6 = s * 0.115926;
        return (float)pink;
    }
};

class BrowningFilter {
    float l;
public:
    BrowningFilter() : l(0.f) {}
    float process(float s) {
        float brown = (l + (0.02f * s)) / 1.02f;
        l = brown;
        return brown * 3.5f;
    }
};

#endif // FILTERS_H
