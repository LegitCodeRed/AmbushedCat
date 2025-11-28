/* Minimal JUCE header replacement for vital DSP
 * This header provides minimal definitions needed by vital DSP code
 * without requiring the full JUCE library.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <memory>

// Define JUCE_LEAK_DETECTOR as a no-op since we're not using JUCE's memory leak detection
#define JUCE_LEAK_DETECTOR(ClassName)

// Define JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR macro
// This macro disables copy constructor and copy assignment operator
#define JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(ClassName) \
    ClassName (const ClassName&) = delete; \
    ClassName& operator= (const ClassName&) = delete; \
    JUCE_LEAK_DETECTOR(ClassName)

// Basic type definitions that might be expected
namespace juce {
    // Empty namespace to satisfy any juce:: references if they exist
}
