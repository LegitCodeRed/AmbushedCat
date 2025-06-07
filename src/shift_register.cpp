#include <bitset>
#include <random>

class ShiftRegister16 {
private:
    std::bitset<16> bits;

public:
    // Initialize with all zeros or a seed
    void reset() {
        bits.reset();
    }

    // Insert new bit (0 or 1), shift right
    void shiftIn(bool newBit) {
        bits <<= 1;
        bits.set(0, newBit);  // insert at position 0
    }

    // Get current bit pattern (for DAC or UI)
    std::bitset<16> getBits() const {
        return bits;
    }

    // Get an integer from top N bits (e.g., bits 15 to 8)
    int getTopBitsAsInt(int count = 8) const {
        int value = 0;
        for (int i = 0; i < count; ++i) {
            value <<= 1;
            value |= bits[15 - i];
        }
        return value;
    }
};
