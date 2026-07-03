#include "manchester.h"

namespace Manchester {

    std::vector<uint8_t> encode(const uint8_t* data, size_t length) {
        std::vector<uint8_t> result;
        result.reserve(length * 2);

        for (size_t i = 0; i < length; ++i) {
            uint16_t encoded_pair = 0;
            for (int bit = 7; bit >= 0; --bit) {
                encoded_pair <<= 2;
                // 1 -> 10 (falling), 0 -> 01 (rising)
                encoded_pair |= ((data[i] >> bit) & 1) ? 0b10 : 0b01; 
            }
            result.push_back((encoded_pair >> 8) & 0xFF);
            result.push_back(encoded_pair & 0xFF);
        }
        return result;
    }

    std::vector<uint8_t> decode(const uint8_t* encoded_data, size_t length) {
        std::vector<uint8_t> result;
        if (length % 2 != 0) return result; // Manchester data must be even in length
        
        result.reserve(length / 2);

        for (size_t i = 0; i < length; i += 2) {
            uint16_t encoded_pair = (encoded_data[i] << 8) | encoded_data[i + 1];
            uint8_t decoded_byte = 0;

            for (int bit = 7; bit >= 0; --bit) {
                uint8_t pair = (encoded_pair >> (bit * 2)) & 0b11;
                decoded_byte <<= 1;
                
                if (pair == 0b10) {
                    decoded_byte |= 1;
                } else if (pair == 0b01) {
                    // bit remains 0
                } else {
                    return {}; // Error: Invalid Manchester state (00 or 11)
                }
            }
            result.push_back(decoded_byte);
        }
        return result;
    }
}