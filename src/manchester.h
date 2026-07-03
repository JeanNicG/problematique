#pragma once
#include <cstdint>
#include <vector>

namespace Manchester {
    // Encodes raw bytes into Manchester (1 byte -> 2 bytes)
    std::vector<uint8_t> encode(const uint8_t* data, size_t length);
    
    // Decodes Manchester bytes back to raw data. 
    // Returns an empty vector if an invalid transition (00 or 11) is detected.
    std::vector<uint8_t> decode(const uint8_t* encoded_data, size_t length);
}