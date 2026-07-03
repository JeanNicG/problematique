#include <Arduino.h>
#include "trame.h"
#include "esp_rom_crc.h"
#include <cstring>

Trame::Trame(TypeCommunication type, uint8_t numero_sequence, uint8_t volume, uint8_t payload[80]) {
    entete.type = type;
    entete.numero_sequence = numero_sequence;
    entete.longueur_payload = 80;
    entete.volume = volume;
    if (payload != nullptr) {
        std::memcpy(this->payload, payload, 80);
    } else {
        std::memset(this->payload, 0, 80);
    }
    crc16 = esp_rom_crc16_be(0, this->payload, 80);
}

// helper
const char* typeToString(TypeCommunication type) {
    switch (type) {
        case TypeCommunication::Debut:
            return "Debut";
        case TypeCommunication::Data:
            return "Data";
        case TypeCommunication::Fin:
            return "Fin";
        case TypeCommunication::Nack:
            return "Nack";
        default:
            return "Unknown";
    }
}

void printBytes(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (data[i] < 0x10) {
            Serial.print('0');
        }
        Serial.print(data[i], HEX);
        if (i + 1 < length) {
            Serial.print(' ');
        }
    }
    Serial.println();
}

void printTrame(const char* tag, const Trame& trame) {
    Serial.printf(
        "%s type=%s seq=%u volume=%u payload_len=%u crc=0x%04X\n",
        tag,
        typeToString(trame.entete.type),
        trame.entete.numero_sequence,
        trame.entete.volume,
        trame.entete.longueur_payload,
        trame.crc16
    );
    Serial.print("  payload text: ");
    for (size_t i = 0; i < sizeof(trame.payload); ++i) {
        if (trame.payload[i] == 0) {
            break;
        }
        Serial.write(trame.payload[i]);
    }
    Serial.println();
}
