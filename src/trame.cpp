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
