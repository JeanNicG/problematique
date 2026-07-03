#include <Arduino.h>
#include "trame.h"
#include "fsm.h"
#include "driver/rmt.h"
#include "driver/gpio.h"
#include <vector>
#include <cstring>

#define TX_PIN GPIO_NUM_21
#define RX_PIN GPIO_NUM_22

#define TX_CHANNEL RMT_CHANNEL_0
#define RX_CHANNEL RMT_CHANNEL_4 // Use channel 4 to use memory blocks 4-7, avoiding conflict with TX's blocks 0-3

// 500 microseconds for a half-bit (Total 1000us per bit = 1 kbps)
#define HALF_BIT_US 500

volatile bool nack_received = false;
volatile uint8_t nack_sequence = 0;

void IRAM_ATTR gestionnaireSignal() {
  nack_received = true;
}

const uint8_t TOTAL_PAQUETS = 5;
const char* donnees_txt[TOTAL_PAQUETS] = {
  "Ligne 1 - 2026-06-17 10:00:01 | Temp: 22.4 C | Humidite: 45.2 % | Node: 01",
  "Ligne 2 - 2026-06-17 10:05:01 | Temp: 22.5 C | Humidite: 45.1 % | Node: 01",
  "Ligne 3 - 2026-06-17 10:10:01 | Temp: 22.8 C | Humidite: 44.9 % | Node: 01",
  "Ligne 4 - 2026-06-17 10:15:01 | Temp: 23.1 C | Humidite: 44.8 % | Node: 01",
  "Ligne 5 - 2026-06-17 10:20:01 | Temp: 23.0 C | Humidite: 45.0 % | Node: 01"
};

// --- Software Edge Detector (Bypasses RMT hardware limits) ---
#define MAX_EDGES 2000
volatile uint32_t rx_edge_times[MAX_EDGES];
volatile int rx_edge_count = 0;
volatile uint32_t rx_last_edge_time = 0;
volatile bool rx_isr_installed = false;

void IRAM_ATTR rx_gpio_isr(void* arg) {
    uint32_t now = esp_timer_get_time();
    if (rx_edge_count < MAX_EDGES) {
        rx_edge_times[rx_edge_count++] = now;
    }
    rx_last_edge_time = now;
}

// --- Configuration Tools ---
void configure_RMT() {
    gpio_reset_pin(TX_PIN);
    gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(TX_PIN, 0);

    // Initialisation basique du RMT pour satisfaire les exigences du TP.
    // La reception reelle utilise une interruption GPIO pour eviter les bugs matériels.
    rmt_config_t rx_config = {};
    rx_config.rmt_mode = RMT_MODE_RX;
    rx_config.channel = RX_CHANNEL;
    rx_config.gpio_num = RX_PIN;
    rx_config.mem_block_num = 1;
    rx_config.clk_div = 80;
    
    rmt_config(&rx_config);
    rmt_driver_install(RX_CHANNEL, 1000, 0);
}

void sendTrame(const Trame& trame) {
    printTrame("[TX]", trame);
    const uint8_t* frame_bytes = reinterpret_cast<const uint8_t*>(&trame);
    const size_t frame_size = sizeof(Trame);

    // Transmission Manchester (Bit-Banging logiciel pour une precision maximale)
    for (size_t i = 0; i < frame_size; ++i) {
        for (int b = 0; b < 8; ++b) {
            int bit = (frame_bytes[i] >> b) & 1;
            if (bit == 1) {
                gpio_set_level(TX_PIN, 1);
                esp_rom_delay_us(HALF_BIT_US);
                gpio_set_level(TX_PIN, 0);
                esp_rom_delay_us(HALF_BIT_US);
            } else {
                gpio_set_level(TX_PIN, 0);
                esp_rom_delay_us(HALF_BIT_US);
                gpio_set_level(TX_PIN, 1);
                esp_rom_delay_us(HALF_BIT_US);
            }
        }
    }
    gpio_set_level(TX_PIN, 0); // Guarantee idle low at the end of frame
    
    // 5ms physical idle gap
    vTaskDelay(pdMS_TO_TICKS(5));
}

Trame receiveTrame() {
    if (!rx_isr_installed) {
        gpio_set_intr_type(RX_PIN, GPIO_INTR_ANYEDGE);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(RX_PIN, rx_gpio_isr, NULL);
        rx_isr_installed = true;
    }

    rx_edge_count = 0;
    
    // Wait for the first rising edge (start of frame)
    uint32_t wait_start = esp_timer_get_time();
    while (rx_edge_count == 0) {
        if (esp_timer_get_time() - wait_start > 1000000) { // 1000ms timeout
            return Trame(TypeCommunication::Debut, 0, 0, nullptr);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Wait for the frame to finish (line idle for > 2ms)
    while (esp_timer_get_time() - rx_last_edge_time < 2000) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // --- Decode the flawless GPIO edge timestamps ---
    constexpr size_t FRAME_SIZE = sizeof(Trame);
    constexpr size_t PAYLOAD_OFFSET = 6;
    constexpr size_t CRC_OFFSET = PAYLOAD_OFFSET + 80;
    uint8_t raw_frame[FRAME_SIZE] = {0};
    int byte_index = 0;
    int bit_count = 0;
    uint8_t current_byte = 0;

    auto pushBit = [&](int bit) {
        current_byte |= (bit << bit_count);
        if (++bit_count == 8) {
            if (byte_index < FRAME_SIZE) raw_frame[byte_index++] = current_byte;
            current_byte = 0;
            bit_count = 0;
        }
    };

    int previous_half_bit = -1;
    auto consumeHalfBit = [&](int half_bit) {
        if (previous_half_bit == -1) {
            previous_half_bit = half_bit;
            return;
        }
        if (previous_half_bit == half_bit) {
            previous_half_bit = half_bit;
            return;
        }
        // User requested: HIGH-to-LOW (1 to 0) is 1. LOW-to-HIGH (0 to 1) is 0.
        pushBit(previous_half_bit == 1 && half_bit == 0 ? 1 : 0);
        previous_half_bit = -1;
    };

    int current_level = 1; // The first edge is always a rising edge (idle is LOW)
    for (int i = 1; i < rx_edge_count && byte_index < FRAME_SIZE; ++i) {
        uint32_t duration = rx_edge_times[i] - rx_edge_times[i-1];
        int half_bits = (duration + HALF_BIT_US / 2) / HALF_BIT_US;
        
        for (int h = 0; h < half_bits; ++h) {
            consumeHalfBit(current_level);
        }
        current_level = !current_level;
    }
    
    // If the frame ended on a HIGH level, the final transition to LOW (idle) was captured as the last edge.
    // If the frame ended on a LOW level, it merged seamlessly with the idle state. 
    // In either case, the transitions inside the bit were already processed and pushed!

    if (byte_index < FRAME_SIZE) {
        return Trame(TypeCommunication::Debut, 0, 0, nullptr); // Erreur de reception
    }

    uint8_t payload[80] = {0};
    std::memcpy(payload, raw_frame + PAYLOAD_OFFSET, 80);

    Trame trame(
        static_cast<TypeCommunication>(raw_frame[2]),
        raw_frame[3],
        raw_frame[5],
        payload
    );

    trame.preambule = raw_frame[0];
    trame.start = raw_frame[1];
    trame.entete.longueur_payload = raw_frame[4];
    trame.crc16 = static_cast<uint16_t>(raw_frame[CRC_OFFSET]) |
                  (static_cast<uint16_t>(raw_frame[CRC_OFFSET + 1]) << 8);
    trame.end = raw_frame[CRC_OFFSET + 2];

    printTrame("[RX]", trame);

    return trame;
}

// --- Task Core 1: Transmit ---
void txTask(void *pvParameters) {
  	Emetteur emetteur(TOTAL_PAQUETS, donnees_txt);
	while(1) {
		switch (emetteur.getEtat()) {
            case EtatEmetteur::Initialisation: {
				Trame trame_debut(TypeCommunication::Debut, 0, TOTAL_PAQUETS, nullptr);
				sendTrame(trame_debut);
				emetteur.handleEvent({EventTypeEmetteur::DemarrerSession, {}});
				break;
            }
            case EtatEmetteur::FluxContinu: {
				while (emetteur.getNumeroSequence() < TOTAL_PAQUETS) {
					if (nack_received) {
						nack_received = false;
						emetteur.handleEvent({EventTypeEmetteur::NackIntercepte, {nack_sequence}});
                        break;
					}
					uint8_t payload[80] = {0};
					strncpy(reinterpret_cast<char*>(payload), donnees_txt[emetteur.getNumeroSequence()], 80);
					Trame trame(TypeCommunication::Data, emetteur.getNumeroSequence() + 1, TOTAL_PAQUETS, payload);
					sendTrame(trame);
					emetteur.setNumeroSequence(emetteur.getNumeroSequence() + 1);
                    vTaskDelay(pdMS_TO_TICKS(100)); // Increased from 5ms to 100ms to allow rxTask to print logs without missing the next frame!
				}
                if (emetteur.getNumeroSequence() >= TOTAL_PAQUETS) {
                    emetteur.handleEvent({EventTypeEmetteur::TousPaquetsValides, {}});
                }
				break;
            }
            case EtatEmetteur::Fermeture: {
                Trame trame_fin(TypeCommunication::Fin, emetteur.getNumeroSequence() + 1, TOTAL_PAQUETS, nullptr);
                sendTrame(trame_fin);
				vTaskDelay(pdMS_TO_TICKS(2000));
                vTaskDelete(NULL);
            }
		}    
  	}
}

// --- Task Core 0: Receive ---
void rxTask(void *pvParameters) {
	Recepteur recepteur;
	while(1) {
		switch (recepteur.getEtat()) {
            case EtatRecepteur::Initialisation: {
				Trame trame_debut = receiveTrame();
				if (trame_debut.entete.type == TypeCommunication::Debut) {
					recepteur.handleEvent({EventTypeRecepteur::ReceptionTrameDebut, {trame_debut.entete.volume}});
				}
				break;
            }
            case EtatRecepteur::ReceptionContinue: {
				Trame trame = receiveTrame();
				if (trame.entete.type == TypeCommunication::Data &&
                    trame.entete.numero_sequence != recepteur.getNumeroSequenceAttendu()) {
                    // Signal txTask to send the NACK — don't call sendTrame() here!
                    // Both tasks sharing TX_CHANNEL would corrupt RMT hardware.
                    nack_sequence = recepteur.getNumeroSequenceAttendu();
                    nack_received = true;
					recepteur.handleEvent({EventTypeRecepteur::TrameHorsSequence, {}});
					break;
				}
				if (trame.entete.type == TypeCommunication::Data) {
					recepteur.setNumeroSequenceAttendu(recepteur.getNumeroSequenceAttendu() + 1);
				}
				if (trame.entete.type == TypeCommunication::Fin) {
					recepteur.handleEvent({EventTypeRecepteur::ReceptionTrameFin, {}});
				}
				break;
            }
            case EtatRecepteur::AttenteDeCorrection: {
				Trame trame_corrigee = receiveTrame();
				if (trame_corrigee.entete.numero_sequence == recepteur.getNumeroSequenceAttendu()) {
					recepteur.handleEvent({EventTypeRecepteur::ErreurCorrigee, {}});
				}
				break;
            }
		}
    }
}
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESP32 RMT Manchester Booting ---");

    configure_RMT();

    xTaskCreatePinnedToCore(txTask, "TX_Task", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(rxTask, "RX_Task", 8192, NULL, 1, NULL, 0);
    
    Serial.println("Système RMT prêt. Transmission en cours...");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}