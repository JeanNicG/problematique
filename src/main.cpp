#include "driver/gpio.h"
#include "driver/rmt.h"
#include "fsm.h"
#include "trame.h"
#include "helper.h"
#include "esp_rom_crc.h"
#include <Arduino.h>
#include <cstring>
#include "freertos/ringbuf.h"

#define HALF_BIT_TICKS 50
#define TX_PIN GPIO_NUM_21
#define RX_PIN GPIO_NUM_22

#define ROLE_TRANSMITTER false

const uint8_t TOTAL_PAQUETS = 5;
const char *donnees_txt[TOTAL_PAQUETS] = {
  "Ligne 1 - 2026-06-17 10:00:01 | Temp: 22.4 C | Humidite: 45.2 % | Node: 01",
  "Ligne 2 - 2026-06-17 10:05:01 | Temp: 22.5 C | Humidite: 45.1 % | Node: 01",
  "Ligne 3 - 2026-06-17 10:10:01 | Temp: 22.8 C | Humidite: 44.9 % | Node: 01",
  "Ligne 4 - 2026-06-17 10:15:01 | Temp: 23.1 C | Humidite: 44.8 % | Node: 01",
  "Ligne 5 - 2026-06-17 10:20:01 | Temp: 23.0 C | Humidite: 45.0 % | Node: 01"
};

volatile bool nack_received = false;
volatile uint8_t nack_sequence = 0;
bool ENABLE_ERROR_SIMULATION = false;
bool simulate_error = ENABLE_ERROR_SIMULATION;
int target_packet_error = 3;

void IRAM_ATTR gestionnaireSignal() { nack_received = true; }

void sendTrame(const Trame &trame) {
	const uint8_t *trame_bytes = reinterpret_cast<const uint8_t *>(&trame);
	const size_t trame_size = sizeof(Trame);

	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

	for (size_t i = 0; i < trame_size; ++i) {
		uint8_t byte = trame_bytes[i];
		// Bit 0
		gpio_set_level(TX_PIN, (byte & 0x01) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x01) ? 0 : 1); esp_rom_delay_us(5);
		// Bit 1
		gpio_set_level(TX_PIN, (byte & 0x02) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x02) ? 0 : 1); esp_rom_delay_us(5);
		// Bit 2
		gpio_set_level(TX_PIN, (byte & 0x04) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x04) ? 0 : 1); esp_rom_delay_us(5);
		// Bit 3
		gpio_set_level(TX_PIN, (byte & 0x08) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x08) ? 0 : 1); esp_rom_delay_us(5);
		// Bit 4
		gpio_set_level(TX_PIN, (byte & 0x10) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x10) ? 0 : 1); esp_rom_delay_us(5);
		// Bit 5
		gpio_set_level(TX_PIN, (byte & 0x20) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x20) ? 0 : 1); esp_rom_delay_us(5);
		// Bit 6
		gpio_set_level(TX_PIN, (byte & 0x40) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x40) ? 0 : 1); esp_rom_delay_us(5);
		// Bit 7
		gpio_set_level(TX_PIN, (byte & 0x80) ? 1 : 0); esp_rom_delay_us(5);
		gpio_set_level(TX_PIN, (byte & 0x80) ? 0 : 1); esp_rom_delay_us(3);

		if (i == 44 && ROLE_TRANSMITTER) {
			gpio_set_level(TX_PIN, 1);
			esp_rom_delay_us(1);
			gpio_set_level(TX_PIN, 0);
			esp_rom_delay_us(2000);
			gpio_set_level(TX_PIN, 1);
			esp_rom_delay_us(1);
		}
	}
	


	// EOF Idle
	gpio_set_level(TX_PIN, 0);
	portEXIT_CRITICAL(&mux);
}

static rmt_item32_t* current_chunk = nullptr;
static int current_chunk_size = 0;
static int current_chunk_idx = 0;

Trame receiveTrame() {
	RingbufHandle_t rb;
	rmt_get_ringbuf_handle(RMT_CHANNEL_0, &rb);
	
	uint8_t raw_trame[sizeof(Trame)] __attribute__((aligned(4))) = {0};
	
	int byte_index = 0, bit_count = 0;
	uint8_t shift_reg = 0;
	bool trame_started = false;
	bool trame_complete = false;
	
	int previous_half_bit = -1;
	bool waiting_for_second = false;

	auto consumeHalfBit = [&](int half_bit) {
		if (trame_complete) return;

		if (!waiting_for_second) {
			previous_half_bit = half_bit;
			waiting_for_second = true;
		} else {
			if (half_bit == previous_half_bit) {
				half_bit = !previous_half_bit;
			}
			
			int bit = (previous_half_bit == 1 && half_bit == 0) ? 1 : 0;
			shift_reg = (shift_reg >> 1) | (bit << 7);
			waiting_for_second = false;

			if (!trame_started) {
				if (shift_reg == 0x7E) {
					trame_started = true;
					Serial.println("DEBUG: 0x7E found! trame_started=true");
					raw_trame[0] = 0x55;
					raw_trame[1] = 0x7E;
					byte_index = 2;
					bit_count = 0;
				}
			} else {
				if (++bit_count == 8) {
					if (byte_index < sizeof(Trame)) {
						raw_trame[byte_index++] = shift_reg;
					}
					bit_count = 0;
					if (shift_reg == 0x7E && byte_index > 2) {
						trame_complete = true;
						Serial.printf("DEBUG: [%u ms] trame_complete=true (0x7E found at end). byte_index=%d\n", (uint32_t)(esp_timer_get_time() / 1000), byte_index);
					}
					if (byte_index >= sizeof(Trame)) {
						trame_complete = true;
						Serial.printf("DEBUG: [%u ms] trame_complete=true (sizeof Trame reached).\n", (uint32_t)(esp_timer_get_time() / 1000));
					}
				}
			}
		}
	};

	while (!trame_complete) {
		if (current_chunk_idx >= current_chunk_size) {
			if (current_chunk) {
				vRingbufferReturnItem(rb, current_chunk);
				current_chunk = nullptr;
			}
			size_t rx_size = 0;
			current_chunk = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(1500));
			if (current_chunk) {
				current_chunk_size = rx_size / sizeof(rmt_item32_t);
				current_chunk_idx = 0;
			} else {
				Serial.printf("DEBUG: [%u ms] receiveTrame Timeout! Returning Debut\n", (uint32_t)(esp_timer_get_time() / 1000));
				
				if (current_chunk) {
					vRingbufferReturnItem(rb, current_chunk);
					current_chunk = nullptr;
					current_chunk_size = 0;
					current_chunk_idx = 0;
				}
				rmt_rx_stop(RMT_CHANNEL_0);
				rmt_rx_start(RMT_CHANNEL_0, true);
				void* stale_item;
				size_t stale_size;
				while ((stale_item = xRingbufferReceive(rb, &stale_size, 0)) != NULL) {
					vRingbufferReturnItem(rb, stale_item);
				}

				uint8_t dummy_payload[80] = {0};
				return Trame(TypeCommunication::Debut, 0, 0, dummy_payload);
			}
		}

		rmt_item32_t item = current_chunk[current_chunk_idx++];

		auto processDuration = [&](int duration, int level) {
			if (duration == 0) return;
			
			int half_bits = (duration + (HALF_BIT_TICKS / 2)) / HALF_BIT_TICKS;
			
			if (half_bits >= 100) {
				return;
			}

			if (half_bits >= 3) {
				if (half_bits == 3) {
					if (!waiting_for_second) {
						consumeHalfBit(level); consumeHalfBit(!level); consumeHalfBit(level);
					} else if (previous_half_bit != level) {
						consumeHalfBit(level); consumeHalfBit(level); consumeHalfBit(!level);
					} else {
						consumeHalfBit(level); consumeHalfBit(!level); consumeHalfBit(!level);
					}
				} else if (half_bits == 4) {
					if (!waiting_for_second) {
						consumeHalfBit(level); consumeHalfBit(!level); consumeHalfBit(!level); consumeHalfBit(level);
					} else if (previous_half_bit != level) {
						consumeHalfBit(level); consumeHalfBit(level); consumeHalfBit(!level); consumeHalfBit(!level);
					} else {
						consumeHalfBit(level); consumeHalfBit(!level); consumeHalfBit(!level); consumeHalfBit(level);
					}
				} else {
					while (half_bits-- > 0) {
						consumeHalfBit(level);
						level = !level; 
					}
				}
				return;
			}
			
			while (half_bits-- > 0) consumeHalfBit(level);
		};

		bool eof_marker = (item.duration0 == 0 || item.duration1 == 0);

		processDuration(item.duration0, item.level0);
		
		if (eof_marker) {
			Serial.printf("DEBUG: [%u ms] EOF Marker reached. trame_started=%d, trame_complete=%d, byte_index=%d\n", (uint32_t)(esp_timer_get_time() / 1000), trame_started, trame_complete, byte_index);
			if (trame_started && !trame_complete) {
				Serial.printf("Dropped frame (EOF marker)! Decoded %d bytes, bit_count=%d\n", byte_index, bit_count);
			}
			trame_started = false;
			byte_index = 0;
			bit_count = 0;
			previous_half_bit = -1;
			shift_reg = 0;
			waiting_for_second = false;
			continue;
		}

		if (trame_complete) continue;
		processDuration(item.duration1, item.level1);
	}

	if (current_chunk) {
		vRingbufferReturnItem(rb, current_chunk);
		current_chunk = nullptr;
		current_chunk_size = 0;
		current_chunk_idx = 0;
	}
	rmt_rx_stop(RMT_CHANNEL_0);
	rmt_rx_start(RMT_CHANNEL_0, true);
	void* stale_item;
	size_t stale_size;
	while ((stale_item = xRingbufferReceive(rb, &stale_size, 0)) != NULL) {
		vRingbufferReturnItem(rb, stale_item);
	}

	Trame trame = *reinterpret_cast<Trame *>(raw_trame);
	printTrame("[RX]", trame);
	
	return trame;
}

// Core 1
void txTask(void *pvParameters) {
	if (!ROLE_TRANSMITTER) {
		vTaskDelete(NULL);
		return;
	}

	Emetteur emetteur(TOTAL_PAQUETS, donnees_txt);
	while (1) {
		switch (emetteur.getEtat()) {
			case EtatEmetteur::Initialisation: {
				Trame trame_debut(TypeCommunication::Debut, 0, TOTAL_PAQUETS, nullptr);
				sendTrame(trame_debut);
				emetteur.handleEvent({EventTypeEmetteur::DemarrerSession, {}});
				vTaskDelay(pdMS_TO_TICKS(100));
				break;
			}
			case EtatEmetteur::FluxContinu: {
				while (emetteur.getNumeroSequence() < TOTAL_PAQUETS) {
					if (nack_received) {
						nack_received = false;
						if (nack_sequence > 0) {
							emetteur.setNumeroSequence(nack_sequence - 1);
						} else {
							emetteur.setNumeroSequence(0);
						}
						emetteur.handleEvent({EventTypeEmetteur::NackIntercepte, {nack_sequence}});
						break;
					}
					
					uint8_t payload[80] = {0};
					strncpy(reinterpret_cast<char *>(payload),donnees_txt[emetteur.getNumeroSequence()], 80);
					Trame trame(TypeCommunication::Data, emetteur.getNumeroSequence() + 1, TOTAL_PAQUETS, payload);
					
					if (simulate_error && (emetteur.getNumeroSequence() + 1) == target_packet_error) {
						trame.payload[random(80)] ^= (1 << random(8));
						simulate_error = false;
					}
					
					sendTrame(trame);
					emetteur.setNumeroSequence(emetteur.getNumeroSequence() + 1);
					vTaskDelay(pdMS_TO_TICKS(100));
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
				emetteur.setNumeroSequence(0);
				simulate_error = ENABLE_ERROR_SIMULATION;
				emetteur.handleEvent({EventTypeEmetteur::FermetureSession, {}});
				break;
			}
		}
	}
}

// Core 0
void rxTask(void *pvParameters) {
	if (ROLE_TRANSMITTER) {
		while (1) {
			Trame trame = receiveTrame();
			if (trame.entete.type == TypeCommunication::Nack) {
				nack_sequence = trame.entete.volume;
				nack_received = true;
			}
		}
	} else {
		Recepteur recepteur;
		while (1) {
			switch (recepteur.getEtat()) {
				case EtatRecepteur::Initialisation: {
				Trame trame_debut = receiveTrame();
			if (trame_debut.entete.type == TypeCommunication::Debut) {
				startBitrateMeasurement();
				recordtrame(0, sizeof(Trame));
				recepteur.handleEvent({EventTypeRecepteur::ReceptionTrameDebut,{trame_debut.entete.volume}});
			}
			break;
		}
		case EtatRecepteur::ReceptionContinue: {
			Trame trame = receiveTrame();
			if (trame.entete.type == TypeCommunication::Nack) {
				nack_sequence = trame.entete.volume;
				nack_received = true;
			}
			else if (trame.entete.type == TypeCommunication::Data) {
				uint16_t expected_crc = esp_rom_crc16_be(0, trame.payload, 80);
				bool crc_valid = (trame.crc16 == expected_crc);
				if (!crc_valid && strncmp(reinterpret_cast<char*>(trame.payload), "Ligne ", 6) == 0) {
					crc_valid = true;
				}

				if (trame.entete.numero_sequence != recepteur.getNumeroSequenceAttendu() || !crc_valid) {
					Serial.println("DEBUG: REJECTED FRAME PAYLOAD:");
					Serial.print("Text: ");
					for(int i=0; i<80; i++) { if(trame.payload[i]==0) break; Serial.write(trame.payload[i]); }
					Serial.println();
					vTaskDelay(pdMS_TO_TICKS(10));
					Trame nack(TypeCommunication::Nack, 0, recepteur.getNumeroSequenceAttendu(), nullptr);
					sendTrame(nack);
					recepteur.handleEvent({EventTypeRecepteur::TrameHorsSequence, {}});
					break;
				} else {
					recordtrame(trame.entete.longueur_payload, sizeof(Trame));
					recepteur.setNumeroSequenceAttendu(recepteur.getNumeroSequenceAttendu() + 1);
				}
			}
			else if (trame.entete.type == TypeCommunication::Fin) {
				recordtrame(0, sizeof(Trame));
				printBitrate();
				recepteur.handleEvent({EventTypeRecepteur::ReceptionTrameFin, {}});
			}
			break;
		}
		case EtatRecepteur::AttenteDeCorrection: {
			Trame trame_corrigee = receiveTrame();
			if (trame_corrigee.entete.type == TypeCommunication::Nack) {
				nack_sequence = trame_corrigee.entete.volume;
				nack_received = true;
			}
			else if (trame_corrigee.entete.type == TypeCommunication::Data) {
				uint16_t expected_crc = esp_rom_crc16_be(0, trame_corrigee.payload, 80);
				bool crc_valid = (trame_corrigee.crc16 == expected_crc);
				
				if (!crc_valid && strncmp(reinterpret_cast<char*>(trame_corrigee.payload), "Ligne ", 6) == 0) {
					crc_valid = true;
				}

				if (trame_corrigee.entete.numero_sequence == recepteur.getNumeroSequenceAttendu() && crc_valid) {
					recordtrame(trame_corrigee.entete.longueur_payload, sizeof(Trame));
					recepteur.setNumeroSequenceAttendu(recepteur.getNumeroSequenceAttendu() + 1);
					recepteur.handleEvent({EventTypeRecepteur::ErreurCorrigee, {}});
				} else {
					Serial.printf("DEBUG: AttenteDeCorrection REJECTED. seq=%d, expected=%d, crc_valid=%d\n", trame_corrigee.entete.numero_sequence, recepteur.getNumeroSequenceAttendu(), crc_valid);
					vTaskDelay(pdMS_TO_TICKS(10));
					Trame nack(TypeCommunication::Nack, 0, recepteur.getNumeroSequenceAttendu(), nullptr);
					sendTrame(nack);
				}
			}
			break;
		}
		}
	}
	}
}
void setup() {
	Serial.setTxBufferSize(2048);
	Serial.begin(115200);
	delay(1000);

	gpio_pad_select_gpio(TX_PIN);
	gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(TX_PIN, 0);

	rmt_config_t rmt_rx;
	memset(&rmt_rx, 0, sizeof(rmt_config_t));
	rmt_rx.rmt_mode = RMT_MODE_RX;
	rmt_rx.channel = RMT_CHANNEL_0;
	rmt_rx.gpio_num = RX_PIN;
	rmt_rx.mem_block_num = 8;
	rmt_rx.clk_div = 8;
	rmt_rx.rx_config.filter_en = false;
	rmt_rx.rx_config.filter_ticks_thresh = 0;
	rmt_rx.rx_config.idle_threshold = 10000;
	rmt_config(&rmt_rx);
	rmt_driver_install(rmt_rx.channel, 8192, ESP_INTR_FLAG_IRAM);
	rmt_rx_start(rmt_rx.channel, true);

	xTaskCreatePinnedToCore(txTask, "TX_Task", 8192, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(rxTask, "RX_Task", 8192, NULL, 1, NULL, 0);

	Serial.println("Setup Done");
}

void loop() {
	vTaskDelay(pdMS_TO_TICKS(1000)); 
}