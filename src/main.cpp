#include "driver/gpio.h"
#include "fsm.h"
#include "trame.h"
#include "helper.h"
#include "esp_rom_crc.h"
#include <Arduino.h>

#include <cstring>

#define HALF_BIT_US 10

#define TX_PIN GPIO_NUM_21
#define RX_PIN GPIO_NUM_22

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
bool ENABLE_ERROR_SIMULATION = true;
bool simulate_error = ENABLE_ERROR_SIMULATION;
int target_packet_error = 3;

void IRAM_ATTR gestionnaireSignal() { nack_received = true; }

// 89*2*8 ~= 1500 edges
volatile uint32_t rx_edge_times[1500];
volatile int rx_edge_count = 0;
volatile uint32_t rx_last_edge_time = 0;

void IRAM_ATTR rx_gpio_isr(void *arg) {
	uint32_t now = esp_timer_get_time();
	if (rx_edge_count < 1500) {
		rx_edge_times[rx_edge_count++] = now;
	}
	rx_last_edge_time = now;
}

void sendTrame(const Trame &trame) {
	printTrame("[TX]", trame);
	Serial.flush();

	const uint8_t *trame_bytes = reinterpret_cast<const uint8_t *>(&trame);
	const size_t trame_size = sizeof(Trame);

	uint32_t cycles_per_half_bit = ESP.getCpuFreqMHz() * HALF_BIT_US;
	
	vTaskSuspendAll();

	uint32_t next_edge = ESP.getCycleCount();

	for (size_t i = 0; i < trame_size; ++i) {
		for (int b = 0; b < 8; ++b) {
			int bit = (trame_bytes[i] >> b) & 1;
			if (bit == 1) {
				GPIO.out_w1ts = (1 << TX_PIN);
				next_edge += cycles_per_half_bit;
				while ((int32_t)(ESP.getCycleCount() - next_edge) < 0);

				GPIO.out_w1tc = (1 << TX_PIN);
				next_edge += cycles_per_half_bit;
				while ((int32_t)(ESP.getCycleCount() - next_edge) < 0);
			} else {
				GPIO.out_w1tc = (1 << TX_PIN);
				next_edge += cycles_per_half_bit;
				while ((int32_t)(ESP.getCycleCount() - next_edge) < 0);

				GPIO.out_w1ts = (1 << TX_PIN);
				next_edge += cycles_per_half_bit;
				while ((int32_t)(ESP.getCycleCount() - next_edge) < 0);
			}
		}
	}
	
	GPIO.out_w1tc = (1 << TX_PIN);
	xTaskResumeAll();
	
	vTaskDelay(pdMS_TO_TICKS(50));
}

Trame receiveTrame() {
	rx_edge_count = 0;
	uint32_t wait_start = esp_timer_get_time();

	uint8_t raw_trame[sizeof(Trame)] = {0};
	int byte_index = 0, bit_count = 0, previous_half_bit = -1;
	uint8_t shift_reg = 0;
	bool trame_started = false;
	bool trame_complete = false;

	auto consumeHalfBit = [&](int half_bit) {
		if (trame_complete) return;

		if (previous_half_bit == -1 || previous_half_bit == half_bit) {
			previous_half_bit = half_bit;
			return;
		}
		int bit = (previous_half_bit == 1 && half_bit == 0) ? 1 : 0;
		shift_reg = (shift_reg >> 1) | (bit << 7);

		if (!trame_started) {
			if (shift_reg == 0x7E) {
				trame_started = true;
				raw_trame[0] = 0x55;
				raw_trame[1] = 0x7E;
				byte_index = 2;
				bit_count = 0;
				wait_start = esp_timer_get_time();
			}
		} else {
			if (++bit_count == 8) {
				if (byte_index < sizeof(Trame)) {
					raw_trame[byte_index++] = shift_reg;
				}
				bit_count = 0;
				if (shift_reg == 0x7E && byte_index > 2) {
					trame_complete = true;
				}
				if (byte_index >= sizeof(Trame)) {
					trame_complete = true;
				}
			}
		}
		previous_half_bit = -1;
	};

	int edge_idx = 1;
	int current_level = 1;

	while (true) {
		if (edge_idx < rx_edge_count) {
			int half_bits = ((rx_edge_times[edge_idx] - rx_edge_times[edge_idx - 1]) + HALF_BIT_US / 2) / HALF_BIT_US;
			while (half_bits--) {
				consumeHalfBit(current_level);
			}
			current_level = !current_level;
			edge_idx++;

			if (trame_complete) {
				break;
			}
		} else {
			if (esp_timer_get_time() - wait_start > 1500000) {
				return Trame(TypeCommunication::Debut, 0, 0, nullptr);
			}
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}

	Trame trame = *reinterpret_cast<Trame *>(raw_trame);
	printTrame("[RX]", trame);
	return trame;
}

// Core 1
void txTask(void *pvParameters) {
	Emetteur emetteur(TOTAL_PAQUETS, donnees_txt);
	while (1) {
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
						vTaskDelay(pdMS_TO_TICKS(60));
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
					vTaskDelay(pdMS_TO_TICKS(5));
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
				if (trame.entete.numero_sequence != recepteur.getNumeroSequenceAttendu() || 
					trame.crc16 != esp_rom_crc16_be(0, trame.payload, 80)) {
					nack_sequence = recepteur.getNumeroSequenceAttendu();
					nack_received = true;
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
			else if (trame_corrigee.entete.numero_sequence == recepteur.getNumeroSequenceAttendu() &&
				trame_corrigee.crc16 == esp_rom_crc16_be(0, trame_corrigee.payload, 80)) {
				recordtrame(trame_corrigee.entete.longueur_payload, sizeof(Trame));
				recepteur.setNumeroSequenceAttendu(recepteur.getNumeroSequenceAttendu() + 1);
				recepteur.handleEvent({EventTypeRecepteur::ErreurCorrigee, {}});
			} else if (trame_corrigee.entete.type == TypeCommunication::Data) {
				nack_sequence = recepteur.getNumeroSequenceAttendu();
				nack_received = true;
				Trame nack(TypeCommunication::Nack, 0, recepteur.getNumeroSequenceAttendu(), nullptr);
				sendTrame(nack);
			}
			break;
		}
		}
	}
}

void setup() {
	Serial.setTxBufferSize(2048);
	Serial.begin(115200);
	delay(1000);

	gpio_reset_pin(TX_PIN);
	gpio_set_direction(TX_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(TX_PIN, 0);

	gpio_set_intr_type(RX_PIN, GPIO_INTR_ANYEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(RX_PIN, rx_gpio_isr, NULL);

	xTaskCreatePinnedToCore(txTask, "TX_Task", 8192, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(rxTask, "RX_Task", 8192, NULL, 1, NULL, 0);

	Serial.println("Setup Done");
}

void loop() {
	vTaskDelay(pdMS_TO_TICKS(1000)); 
}