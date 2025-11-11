#include "hx711.h"
#include <esp_rom_sys.h>

const char* HX711::TAG = "HX711";

HX711::HX711() : GAIN(1), OFFSET(0), SCALE(1.0f), read_interval_us(12500) {
	// Default read interval is 80Hz (1000000us / 80 = 12500us)
}

HX711::~HX711() {
}

void HX711::begin(gpio_num_t dout, gpio_num_t pd_sck, byte gain) {
	PD_SCK = pd_sck;
	DOUT = dout;

	ESP_LOGI(TAG, "Initializing HX711 - DOUT: GPIO%d, PD_SCK: GPIO%d, Gain: %d", dout, pd_sck, gain);

	// Configure PD_SCK as output and set LOW
	gpio_reset_pin(PD_SCK);
	gpio_set_direction(PD_SCK, GPIO_MODE_OUTPUT);
	gpio_set_level(PD_SCK, 0);
	ESP_LOGI(TAG, "PD_SCK configured as output and set LOW");

	// Configure DOUT as input
	gpio_reset_pin(DOUT);
	gpio_set_direction(DOUT, GPIO_MODE_INPUT);
	ESP_LOGI(TAG, "DOUT configured as input");

	// Set the gain (this will take effect on next read)
	set_gain(gain);

	ESP_LOGI(TAG, "HX711 initialization complete - waiting for first conversion...");
}

bool HX711::is_ready() {
	// HX711 is ready when DOUT is LOW
	return gpio_get_level(DOUT) == 0;
}

void HX711::set_gain(byte gain) {
	ESP_LOGI(TAG, "Setting gain to %d", gain);

	switch (gain) {
		case 128:  // Channel A, gain factor 128
			GAIN = 1;  // 25 pulses total (24 data + 1 gain)
			ESP_LOGI(TAG, "Configured: Channel A, Gain 128 (25 pulses)");
			break;
		case 64:   // Channel A, gain factor 64
			GAIN = 3;  // 27 pulses total (24 data + 3 gain)
			ESP_LOGI(TAG, "Configured: Channel A, Gain 64 (27 pulses)");
			break;
		case 32:   // Channel B, gain factor 32
			GAIN = 2;  // 26 pulses total (24 data + 2 gain)
			ESP_LOGI(TAG, "Configured: Channel B, Gain 32 (26 pulses)");
			break;
		default:
			ESP_LOGW(TAG, "Invalid gain %d, defaulting to 128", gain);
			GAIN = 1;
			break;
	}
}

void HX711::set_sample_rate(uint32_t rate_hz) {
	if (rate_hz == 0) {
		ESP_LOGW(TAG, "Invalid sample rate 0 Hz, using default 80 Hz");
		rate_hz = 80;
	}

	read_interval_us = 1000000 / rate_hz;
	ESP_LOGI(TAG, "Sample rate set to %lu Hz (minimum %lu us between reads)", rate_hz, read_interval_us);
}

uint8_t HX711::read_bit() {
	// Clock pulse: LOW -> HIGH -> read -> LOW
	gpio_set_level(PD_SCK, 1);
	esp_rom_delay_us(1);  // Wait for data to settle (datasheet: min 0.1us, using 1us for safety)

	uint8_t bit = gpio_get_level(DOUT);

	gpio_set_level(PD_SCK, 0);
	esp_rom_delay_us(1);  // Keep clock low for at least 0.2us

	return bit;
}

void HX711::clock_pulse() {
	gpio_set_level(PD_SCK, 1);
	esp_rom_delay_us(1);
	gpio_set_level(PD_SCK, 0);
	esp_rom_delay_us(1);
}

int32_t HX711::read_24bit() {
	int32_t value = 0;

	// Read 24 bits, MSB first
	for (int i = 0; i < 24; i++) {
		value = value << 1;  // Shift left to make room for next bit
		value |= read_bit(); // Read bit and add to value
	}

	// Check if value is negative (bit 23 is set)
	// If so, extend the sign to fill 32-bit integer
	if (value & 0x800000) {
		value |= 0xFF000000;  // Sign extend from 24-bit to 32-bit
	}

	return value;
}

long HX711::read() {
	// Wait for the chip to become ready
	if (!is_ready()) {
		wait_ready();
	}

	ESP_LOGD(TAG, "Starting HX711 read sequence...");

	// Critical section to prevent interrupts from stretching clock pulses beyond 60us
	// which would cause the HX711 to enter power-down mode
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

	// Read 24 bits of data
	int32_t value = read_24bit();

	// Send gain/channel selection pulses
	// The number of additional pulses sets the gain for the NEXT reading
	for (unsigned int i = 0; i < GAIN; i++) {
		clock_pulse();
	}

	portEXIT_CRITICAL(&mux);

	// Log the results after exiting critical section
	ESP_LOGD(TAG, "Read 24 bits of data: %ld (0x%06lX)", value, value & 0xFFFFFF);
	ESP_LOGD(TAG, "Sent %d gain selection pulse(s)", GAIN);
	ESP_LOGI(TAG, "Read complete, raw value: %ld", value);

	return static_cast<long>(value);
}

void HX711::wait_ready(unsigned long delay_ms) {
	ESP_LOGD(TAG, "Waiting for HX711 to become ready...");

	uint32_t wait_count = 0;
	while (!is_ready()) {
		if (delay_ms > 0) {
			vTaskDelay(pdMS_TO_TICKS(delay_ms));
		} else {
			vTaskDelay(1);  // Minimum delay to feed watchdog
		}
		wait_count++;

		// Log every 100 iterations to avoid spam
		if (wait_count % 100 == 0) {
			ESP_LOGW(TAG, "Still waiting... (%lu iterations)", wait_count);
		}
	}

	ESP_LOGD(TAG, "HX711 ready after %lu iterations", wait_count);
}

bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	ESP_LOGD(TAG, "Waiting for HX711 with retry limit: %d", retries);

	int count = 0;
	while (count < retries) {
		if (is_ready()) {
			ESP_LOGI(TAG, "HX711 ready after %d attempts", count + 1);
			return true;
		}

		if (delay_ms > 0) {
			vTaskDelay(pdMS_TO_TICKS(delay_ms));
		} else {
			vTaskDelay(1);
		}
		count++;
	}

	ESP_LOGW(TAG, "HX711 not ready after %d retries", retries);
	return false;
}

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	ESP_LOGD(TAG, "Waiting for HX711 with timeout: %lu ms", timeout);

	TickType_t startTicks = xTaskGetTickCount();
	TickType_t timeoutTicks = pdMS_TO_TICKS(timeout);

	while ((xTaskGetTickCount() - startTicks) < timeoutTicks) {
		if (is_ready()) {
			uint32_t elapsed = pdTICKS_TO_MS(xTaskGetTickCount() - startTicks);
			ESP_LOGD(TAG, "HX711 ready after %lu ms", elapsed);
			return true;
		}

		if (delay_ms > 0) {
			vTaskDelay(pdMS_TO_TICKS(delay_ms));
		} else {
			vTaskDelay(1);
		}
	}

	ESP_LOGW(TAG, "HX711 timeout after %lu ms", timeout);
	return false;
}

long HX711::read_average(byte times) {
	ESP_LOGD(TAG, "Reading average of %d samples...", times);

	long sum = 0;
	for (byte i = 0; i < times; i++) {
		long reading = read();
		sum += reading;
		ESP_LOGD(TAG, "Sample %d/%d: %ld (running sum: %ld)", i + 1, times, reading, sum);

		// Small delay between reads and feed watchdog
		vTaskDelay(pdMS_TO_TICKS(5));
	}

	long average = sum / times;
	ESP_LOGI(TAG, "Average of %d samples: %ld", times, average);

	return average;
}

double HX711::get_value(byte times) {
	long raw = read_average(times);
	double value = raw - OFFSET;

	ESP_LOGI(TAG, "get_value: raw=%ld, offset=%ld, result=%.2f", raw, OFFSET, value);

	return value;
}

float HX711::get_units(byte times) {
	double value = get_value(times);
	float units = value / SCALE;

	ESP_LOGI(TAG, "get_units: value=%.2f, scale=%.6f, result=%.6f", value, SCALE, units);

	return units;
}

void HX711::tare(byte times) {
	ESP_LOGI(TAG, "Starting tare operation with %d samples...", times);

	long sum = read_average(times);
	set_offset(sum);

	ESP_LOGI(TAG, "Tare complete - offset set to %ld", OFFSET);
}

void HX711::set_scale(float scale) {
	ESP_LOGI(TAG, "Setting scale factor: %.6f (was %.6f)", scale, SCALE);
	SCALE = scale;
}

float HX711::get_scale() {
	return SCALE;
}

void HX711::set_offset(long offset) {
	ESP_LOGI(TAG, "Setting offset: %ld (was %ld)", offset, OFFSET);
	OFFSET = offset;
}

long HX711::get_offset() {
	return OFFSET;
}

void HX711::power_down() {
	ESP_LOGI(TAG, "Powering down HX711...");

	// Set PD_SCK HIGH for more than 60 microseconds to enter power-down mode
	gpio_set_level(PD_SCK, 0);
	esp_rom_delay_us(1);
	gpio_set_level(PD_SCK, 1);
	esp_rom_delay_us(70);  // Hold high for 70us (> 60us required)

	ESP_LOGI(TAG, "HX711 in power-down mode");
}

void HX711::power_up() {
	ESP_LOGI(TAG, "Powering up HX711...");

	// Set PD_SCK LOW to wake up
	gpio_set_level(PD_SCK, 0);

	ESP_LOGI(TAG, "HX711 waking up - waiting for ready state...");

	// Wait for the chip to complete reset and calibration (~400ms typical)
	vTaskDelay(pdMS_TO_TICKS(500));

	// Wait until DOUT goes LOW (data ready)
	wait_ready(10);

	ESP_LOGI(TAG, "HX711 powered up and ready");
}
