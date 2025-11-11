#pragma once

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <stdint.h>

// ESP-IDF type definitions to replace Arduino types
typedef uint8_t byte;

class HX711
{
private:
	static const char* TAG;

	gpio_num_t PD_SCK;      // Power Down and Serial Clock Input Pin
	gpio_num_t DOUT;        // Serial Data Output Pin
	byte GAIN;              // Gain setting (1=128/A, 2=32/B, 3=64/A)
	long OFFSET;            // Used for tare weight
	float SCALE;            // Used to return weight in grams, kg, ounces, whatever
	uint32_t read_interval_us; // Minimum microseconds between reads (based on sample rate)

	/**
	 * Read a single bit from DOUT on the rising edge of PD_SCK
	 */
	uint8_t read_bit();

	/**
	 * Send a clock pulse (LOW->HIGH->LOW)
	 */
	void clock_pulse();

	/**
	 * Read 24 bits of data from HX711 (MSB first)
	 */
	int32_t read_24bit();

public:
	HX711();
	virtual ~HX711();

	/**
	 * Initialize library with data output pin, clock input pin and gain factor.
	 * Channel selection is made by passing the appropriate gain:
	 * - With a gain factor of 64 or 128, channel A is selected
	 * - With a gain factor of 32, channel B is selected
	 * The library default is "128" (Channel A).
	 *
	 * @param dout Data output pin from HX711
	 * @param pd_sck Clock pin to HX711
	 * @param gain Gain factor (32, 64, or 128)
	 */
	void begin(gpio_num_t dout, gpio_num_t pd_sck, byte gain = 128);

	/**
	 * Check if HX711 is ready for reading.
	 * From the datasheet: When output data is not ready for retrieval, digital output
	 * pin DOUT is high. When DOUT goes to low, it indicates data is ready for retrieval.
	 *
	 * @return true if HX711 has data ready (DOUT is LOW)
	 */
	bool is_ready();

	/**
	 * Wait for the HX711 to become ready (blocking)
	 *
	 * @param delay_ms Delay between checks in milliseconds
	 */
	void wait_ready(unsigned long delay_ms = 0);

	/**
	 * Wait for HX711 to be ready with retry limit
	 *
	 * @param retries Maximum number of retries
	 * @param delay_ms Delay between retries in milliseconds
	 * @return true if ready, false if timed out
	 */
	bool wait_ready_retry(int retries = 3, unsigned long delay_ms = 0);

	/**
	 * Wait for HX711 to be ready with timeout
	 *
	 * @param timeout Maximum time to wait in milliseconds
	 * @param delay_ms Delay between checks in milliseconds
	 * @return true if ready, false if timed out
	 */
	bool wait_ready_timeout(unsigned long timeout = 1000, unsigned long delay_ms = 0);

	/**
	 * Set the gain factor; takes effect only after a call to read()
	 * Channel A can be set for a 128 or 64 gain; channel B has a fixed 32 gain
	 * Depending on the parameter, the channel is also set to either A or B
	 *
	 * @param gain Gain factor (32, 64, or 128)
	 */
	void set_gain(byte gain = 128);

	/**
	 * Set the sample rate (determines minimum time between reads)
	 * HX711 supports 10 Hz or 80 Hz based on RATE pin (hardware setting)
	 * This setting helps avoid reading too frequently
	 *
	 * @param rate_hz Sample rate in Hz
	 */
	void set_sample_rate(uint32_t rate_hz = 10);

	/**
	 * Waits for the chip to be ready and returns a reading.
	 * Reads 24 bits of data and sends gain/channel selection pulses.
	 *
	 * @return 32-bit signed integer value (24-bit value sign-extended)
	 */
	long read();

	/**
	 * Returns an average reading
	 *
	 * @param times How many times to read and average
	 * @return Average of multiple readings
	 */
	long read_average(byte times = 10);

	/**
	 * Returns (read_average() - OFFSET), that is the current value without the tare weight
	 *
	 * @param times How many readings to average
	 * @return Value with offset removed
	 */
	double get_value(byte times = 1);

	/**
	 * Returns get_value() divided by SCALE, that is the raw value divided by
	 * a value obtained via calibration
	 *
	 * @param times How many readings to average
	 * @return Calibrated value in desired units
	 */
	float get_units(byte times = 1);

	/**
	 * Set the OFFSET value for tare weight
	 *
	 * @param times How many times to read the tare value (for averaging)
	 */
	void tare(byte times = 10);

	/**
	 * Set the SCALE value; this value is used to convert the raw data
	 * to "human readable" data (measurement units)
	 *
	 * @param scale Scale factor
	 */
	void set_scale(float scale = 1.f);

	/**
	 * Get the current SCALE
	 *
	 * @return Current scale factor
	 */
	float get_scale();

	/**
	 * Set OFFSET, the value that's subtracted from the actual reading (tare weight)
	 *
	 * @param offset Offset value
	 */
	void set_offset(long offset = 0);

	/**
	 * Get the current OFFSET
	 *
	 * @return Current offset value
	 */
	long get_offset();

	/**
	 * Puts the chip into power down mode
	 * Holds PD_SCK HIGH for >60 microseconds
	 */
	void power_down();

	/**
	 * Wakes up the chip after power down mode
	 * Sets PD_SCK LOW and waits for chip to be ready
	 */
	void power_up();
};
