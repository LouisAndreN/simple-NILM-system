/*
 * NILM System with Full FFT Analysis
 * ESP32 + ADS1115 + SCT-013
 * 
 * Features:
 * - Full FFT (256 samples @ 2kHz = 128 frequency bins)
 * - Differential voltage measurement (A0-A1)
 * - Real-time RMS, power, THD calculation
 * - Serial output in JSON format
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <arduinoFFT.h>

// ===== HARDWARE CONFIG =====
Adafruit_ADS1115 ads;

// ===== FFT CONFIG =====
#define SAMPLES 256              // Must be power of 2
#define SAMPLING_FREQ 2000       // Hz (Nyquist = 1000 Hz max)
#define FFT_BINS (SAMPLES / 2)   // 128 frequency bins

ArduinoFFT<double> FFT = ArduinoFFT<double>();

double vReal[SAMPLES];
double vImag[SAMPLES];

// ===== CALIBRATION =====
float voltage_multiplier = 1.0;  // V/ADC (to be calibrated)
float current_multiplier = 1.0;  // A/ADC (to be calibrated)
float gain_calibrated = 1.0;

// ===== GRID PARAMETERS =====
float grid_frequency = 50.0;     // Hz
float grid_voltage = 100.0;      // V RMS

// ===== TIMING =====
unsigned long last_fft_time = 0;
const unsigned long FFT_INTERVAL = 500;  // FFT every 500ms (2 Hz)





// ===== FFT ACQUISITION & PROCESSING =====
void performFFT() {
    // Acquire samples
    unsigned long start_time = micros();
    for (int i = 0; i < SAMPLES; i++) {
        // Read differential voltage A0-A1
        int16_t adc_value = ads.readADC_Differential_0_1();
        
        // Convert to voltage (mV)
        // ADS1115 @ GAIN_TWO: ±2.048V, 16-bit
        // LSB = 2.048V / 32768 = 0.0625 mV
        float voltage_mv = adc_value * 0.0625;
        
        vReal[i] = voltage_mv;
        vImag[i] = 0;
        
        // Wait for next sample (500 µs = 2 kHz)
        delayMicroseconds(500);
    }
    unsigned long acquisition_time = micros() - start_time;
    
    // Apply window function (Hamming)
    FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    
    // Compute FFT
    FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    
    // Compute magnitudes
    FFT.complexToMagnitude(vReal, vImag, SAMPLES);
    
    // Calculate frequency resolution
    float freq_resolution = (float)SAMPLING_FREQ / SAMPLES;  // 7.8125 Hz
    
    // ===== CALCULATE RMS FROM FFT =====
    // RMS = sqrt(sum of all frequency components squared)
    float sum_squares = 0;
    for (int i = 1; i < FFT_BINS; i++) {  // Skip DC component (i=0)
        sum_squares += vReal[i] * vReal[i];
    }
    float rms_voltage_mv = sqrt(sum_squares / SAMPLES);
    float rms_voltage_v = rms_voltage_mv / 1000.0;
    
    // ===== CALCULATE CURRENT & POWER =====
    float irms = rms_voltage_v * current_multiplier * gain_calibrated;
    float power = irms * grid_voltage;
    
    // ===== CALCULATE THD =====
    // Find fundamental frequency bin
    int fundamental_bin = round(grid_frequency / freq_resolution);
    float fundamental_magnitude = vReal[fundamental_bin];
    
    // Sum harmonics (2nd to 16th)
    float harmonics_sum_sq = 0;
    for (int h = 2; h <= 16; h++) {
        int bin = fundamental_bin * h;
        if (bin < FFT_BINS) {
            harmonics_sum_sq += vReal[bin] * vReal[bin];
        }
    }
    
    float thd = 0;
    if (fundamental_magnitude > 0) {
        thd = sqrt(harmonics_sum_sq) / fundamental_magnitude;
    }
    
    // ===== OUTPUT JSON =====
    Serial.print("{");
    
    // Metadata
    Serial.print("\"timestamp\":");
    Serial.print(millis());
    Serial.print(",\"acquisition_us\":");
    Serial.print(acquisition_time);
    
    // Raw differential voltage (mV)
    Serial.print(",\"v_diff_mv\":");
    Serial.print(rms_voltage_mv, 3);
    
    // Calculated values
    Serial.print(",\"v_rms\":");
    Serial.print(rms_voltage_v, 4);
    Serial.print(",\"i_rms\":");
    Serial.print(irms, 4);
    Serial.print(",\"power\":");
    Serial.print(power, 2);
    Serial.print(",\"thd\":");
    Serial.print(thd, 4);
    
    // Calibration gain
    Serial.print(",\"gain\":");
    Serial.print(gain_calibrated, 4);
    
    // FFT metadata
    Serial.print(",\"fft_bins\":");
    Serial.print(FFT_BINS);
    Serial.print(",\"freq_resolution\":");
    Serial.print(freq_resolution, 4);
    Serial.print(",\"sampling_freq\":");
    Serial.print(SAMPLING_FREQ);
    
    // Full FFT spectrum (magnitudes only)
    Serial.print(",\"fft\":[");
    for (int i = 0; i < FFT_BINS; i++) {
        Serial.print(vReal[i], 4);
        if (i < FFT_BINS - 1) Serial.print(",");
    }
    Serial.print("]");
    
    Serial.println("}");
}

// ===== SERIAL COMMANDS =====
void serialEvent() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command.startsWith("CAL_CURRENT:")) {
            // Set current multiplier
            float val = command.substring(12).toFloat();
            current_multiplier = val;
            Serial.print("{\"status\":\"current_multiplier_set\",\"value\":");
            Serial.print(current_multiplier, 6);
            Serial.println("}");
        }
        else if (command.startsWith("CAL_GAIN:")) {
            // Set calibration gain
            float val = command.substring(9).toFloat();
            gain_calibrated = val;
            Serial.print("{\"status\":\"gain_set\",\"value\":");
            Serial.print(gain_calibrated, 6);
            Serial.println("}");
        }
        else if (command.startsWith("GRID_FREQ:")) {
            // Set grid frequency
            float val = command.substring(10).toFloat();
            grid_frequency = val;
            Serial.print("{\"status\":\"grid_frequency_set\",\"value\":");
            Serial.print(grid_frequency, 2);
            Serial.println("}");
        }
        else if (command.startsWith("GRID_VOLT:")) {
            // Set grid voltage
            float val = command.substring(10).toFloat();
            grid_voltage = val;
            Serial.print("{\"status\":\"grid_voltage_set\",\"value\":");
            Serial.print(grid_voltage, 2);
            Serial.println("}");
        }
        else if (command == "GET_CONFIG") {
            // Return current configuration
            Serial.print("{\"config\":{");
            Serial.print("\"current_multiplier\":");
            Serial.print(current_multiplier, 6);
            Serial.print(",\"gain\":");
            Serial.print(gain_calibrated, 6);
            Serial.print(",\"grid_frequency\":");
            Serial.print(grid_frequency, 2);
            Serial.print(",\"grid_voltage\":");
            Serial.print(grid_voltage, 2);
            Serial.println("}}");
        }
    }
}






// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    
    // Initialize ADS1115
    if (!ads.begin()) {
        Serial.println("{\"error\":\"ADS1115 not found\"}");
        while (1);
    }
    
    // Set gain for ±2.048V range
    ads.setGain(GAIN_TWO);
    
    // Set data rate to maximum (860 SPS)
    ads.setDataRate(RATE_ADS1115_860SPS);
    
    Serial.println("{\"status\":\"ready\"}");
    delay(1000);
}

// ===== MAIN LOOP =====
void loop() {
    if (millis() - last_fft_time >= FFT_INTERVAL) {
        last_fft_time = millis();
        
        // Perform FFT analysis
        performFFT();
    }
}
