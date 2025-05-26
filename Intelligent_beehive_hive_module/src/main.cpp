#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include <OneWire.h>
#include <HX711.h>
#include <Adafruit_MPU6050.h>
#include <DallasTemperature.h>
#include <soc/i2s_reg.h>
#include <DHT.h>
#include <Wire.h>
#include <utility> // Pro std::pair
#include <esp_wifi.h>
#include <esp_sleep.h>

// Parametry
#define SAMPLES      1024
#define SAMPLE_RATE  16000
//#define FILTER_LOW   100         // Dolní hranice pásma (100 Hz)
//#define FILTER_HIGH  600         // Horní hranice pásma (600 Hz)
#define FILTER_LOW   600         // Dolní hranice pásma (100 Hz)
#define FILTER_HIGH  100         // Horní hranice pásma (600 Hz)
#define I2S_PORT     I2S_NUM_0
 
#define I2S_DATA_PIN GPIO_NUM_39 
#define I2S_BCLK_PIN GPIO_NUM_40 
#define I2S_LRCL_PIN GPIO_NUM_38 

#define ONE_WIRE_BUS GPIO_NUM_4 // Pro DS18 teploměr musí bý mezi datovým a VIN 4.7k ohm pullup rezistor 
#define DHTPIN       GPIO_NUM_15
#define DHTTYPE      DHT22

#define HX711_DOUT   GPIO_NUM_12
#define HX711_CLK    GPIO_NUM_13

#define MPU_SCL      GPIO_NUM_8
#define MPU_SDA      GPIO_NUM_9

#define WIFI_CHANNEL 6

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Inicializace FFT
double vReal[SAMPLES];          // Reálná část (vstup pro FFT)
double vImag[SAMPLES];          // Imaginární část (FFT výstup)

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLE_RATE);
float prevInput = 0.0, prevOutput = 0.0;

// Vytvoření instancí OneWire, DallasTemperature DHT, HX711 a MPU6050
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DHT dht(DHTPIN, DHTTYPE);
HX711 scale;
Adafruit_MPU6050 mpu;

// Proměnné pro kalibraci a vážení
float calibration_factor = -18665.0; // Kalibrační faktor, upravte po kalibraci
float known_weight = 3.0; // Známá hmotnost (v kg) použitá při kalibraci
float known_offset = 5.0;

// Zpráva na sběr dat
typedef struct esp_now_message {
  char mac[18];
  float weight;
  float hiveTemp;
  float moduleTemp;
  float moduleHum;
  float frequency;
  float gyroX;
  float gyroY;
  float gyroZ;
  float accelX;
  float accelY;
  float accelZ;
} esp_now_message;

esp_now_message data;

struct MPU6050Data {
    float gyroX;
    float gyroY;
    float gyroZ;
    float accelX;
    float accelY;
    float accelZ;
};

// Deklarace funkcí
void setupI2S();
void setupHX711();
void setupESP_now();
void setupMPU();
float readMicrophoneData();
float highPassFilter(float input, float cutoffFreq, float sampleRate);
float lowPassFilter(float input, float cutoffFreq, float sampleRate);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
float getDS18B20();
std::pair<float, float> getDHT22();
float getWeight();
MPU6050Data getMPUValues();
void esp_now_process();
void calibrateScale();

void setup() {
    Serial.begin(115200);
    // Setup ESP-NOW, Setup I2S a inicializace všech senzorů
    setupESP_now();
    setupI2S();
    setupHX711();
    setupMPU();
    sensors.begin();
    dht.begin();
    esp_sleep_enable_timer_wakeup(5 * 1000000);
    
    delay(2000);
    esp_now_process();
}

void loop() {
    //delay(10000); // Aktualizace každou sekundu
    /*MPU6050Data mpuData = getMPUValues();

    Serial.printf("Gyro [°/s] -> gX: %.2f, gY: %.2f, gZ: %.2f\n", mpuData.gyroX, mpuData.gyroY, mpuData.gyroZ);
    Serial.printf("Accel [m/s²] -> aX: %.2f, aY: %.2f, aZ: %.2f\n", mpuData.accelX, mpuData.accelY, mpuData.accelZ);*/
    
    /*if (Serial.available()) {
        char command = Serial.read();
        if (command == 'c') {
            calibrateScale();
        }
    }

    // Použij nastavený kalibrační faktor
    scale.set_scale(calibration_factor);

    // Získání hmotnosti
    getWeight();

    delay(2000); // Měření každé 2 sekundy*/
    //esp_now_process();

}

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Mistr, přijímač
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Pouze levý kanál
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_LRCL_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATA_PIN};

    esp_err_t err;
    err =i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.println("I2S driver initialization failed");
        while (true);
    }

    // Delay by falling edge
    REG_SET_BIT(I2S_RX_TIMING_REG(I2S_PORT), BIT(1));
    // Force Philips mode
    REG_SET_BIT(I2S_RX_CONF1_REG(I2S_PORT), I2S_RX_MSB_SHIFT);

    err = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (err != ESP_OK) {
        Serial.println("Setting pins failed");
        while (true);
    }
}

void setupESP_now() {
    // Inicializace WiFi ve stavu STA
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_err_t result = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (result == ESP_OK) {
        Serial.printf("WiFi channel set to %d successfully.\n", WIFI_CHANNEL);
    } else {
        Serial.printf("Failed to set WiFi channel. Error code: %d\n", result);
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        delay(1000); // Malá pauza před restartem
        ESP.restart();
    }
    
    delay(2000);
    // Registrace callbacku pro odesílání
    esp_now_register_send_cb(onDataSent);

    // Nastavení broadcast peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void setupHX711() {
    scale.begin(HX711_DOUT, HX711_CLK);
    // Zkontrolujte, zda je HX711 připraven
    //scale.set_scale(); // Reset kalibračního faktoru
    //scale.tare();      // Nulování bez zátěže
    if (!scale.is_ready()) {
        Serial.println("HX711 není připraven. Zkontrolujte připojení.");
        return;
    } else {
        Serial.println("HX711 inicializován.");
        
        scale.set_scale(calibration_factor);
        scale.set_offset(known_offset);
    }
}

void setupMPU() {
    Wire.begin(MPU_SDA, MPU_SCL);
    if (!mpu.begin()) {
        Serial.println("MPU initialization failed!");
        return;
    }
    Serial.println("MPU initialized!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
}

float readMicrophoneData() {
    int numMeasurements = 20;
    float peakFrequencies[numMeasurements];

    for (int i = 0; i < numMeasurements; i++) {
        size_t bytesRead;
        int32_t buffer[SAMPLES];

        esp_err_t err = i2s_read(I2S_PORT, (uint8_t*)buffer, SAMPLES * sizeof(int32_t), &bytesRead, portMAX_DELAY);
        if (err != ESP_OK) {
            Serial.println("Error reading from microphone");
            Serial.println(err);
            continue;
        }

        // Převod dat na 16-bitové hodnoty a aplikace filtrů
        for (int i = 0; i < SAMPLES; i++) {
            float sample = (float)(buffer[i] >> 14); // Převod na 16-bit (odstranění šumu)
            sample = highPassFilter(sample, FILTER_HIGH, SAMPLE_RATE); // High-pass filtr
            sample = lowPassFilter(sample, FILTER_LOW, SAMPLE_RATE);   // Low-pass filtr
            vReal[i] = sample;
            vImag[i] = 0; // Imaginární část je nulová
        }

        // FFT analýza
        FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);

        // Dominantní frekvence
        double peakFreq = FFT.majorPeak(vReal, SAMPLES, SAMPLE_RATE);
        peakFrequencies[i] = peakFreq;

        /*Serial.print("Dominantní frekvence #");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(peakFreq);
        Serial.println(" Hz");*/

        delay(200); // Zpoždění mezi měřeními
    }

    // Vypočítejte průměr
    float avgFreq = 0.0;
    for (int i = 0; i < numMeasurements; i++) {
        avgFreq += peakFrequencies[i];
    }
    avgFreq /= numMeasurements;

    // Odeslání průměrné frekvence přes ESP-NOW
    //sendDataToPeer(avgFreq);

    Serial.print("Průměrná frekvence: ");
    Serial.println(avgFreq);
    return avgFreq;
}

// Implementace high-pass filtru (jednoduchý RC filtr)
float highPassFilter(float input, float cutoffFreq, float sampleRate) {
    float RC = 1.0 / (2.0 * PI * cutoffFreq);
    float alpha = RC / (RC + 1.0 / sampleRate);
    float output = alpha * (prevOutput + input - prevInput);
    prevInput = input;
    prevOutput = output;
    return output;
}

// Implementace low-pass filtru (jednoduchý RC filtr)
float lowPassFilter(float input, float cutoffFreq, float sampleRate) {
    float RC = 1.0 / (2.0 * PI * cutoffFreq);
    float alpha = RC / (RC + 1.0 / sampleRate);
    float output = alpha * input + (1.0 - alpha) * prevOutput;
    prevOutput = output;
    return output;
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if(status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Trying again in 30s");
        delay(30000);
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
    } else {
        delay(1000);
        Serial.println("Entering Deep Sleep");
        esp_deep_sleep_start(); //Enter deep sleep
    }
}

float getDS18B20() {
    sensors.requestTemperatures();  // Pošleme příkaz k teploměru pro čtení teploty
    float temperatureC = sensors.getTempCByIndex(0);    // Získáme teplotu z prvního připojeného teploměru (index 0)
    
    // Pokud teplota není platná, vypíšeme chybu
    if (temperatureC == DEVICE_DISCONNECTED_C) {
        Serial.println("Error: Could not read temperature");
        return -127.00;
    } else {
        // Vypíšeme naměřenou teplotu
        Serial.print("Temperature: ");
        Serial.print(temperatureC);
        Serial.println(" °C");
        return temperatureC;
    }
}

std::pair<float, float> getDHT22() {
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();

    if (isnan(temp) || isnan(hum)) {
        Serial.println("Error reading values from DHT22");
        return std::make_pair(-127.00, -127.00); // Chybové hodnoty
    } else {
        Serial.print("Temp: ");
        Serial.print(temp);
        Serial.print(", Hum: ");
        Serial.println(hum);
    }

    return std::make_pair(temp, hum);
}

float getWeight() {
    // je třeba dodělat kalibraci
    scale.set_scale(calibration_factor);
    float weight = scale.get_units(25); // Průměrování z 10 měření
    weight = weight - known_offset;
    Serial.print("Hmotnost (kg): ");
    Serial.println(weight, 2);
    return weight;
}

MPU6050Data getMPUValues() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Převod gyroskopu na °/s
    const float radToDeg = 180.0 / PI;
    Serial.printf("Gyro [°/s] -> gX: %.2f, gY: %.2f, gZ: %.2f\n", gyro.gyro.x * radToDeg, gyro.gyro.y * radToDeg, gyro.gyro.z * radToDeg);
    Serial.printf("Accel [m/s²] -> aX: %.2f, aY: %.2f, aZ: %.2f\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    Serial.printf("Gyro [°/s] -> gX: %f, gY: %f, gZ: %f\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    Serial.printf("Accel [m/s²] -> aX: %f, aY: %f, aZ: %f\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    return {
        gyro.gyro.x, 
        gyro.gyro.y, 
        gyro.gyro.z,
        accel.acceleration.x, 
        accel.acceleration.y, 
        accel.acceleration.z
    };
}

void esp_now_process() {
    MPU6050Data mpuData = getMPUValues();
    Serial.println("MPU Data získána");
    strncpy(data.mac, WiFi.macAddress().c_str(), sizeof(data.mac) - 1);
    data.mac[17] = '\0';
    data.weight = getWeight();
    Serial.print("Váha: ");
    Serial.println(data.weight);
    Serial.println("Váha Data získána");
    data.hiveTemp = getDS18B20();
    Serial.println("DS Data získána");
    data.moduleTemp = getDHT22().first;
    data.moduleHum = getDHT22().second;
    Serial.println("DHT Data získána");
    data.frequency = readMicrophoneData();
    Serial.println("Mic Data získána");
    data.gyroX = mpuData.gyroX;
    data.gyroY = mpuData.gyroY;
    data.gyroZ = mpuData.gyroZ;
    data.accelX = mpuData.accelX;
    data.accelY = mpuData.accelY;
    data.accelZ = mpuData.accelZ;

    delay(10000);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &data, sizeof(data));
    if (result == ESP_OK) {
        Serial.println("Data send with success");
    } else {
        Serial.println("Error sending the data, trying again");
    }
}

void calibrateScale() {
    Serial.println("Odstraň veškerou zátěž z váhy.");
    delay(5000);

    scale.set_scale(); // reset měřítka
    scale.tare();      // vynuluj váhu
    Serial.println("Nyní polož známou hmotnost a stiskni 'n'.");

    // Čekej na zadání 'n'
    while (Serial.available() == 0 || Serial.read() != 'n') {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nČtu hodnotu...");

    float reading = scale.get_units(20);
    calibration_factor = reading / known_weight;

    Serial.print("Nový kalibrační faktor: ");
    Serial.println(calibration_factor, 4);

    scale.set_scale(calibration_factor);
    Serial.println("Kalibrace uložena a použita.");
}