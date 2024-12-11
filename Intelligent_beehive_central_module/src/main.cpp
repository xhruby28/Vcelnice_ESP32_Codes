#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <RTClib.h>
#include <UMS3.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_bt_main.h>

#define CS GPIO_NUM_34

#define SERVICE_UUID                "f6bb2d8d-6ce2-4847-9d0e-47344f9ce34f"
#define NOTIFY_CHARACTERISTIC_UUID  "d898d3cd-d161-4876-8858-c07024ab5136"
#define WRITE_CHARACTERISTIC_UUID   "107c0a9c-df0a-423d-9f7c-faf0e5107408"

#define WIFI_SSID "Hive_Gateway"
#define WIFI_PASSWORD "6m=JaFt0P=J0f!z}Sq69"
#define WIFI_PORT 80
#define WIFI_CHANNEL 6

// BLE proměnné
BLEServer *bleServer = nullptr;
BLECharacteristic *notifyCharacteristic = nullptr;
BLECharacteristic *writeCharacteristic = nullptr;

// RTC pro získání času
RTC_DS3231 rtc;

// HTTP server na portu 80
AsyncWebServer server(80);
IPAddress localIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 1, 1); 
IPAddress subnet(255, 255, 255, 0);

// FreeRTOS Handlery
TaskHandle_t WiFiTaskHandle = nullptr;

bool syncInProgress = false;
bool dataRecAndSyncActive = false;

// Logovací makra
#define LOG_DEBUG(msg) Serial.println(String("[DEBUG] ") + msg)
#define LOG_INFO(msg) Serial.println(String("[INFO] ") + msg)
#define LOG_ERROR(msg) Serial.println(String("[ERROR] ") + msg)

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

esp_now_message receivedData;

// Deklarace funkcí
void startWiFi();
void stopWiFi();
void wifiTask(void *param);
void setupRTC();
void setupBLE();
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void setupESPNow();
void WriteSD(esp_now_message *incomingData, const char * path);
void createFile(const char * path);
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);
long getTime();
void mergeTempToData();

// BLE callbacky
class MyBLEServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        Serial.println("Device connected");
    }

    void onDisconnect(BLEServer *pServer) {
        Serial.println("Device disconnected");
        stopWiFi();
    }
};

class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        Serial.print("Received data: ");
        Serial.println(value.c_str());
        if (value == "SYNC") {
            xTaskNotify(WiFiTaskHandle, 0, eNoAction);
            syncInProgress = true;
            Serial.println("SYNC received.");
            //notifyCharacteristic->setValue("WIFI_STARTED");
            //notifyCharacteristic->notify();
        } else if (value == "SYNC_COMPLETE") {
            syncInProgress = false;
            notifyCharacteristic->setValue("SYNC_COMPLETE_ACK");
            notifyCharacteristic->notify();
            Serial.println("SYNC_COMPLETE recieved.");
            if(dataRecAndSyncActive){
                mergeTempToData();
                dataRecAndSyncActive = false;
            }

            xTaskCreate(
            [](void *param) {
                delay(1000);
                stopWiFi();
                vTaskDelete(NULL);
            },
            "DelayedStopWiFi",
            2048,
            NULL,
            1,
            NULL
            );
            
            /*BLEDevice::deinit();
            setupBLE();
            stopWiFi();*/
        }
    }
};

// Setup funkce
void setup() {
    Serial.begin(115200);

    Wire.begin();
    SD.begin(CS);

    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
    }
    //Inicializace souboru data.csv
    createFile("/data.csv");

    // Inicializace RTC, ESP-NOW, BLE
    setupRTC();
    setupESPNow();
    setupBLE();

    // Vytvoření úloh pro zpracování dat
    xTaskCreate(wifiTask, "DataProcessing", 4096, NULL, 1, &WiFiTaskHandle);
}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void startWiFi() {
    // Inicializace WiFi v režimu AP
    if (WiFi.softAP(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("WiFi AP started successfully");
        Serial.print("SSID: ");
        Serial.println(WIFI_SSID);
        WiFi.softAPConfig(localIP,gateway,subnet);
        Serial.print("IP Address: ");
        Serial.println(WiFi.softAPIP());

        // Odeslání notifikace přes BLE
        notifyCharacteristic->setValue("WIFI_STARTED");
        notifyCharacteristic->notify();
    } else {
        Serial.println("Failed to start WiFi AP");
        notifyCharacteristic->setValue("WIFI_FAILED");
        notifyCharacteristic->notify();
    }

    // Nastavení HTTP GET endpointu pro stahování dat
    server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (!SD.begin(CS)) {
            request->send(500, "text/plain", "SD Card initialization failed");
            Serial.println("SD Card initialization failed.");
            return;
        }

        File file = SD.open("/data.csv");
        if (!file) {
            Serial.println("File not found.");
            request->send(404, "text/plain", "File not found");
            return;
        }

        Serial.println("Posting data");
        AsyncWebServerResponse *response = request->beginResponse(file, "data.csv", "text/csv");
        request->send(response);
        file.close();
        Serial.println("Data sent successfully.");
    });

    // Spuštění serveru
    server.begin();
    Serial.println("HTTP server started");
}

void wifiTask(void *param) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        startWiFi();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void stopWiFi() {
    server.end();
    Serial.println("WiFi AP stopped");
    WiFi.softAPdisconnect(true);
    esp_restart();
}

// Inicializace RTC pro DS3231
void setupRTC() {
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        while (1);
    } else {
        Serial.println("Serial connected. Syncing time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Nastavení času z kompilace
        Serial.println("RTC synced with compile time.");
    }
}

// Inicializace BLE
void setupBLE() {
    BLEDevice::init("Hive Gateway");
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new MyBLEServerCallbacks());

    BLEService *bleService = bleServer->createService(SERVICE_UUID);
    writeCharacteristic = bleService->createCharacteristic(
        WRITE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    writeCharacteristic->setCallbacks(new MyBLECharacteristicCallbacks());

    notifyCharacteristic = bleService->createCharacteristic(
        NOTIFY_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    notifyCharacteristic->addDescriptor(new BLE2902());
    bleService->start();

    bleServer->getAdvertising()->start();
    Serial.println("BLE server started");
}

//##################################################
//##############   ESP-NOW Functions   #############
//##################################################

void WriteSD(esp_now_message *incomingData, const char * path){
    String datamsg = String(incomingData->mac) + ","
                   + String(getTime()) + ","
                   + String(incomingData->weight) + ","
                   + String(incomingData->hiveTemp) + ","
                   + String(incomingData->moduleTemp) + ","
                   + String(incomingData->moduleHum) + ","
                   + String(incomingData->frequency) + ","
                   + String(incomingData->gyroX) + ","
                   + String(incomingData->gyroY) + ","
                   + String(incomingData->gyroZ) + ","
                   + String(incomingData->accelX) + ","
                   + String(incomingData->accelY) + ","
                   + String(incomingData->accelZ);
    appendFile(SD,path,datamsg.c_str());
}

void setupESPNow(){
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect();
    esp_err_t result = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (result == ESP_OK) {
        Serial.printf("WiFi channel set to %d successfully.\n", WIFI_CHANNEL);
    } else {
        Serial.printf("Failed to set WiFi channel. Error code: %d\n", result);
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed");
        return;
        //delay(1000); // Malá pauza před restartem
        //ESP.restart();
    }

    delay(2000);
    esp_now_register_recv_cb(OnDataRecv);  // Registrování callbacku pro příjem dat
    Serial.println("ESP-NOW initialized and callback registered");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    char macStr[18];
    Serial.print("Packet from: ");
    snprintf(macStr, sizeof(macStr),"%02x:%02x:%02x:%02x:%02x:%02x",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(macStr);
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    if(!syncInProgress) {
        WriteSD(&receivedData, "/data.csv");
        Serial.printf("%s, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
                        receivedData.mac, 
                        receivedData.weight, 
                        receivedData.hiveTemp, 
                        receivedData.moduleTemp, 
                        receivedData.moduleHum, 
                        receivedData.frequency,
                        receivedData.gyroX, 
                        receivedData.gyroY, 
                        receivedData.gyroZ,
                        receivedData.accelX, 
                        receivedData.accelY, 
                        receivedData.accelZ);
    } else {
        dataRecAndSyncActive = true;
        createFile("/temp.csv");
        delay(1000);
        WriteSD(&receivedData, "/temp.csv");
    }
}

void createFile(const char * path){
  File file = SD.open(path);

  if (!file){
    Serial.printf("Creating %s",path);
    writeFile(SD,path,"");
  } else{
    Serial.printf("File %s alredy exists\n",path);
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.println(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

long getTime(){
    DateTime now = rtc.now();
    return now.unixtime();
}

void mergeTempToData() {
    File tempFile = SD.open("/temp.csv", FILE_READ);
    File dataFile = SD.open("/data.csv", FILE_APPEND);

    if (tempFile && dataFile) {
        while (tempFile.available()) {
            String line = tempFile.readStringUntil('\n');  // Načte řádek z temp.csv
            dataFile.println(line);  // Zapíše řádek do data.csv
        }
        tempFile.close();
        dataFile.close();

        // Smaže temp.csv
        if (SD.remove("/temp.csv")) {
            Serial.println("temp.csv merged and deleted.");
        } else {
            Serial.println("Failed to delete temp.csv");
        }
    } else {
        Serial.println("Error opening files for merging.");
    }
}