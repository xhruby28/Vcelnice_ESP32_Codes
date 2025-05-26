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
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <DHT.h>


#define CS GPIO_NUM_34

#define SERVICE_UUID                "f6bb2d8d-6ce2-4847-9d0e-47344f9ce34f"
#define NOTIFY_CHARACTERISTIC_UUID  "d898d3cd-d161-4876-8858-c07024ab5136"
#define WRITE_CHARACTERISTIC_UUID   "107c0a9c-df0a-423d-9f7c-faf0e5107408"

#define WIFI_SSID "ApiaryConnect"
#define WIFI_PASSWORD "6m=JaFt0P=J0f!z}Sq69"
#define WIFI_PORT 80
#define WIFI_CHANNEL 6

// SIM
HardwareSerial sim800l(1);
#define TX_PIN 43
#define RX_PIN 44

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

// DHT22
#define DHTPIN  GPIO_NUM_6         // Pin, kde je DHT22 připojen
#define DHTTYPE DHT22     // Typ senzoru
DHT dht(DHTPIN, DHTTYPE);

TaskHandle_t smsTaskHandle = NULL;
volatile bool sendSmsFlag = false;
float tempVal = 0;
float humVal = 0;
char phoneNumber[20] = {0};

const char *LAST_SMS_FILE = "/last_sms.txt";
time_t lastSmsTime = 0;          // Poslední čas odeslání SMS
const unsigned long SMS_INTERVAL = 2 * 3600; // 2 hodiny v sekundách

// FreeRTOS Handlery
TaskHandle_t WiFiTaskHandle = nullptr;

bool syncInProgress = false;
bool dataRecAndSyncActive = false;
bool shouldStopWiFi = false;
bool isSim = false;
bool telSet = false;
bool pinSet = false;

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

File serverFile;

// Deklarace funkcí
void startWiFi();
void stopWiFi();
void wifiTask(void *param);
void sendSmsTask(void *pvParameters);
void setupRTC();
void setupBLE();
void initSIM800L();
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void setupESPNow();
void WriteSD(esp_now_message *incomingData, const char * path);
void createFile(const char * path);
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);
void deleteFile(fs::FS &fs, const char*);
String readFile(fs::FS &fs, const char * path);
long getTime();
void mergeTempToData();
void sendCommand(String command);
String xorEncrypt(const String &input, const String &key);
void saveLastSmsTime(time_t t);
time_t loadLastSmsTime();


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
        } else if(value == "SEND_TEL") {
            telSet = false;
            pinSet = false;
            notifyCharacteristic->setValue("SEND_TEL_ACK");
            notifyCharacteristic->notify();
            Serial.println("[TEL] Ready to recieve telephone information");
        } else if (value == "SYNC_COMPLETE") {
            telSet = false;
            pinSet = false;
            if(serverFile){
                Serial.println("Closing data.csv, HTTP GET successful");
                serverFile.close();
            }
            syncInProgress = false;
            notifyCharacteristic->setValue("SYNC_COMPLETE_ACK");
            notifyCharacteristic->notify();
            Serial.println("SYNC_COMPLETE recieved.");
            if(dataRecAndSyncActive){
                mergeTempToData();
                dataRecAndSyncActive = false;
            }

            xTaskCreatePinnedToCore(
            [](void *param) {
                delay(1000);
                stopWiFi();
                vTaskDelete(NULL);
            },
            "DelayedStopWiFi",
            2048,
            NULL,
            1,
            NULL,
            1);
            
            /*BLEDevice::deinit();
            setupBLE();
            stopWiFi();*/ 
        } else if(value == "TEL:NULL"){
            Serial.println("Získano žádné tel. číslo");
            Serial.println("Zapisuji vypnutí SIM");

            EEPROM.write(1, 0);
            EEPROM.commit();

            notifyCharacteristic->setValue("PIN_ACK");
            notifyCharacteristic->notify();

        }else if (value.rfind("TEL:", 0) == 0) {
            String tel = value.substr(4).c_str();
            tel.trim();

            
            EEPROM.write(1, 1);
            EEPROM.commit();

            // Vytvoření tasky pro zápis telefonního čísla
            if (tel != "NULL") {
                xTaskCreatePinnedToCore(
                    [](void *param) {
                        String *telStr = (String *)param;
                        if(!telSet){
                            telSet = true;
                            createFile("/tel.txt"); // vytvořit soubor, pokud neexistuje
                            File telFile = SD.open("/tel.txt", FILE_READ);
                            String currentTel = "";
                            if (telFile) {
                                currentTel = telFile.readString();
                                telFile.close();
                                currentTel.trim();
                            }

                            // Pokud telefonní číslo není NULL a je jiné, uložíme nové
                            if (*telStr != "NULL" && *telStr != currentTel) {
                                writeFile(SD, "/tel.txt", telStr->c_str());
                                Serial.print("[TEL] Telefonní číslo uloženo: ");
                                Serial.println(*telStr);
                                sim800l.println(""); delay(100);
                                sim800l.println("AT"); delay(100);
                                sim800l.println("AT+CSCLK=0"); // probuzení SIM800L
                                Serial.println("[SIM] SIM800L probuzen.");
                            } else {
                                Serial.println("[TEL] Telefonní číslo je stejné, neukládá se.");
                            }
                            // Odeslat zpět ack
                            notifyCharacteristic->setValue("TEL_ACK");
                            notifyCharacteristic->notify();
                        }
                        delete telStr; // uvolnit paměť
                        vTaskDelete(NULL); // Ukončit task
                        
                    },
                    "TelWriter",
                    4096,
                    new String(tel),
                    1,
                    NULL,
                    1 // Core 1
                );
            }
        } else if (value.rfind("PIN:", 0) == 0) {
            String pin = value.substr(4).c_str();
            pin.trim();

            // Vytvoření tasky pro zápis PINu
            xTaskCreatePinnedToCore(
                [](void *param) {
                    String *pinStr = (String *)param;
                    if(!pinSet){
                        pinSet = true;
                        createFile("/pin.txt"); // vytvořit soubor, pokud neexistuje
                        File pinFile = SD.open("/pin.txt", FILE_READ);
                        String currentEncrypted = "";
                        if (pinFile) {
                            currentEncrypted = pinFile.readString();
                            pinFile.close();
                            currentEncrypted.trim();
                        }

                        String newEncrypted = xorEncrypt(*pinStr, "asdf1560sdfa23d");
                        String encryptedPin = readFile(SD, "/pin.txt");
                        String decryptedPin = xorEncrypt(encryptedPin, "asdf1560sdfa23d");
                        Serial.print("Dešifrovaný pin: ");
                        Serial.println(decryptedPin);

                        // Pokud je PIN jiný než aktuální, uložíme nový
                        if (*pinStr != "NULL" && newEncrypted != currentEncrypted) {
                            writeFile(SD, "/pin.txt", newEncrypted.c_str());
                            Serial.println("[PIN] PIN byl zašifrován a uložen.");
                        } else if (*pinStr == "NULL") {
                            writeFile(SD, "/pin.txt", "NULL");
                            Serial.println("[PIN] PIN nastaven na NULL.");
                        } else {
                            Serial.println("[PIN] PIN je stejný, neukládá se.");
                        }

                        // Odeslat zpět ack
                        notifyCharacteristic->setValue("PIN_ACK");
                        notifyCharacteristic->notify();
                    }
                    delete pinStr; // uvolnit paměť
                    vTaskDelete(NULL); // Ukončit task
                },
                "PinWriter",
                4096,
                new String(pin),
                1,
                NULL,
                1 // Core 1
            );
        }
    }
};

// Setup funkce
void setup() {
    Serial.begin(115200);

    sim800l.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    initSIM800L();

    //sendSMS("+420724856009", "Pozdrav z SmartHive"); // Pošli SMS
    delay(5000);
    Wire.begin();
    dht.begin();

    if (!SD.begin(CS)) {
        Serial.println("Chyba při inicializaci SD karty");
    } else {
        Serial.println("SD karta inicializována");
        File file = SD.open("/tel.txt");
        if (file) {
            size_t len = file.readBytes(phoneNumber, sizeof(phoneNumber) - 1);
            phoneNumber[len] = 0;
            file.close();
            Serial.print("Telefonní číslo: ");
            Serial.println(phoneNumber);
        } else {
            Serial.println("Nepodařilo se otevřít tel.txt");
        }

        lastSmsTime = loadLastSmsTime();
        Serial.print("Poslední SMS odeslána v: ");
        Serial.println((uint32_t)lastSmsTime);
    }
    //Inicializace souboru data.csv
    createFile("/data.csv");
    delay(2000);
    createFile("/last_sms.txt");

    // Inicializace RTC, ESP-NOW, BLE
    setupRTC();
    setupESPNow();
    setupBLE();

    // Vytvoření úloh pro zpracování dat
    xTaskCreatePinnedToCore(wifiTask, "DataProcessing", 4096, NULL, 2, &WiFiTaskHandle, 1);
    xTaskCreatePinnedToCore(sendSmsTask, "SendSmsTask", 4096, NULL, 1, &smsTaskHandle, 1);


}

void loop() {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (EEPROM.read(1) == 1) {

        tempVal = dht.readTemperature();
        humVal = dht.readHumidity();

        if (isnan(tempVal) || isnan(humVal)) {
            Serial.println("Chyba čtení z DHT22");
        } else {
            Serial.print("T: ");
            Serial.print(tempVal);
            Serial.print(" °C, H: ");
            Serial.print(humVal);
            Serial.println(" %");

            if (EEPROM.read(1) == 1){
                Serial.println("Kontrola času k poslání SMS");
                time_t now = time(NULL);
                bool canSend = (now > 0) && (now - lastSmsTime >= SMS_INTERVAL);

                if ((tempVal > 70 || humVal > 80) && canSend) {
                    sendSmsFlag = true;
                }
            } else {
                Serial.println("Není nastavno posílání SMS");
            }
        }

        delay(5000);
        }
    }

// Inicializace RTC pro DS3231
void setupRTC() {
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
    } else {
        EEPROM.begin(512);
        Serial.println("Serial connected. Syncing time...");
        //EEPROM.write(0, 0); // Uložíme flag, že čas byl nastaven
        //EEPROM.commit(); // Uložíme změny do EEPROM
        // Zkontrolujeme, jestli byl čas již nastaven
        if (EEPROM.read(0) == 0) {
            // Pokud čas ještě nebyl nastaven (flag = 0)
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Nastavení času z kompilace
            EEPROM.write(0, 1); // Uložíme flag, že čas byl nastaven
            EEPROM.commit(); // Uložíme změny do EEPROM
            Serial.println("Time set to compile time.");
        } else {
            // Pokud čas již byl nastaven, používáme aktuální čas RTC   
            Serial.println("Time is already set.");
        }
        Serial.println("RTC synced with compile time.");
        DateTime now = rtc.now();
        struct timeval tv;
        tv.tv_sec = now.unixtime();
        tv.tv_usec = 0;
        settimeofday(&tv, nullptr);
    }
}

// Inicializace BLE
void setupBLE() {
    BLEDevice::init("ApiaryConnect Core");
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

void initSIM800L() {
    Serial.println("Inicializace SIM800L...");
    if (EEPROM.read(1) == 1){
        delay(10000);
        sendCommand("AT");

        bool simDetected = false;
        sendCommand("AT+CSMINS?");
        
        while (sim800l.available()) {
            String response = sim800l.readStringUntil('\n');
            response.trim();
            Serial.println(response);

            if (response.indexOf("+CSMINS: 0,1") >= 0) {
            simDetected = true;
            Serial.println("SIM karta je zastrčená a připravená.");
            } else if (response.indexOf("+CSMINS: 0,0") >= 0) {
            Serial.println("SIM karta není vložená nebo není rozpoznaná.");
            }
        }
        if (simDetected) {
            sendCommand("AT+CPIN?");

            String response = "";
            long timeout = millis() + 2000;
            while (millis() < timeout && sim800l.available()) {
                response += (char)sim800l.read();
            }

            if (response.indexOf("SIM PIN") != -1) {
                Serial.println("[SIM] Vyžaduje PIN.");
                String encryptedPin = readFile(SD, "/pin.txt");
                if (encryptedPin == "" || encryptedPin == "NULL") {
                    Serial.println("[SIM] PIN nenalezen nebo NULL.");
                    return;
                }

                String decryptedPin = xorEncrypt(encryptedPin, "asdf1560sdfa23d");
                sendCommand("AT+CPIN=\"" + decryptedPin + "\"");
                delay(500);
                Serial.println("[SIM] PIN odeslán.");
            } else {
                Serial.println("[SIM] PIN není potřeba nebo SIM není připojena.");
            }

            sendCommand("AT+CMGF=1"); // Přepnout na textový režim SMS
        } else {
            sim800l.println("AT+CSCLK=2");
        }
    } else {
        Serial.println("[SIM] není připravena");
        sim800l.println("AT+CSCLK=2");
    }
}

//##################################################
//##############   ESP-NOW Functions   #############
//##################################################

void sendCommand(String command) {
    sim800l.println(command);
    delay(500);
}

void stopWiFi() {
    server.end();
    shouldStopWiFi = true;  // Signal WiFi task to stop
    xTaskNotify(WiFiTaskHandle, 0, eNoAction);
    vTaskDelete(WiFiTaskHandle);
    Serial.println("WiFi AP stopped");
    WiFi.softAPdisconnect(true);
    esp_restart();
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

        serverFile = SD.open("/data.csv");
        if (!serverFile) {
            Serial.println("File not found.");
            request->send(404, "text/plain", "File not found");
            return;
        }

        Serial.println("Posting data");
        size_t fileSize = serverFile.size();
        Serial.print("File size: ");
        Serial.println(fileSize);

        AsyncWebServerResponse *response = request->beginResponse(serverFile, "data.csv", "text/csv");
        response->addHeader("Content-Length", String(fileSize));

        request->send(response);
        Serial.println("Data sent successfully.");
    });

    // Spuštění serveru
    server.begin();
    Serial.println("HTTP server started");
}

void wifiTask(void *param) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (shouldStopWiFi) {
            Serial.println("Stopping WiFi task.");
            vTaskDelete(NULL);  // Ukončení tasku
        }

        startWiFi();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sendSmsTask(void *pvParameters) {
  for (;;) {
    if (sendSmsFlag) {
      sendSmsFlag = false;

      sim800l.println("AT+CMGF=1");
      delay(100);
      sim800l.print("AT+CMGS=\"");
      sim800l.print(phoneNumber);
      sim800l.println("\"");
      delay(100);

      String msg = "[WARNING] High AC_Core Temp=" + String(tempVal) + "C Hum=" + String(humVal) + "%";
      sim800l.print(msg);
      sim800l.write(26); // Ctrl+Z
      delay(5000);

      time_t now = time(NULL);
      saveLastSmsTime(now);
      lastSmsTime = now;
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Funkce pro uložení času poslední SMS na SD kartu
void saveLastSmsTime(time_t t) {
  File f = SD.open(LAST_SMS_FILE, FILE_WRITE);
  if (f) {
    f.seek(0); // začátek souboru
    f.print((uint32_t)t);
    f.close();
  }
}

// Funkce pro načtení času poslední SMS z SD karty
time_t loadLastSmsTime() {
  if (!SD.exists(LAST_SMS_FILE)) return 0;

  File f = SD.open(LAST_SMS_FILE);
  if (f) {
    String data = f.readStringUntil('\n');
    f.close();
    return (time_t)data.toInt();
  }
  return 0;
}

void WriteSD(esp_now_message *incomingData, const char * path){
    String datamsg = String(incomingData->mac) + ","
                   + String(getTime()) + ","
                   + String(incomingData->weight, 2) + ","
                   + String(incomingData->hiveTemp, 2) + ","
                   + String(incomingData->moduleHum, 2) + ","
                   + String(incomingData->moduleTemp, 2) + ","
                   + String(incomingData->frequency, 2) + ","
                   + String(incomingData->gyroX, 2) + ","
                   + String(incomingData->gyroY, 2) + ","
                   + String(incomingData->gyroZ, 2) + ","
                   + String(incomingData->accelX, 2) + ","
                   + String(incomingData->accelY, 2) + ","
                   + String(incomingData->accelZ, 2);
    Serial.println(datamsg);
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
    DateTime now = rtc.now();
    Serial.printf("%d.%d.%d %d:%d:%d\n",now.day(),now.month(),now.year(),now.hour(),now.minute(),now.second());
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
    Serial.printf("Creating %s\n",path);
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

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

String readFile(fs::FS &fs, const char * path) {
    File file = fs.open(path, FILE_READ);
    if (!file || file.isDirectory()) return "";
    String content = file.readString();
    file.close();
    return content;
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

String xorEncrypt(const String &input, const String &key) {
    String output = input;
    for (int i = 0; i < input.length(); i++) {
        output[i] = input[i] ^ key[i % key.length()];
    }
    return output;
}