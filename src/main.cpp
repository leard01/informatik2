/**
 * @file main.cpp
 * @brief Smart Building Automation System für ESP32
 * @details Umfassendes Smart Building System mit Bewegungserkennung, Umweltsensoren, RFID-Zugang,
 *          Webinterface und MQTT-Kommunikation. Das System verwaltet Türschloss, Beleuchtung,
 *          Alarm und Sensorüberwachung.
 * 
 * @author Leard
 * @author David
 * @date 09.07.2025
 * @version 1.0
 * 
 * @cite Für die Grundlegenden Arbeiten wurden Vorlesungsfolien oä. von Alexander Roloff benutzt.
 * @cite Für Erklerungen und Fehlerbehebungen wurde mit ChatGPT zusammengearbeitet.
 * @cite Für weitere Erklärungen wurde https://esp32io.com/ benutzt.
 * 
 * Hardware-Komponenten:
 * - ESP32 Mikrocontroller
 * - DHT11 Temperatur/Luftfeuchtigkeit Sensor
 * - PIR Bewegungssensor
 * - RFID-RC522 Leser
 * - Servo Motor für Türschloss
 * - NeoPixel RGB LED Strip
 * - 16x2 LCD Display
 * - Gas-Sensor MQ-2
 * - Regensensor
 * - Buzzer und LED für Alarme
 * - Taster für Navigation
 * 
 * Funktionen:
 * - Bewegungserkennung mit automatischer Beleuchtung
 * - RFID-basierter Türzugang
 * - Umweltüberwachung (Temperatur, Luftfeuchtigkeit, Regen, Gas)
 * - Webinterface mit Echtzeitdaten
 * - MQTT-Integration für ThingSpeak
 * - Alarm-System bei Gasdetektion
 * - Offline-Betrieb mit lokalem Menü
 * 
 * @warning Alle Pin-Definitionen müssen vor Verwendung überprüft werden
 * @note Verwendet Arduino Framework auf ESP32
 */

#include <Arduino.h>
#include <WiFi.h>
#include <DHTesp.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <time.h>
#include <ESP32Servo.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <MFRC522_I2C.h>
#include <Wire.h>
#include <ESP32PWM.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>

void sendSensorDataWS();
void setupWebServer();

/**
 * @class ISmaBui_IO
 * @brief Interface für alle Smart Building I/O-Komponenten
 * @details Definiert die grundlegenden Methoden für alle Hardware-Komponenten
 *          des Smart Building Systems. Implementiert das Template Method Pattern
 *          für einheitliche Initialisierung und Verarbeitung.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
class ISmaBui_IO {
public:
    /**
     * @brief Initialisiert die Hardware-Komponente
     * @details Wird einmal beim Systemstart aufgerufen
     */
    virtual void begin() = 0;
    
    /**
     * @brief Verarbeitet die Komponente in der Hauptschleife
     * @details Wird zyklisch aufgerufen für kontinuierliche Verarbeitung
     */
    virtual void handle() = 0;
};

class MotionSensorAutomation;

/**
 * @class MotionState
 * @brief Abstrakte Basisklasse für den State Pattern der Bewegungserkennung
 * @details Implementiert das State Pattern für die Bewegungssensor-Steuerung.
 *          Definiert die Schnittstelle für verschiedene Bewegungszustände.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
class MotionState {
public:
    virtual ~MotionState() {}
    
    /**
     * @brief Wird aufgerufen wenn Bewegung erkannt wird
     * @param context Zeiger auf die MotionSensorAutomation Instanz
     */
    virtual void onMotionDetected(MotionSensorAutomation* context) = 0;
    
    /**
     * @brief Wird aufgerufen wenn Timeout erreicht wird
     * @param context Zeiger auf die MotionSensorAutomation Instanz
     */
    virtual void onTimeout(MotionSensorAutomation* context) = 0;
    
    /**
     * @brief Aktualisiert den Zustand in der Hauptschleife
     * @param context Zeiger auf die MotionSensorAutomation Instanz
     */
    virtual void update(MotionSensorAutomation* context) = 0;
    
    /**
     * @brief Gibt den Namen des aktuellen Zustands zurück
     * @return Name des Zustands als C-String
     */
    virtual const char* getStateName() const = 0;
};

/**
 * @class IdleState
 * @brief Zustand für inaktive Bewegungserkennung
 * @details Implementiert den Ruhezustand der Bewegungserkennung.
 *          In diesem Zustand sind die LEDs ausgeschaltet und das System
 *          wartet auf Bewegung.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
class IdleState : public MotionState {
public:
    void onMotionDetected(MotionSensorAutomation* context) override;
    void onTimeout(MotionSensorAutomation* context) override {}
    void update(MotionSensorAutomation* context) override;
    const char* getStateName() const override { return "Idle"; }
};

/**
 * @class ActiveState
 * @brief Zustand für aktive Bewegungserkennung
 * @details Implementiert den aktiven Zustand der Bewegungserkennung.
 *          In diesem Zustand sind die LEDs eingeschaltet und das System
 *          überwacht das Timeout für automatisches Ausschalten.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
class ActiveState : public MotionState {
public:
    void onMotionDetected(MotionSensorAutomation* context) override;
    void onTimeout(MotionSensorAutomation* context) override;
    void update(MotionSensorAutomation* context) override;
    const char* getStateName() const override { return "Active"; }
};

/**
 * @class MotionSensorAutomation
 * @brief Automatisierte Bewegungserkennung mit LED-Steuerung
 * @details Implementiert eine State-Machine für die Bewegungserkennung.
 *          Steuert NeoPixel LEDs basierend auf PIR-Sensor Eingaben.
 *          Unterstützt automatisches Timeout und verschiedene Farben.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Hardware:
 * - PIR Bewegungssensor
 * - NeoPixel LED Strip (WS2812B)
 * 
 */
class MotionSensorAutomation : public ISmaBui_IO {
private:
    uint8_t motionPin;                    ///< Pin für PIR-Sensor
    uint8_t ledPin;                       ///< Pin für NeoPixel Strip
    Adafruit_NeoPixel strip;              ///< NeoPixel Strip Controller
    MotionState* currentState;            ///< Aktueller Zustand
    IdleState idleState;                  ///< Ruhezustand Instanz
    ActiveState activeState;              ///< Aktiver Zustand Instanz
    unsigned long lastMotionTime;         ///< Zeitpunkt der letzten Bewegung
    unsigned long timeoutDuration;        ///< Timeout für automatisches Ausschalten
    bool lastMotionReading;               ///< Letzter Sensor-Wert
    unsigned long lastMotionCheck;        ///< Zeitpunkt der letzten Sensor-Abfrage
    const unsigned long motionCheckInterval = 100;  ///< Intervall für Sensor-Checks

public:
    MotionSensorAutomation(uint8_t motionPin, uint8_t ledPin, uint16_t numLeds = 4) 
        : motionPin(motionPin), ledPin(ledPin), strip(numLeds, ledPin, NEO_GRB + NEO_KHZ800),
          currentState(&idleState), lastMotionTime(0), timeoutDuration(10000),
          lastMotionReading(false), lastMotionCheck(0) {}

    void begin() override {
        pinMode(motionPin, INPUT);
        strip.begin();
        strip.clear();
        strip.show();
        currentState = &idleState;
        lastMotionReading = digitalRead(motionPin);
        lastMotionCheck = millis();
    }

    void handle() override {
        unsigned long now = millis();
        if (now - lastMotionCheck >= motionCheckInterval) {
            lastMotionCheck = now;
            bool currentMotionReading = digitalRead(motionPin);
            if (currentMotionReading == HIGH && lastMotionReading == LOW) {
                currentState->onMotionDetected(this);
            }
            lastMotionReading = currentMotionReading;
        }
        currentState->update(this);
    }

    void setState(MotionState* newState) {
        if (currentState != newState) {
            currentState = newState;
        }
    }

    void turnOnLEDs() {
        for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(255, 255, 255));
        }
        strip.show();
    }

    void turnOffLEDs() {
        strip.clear();
        strip.show();
    }

    void setLEDsRed() {
        for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(255, 0, 0));
        }
        strip.show();
    }

    void setLEDsGreen() {
        for (int i = 0; i < strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0, 255, 0));
        }
        strip.show();
    }

    void resetTimer() {
        lastMotionTime = millis();
    }

    bool isTimedOut() {
        unsigned long elapsed = millis() - lastMotionTime;
        bool timedOut = elapsed >= timeoutDuration;
        return timedOut;
    }

    unsigned long getRemainingTime() {
        unsigned long elapsed = millis() - lastMotionTime;
        return (elapsed >= timeoutDuration) ? 0 : (timeoutDuration - elapsed);
    }

    IdleState* getIdleState() { return &idleState; }
    ActiveState* getActiveState() { return &activeState; }
    const char* getCurrentStateName() const { return currentState->getStateName(); }
    void setRGBRed() { setLEDsRed(); }
    void setRGBGreen() { setLEDsGreen(); }
    void setRGBOff() { turnOffLEDs(); }
};

void IdleState::onMotionDetected(MotionSensorAutomation* context) {
    context->turnOnLEDs();
    context->resetTimer();
    context->setState(context->getActiveState());
}

void IdleState::update(MotionSensorAutomation* context) {}

void ActiveState::onMotionDetected(MotionSensorAutomation* context) {
    context->resetTimer();
}

void ActiveState::onTimeout(MotionSensorAutomation* context) {
    context->turnOffLEDs();
    context->setState(context->getIdleState());
}

void ActiveState::update(MotionSensorAutomation* context) {
    if (context->isTimedOut()) {
        onTimeout(context);
    }
}

/**
 * @class SensorDHT
 * @brief DHT11 Temperatur- und Luftfeuchtigkeitssensor
 * @details Kapselt die DHT11 Sensor-Funktionalität für Temperatur- und 
 *          Luftfeuchtigkeitsmessung. Bietet einfache Getter-Methoden für
 *          die aktuellen Messwerte.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Hardware:
 * - DHT11 Sensor für Temperatur und Luftfeuchtigkeit
 * - Digitaler Pin für Datenübertragung
 * 
 */
class SensorDHT : public ISmaBui_IO {
private:
    uint8_t pin;          ///< Pin für DHT11 Sensor
    DHTesp dht;           ///< DHT Sensor Controller
    float temperature;    ///< Aktuelle Temperatur in °C
    float humidity;       ///< Aktuelle Luftfeuchtigkeit in %

public:
    /**
     * @brief Konstruktor für DHT Sensor
     * @param pin GPIO Pin für DHT11 Sensor
     */
    SensorDHT(uint8_t pin) : pin(pin), temperature(0), humidity(0) {}

    /**
     * @brief Initialisiert den DHT Sensor
     * @details Konfiguriert den DHT11 Sensor für Datenübertragung
     */
    void begin() override {
        dht.setup(pin, DHTesp::DHT11);
    }

    /**
     * @brief Liest aktuelle Sensor-Werte
     * @details Wird zyklisch aufgerufen um Temperatur und Luftfeuchtigkeit zu aktualisieren
     */
    void handle() override {
        TempAndHumidity data = dht.getTempAndHumidity();
        temperature = data.temperature;
        humidity = data.humidity;
    }

    /**
     * @brief Gibt aktuelle Temperatur zurück
     * @return Temperatur in Grad Celsius
     */
    float getTemperature() const {
        return temperature;
    }

    /**
     * @brief Gibt aktuelle Luftfeuchtigkeit zurück
     * @return Luftfeuchtigkeit in Prozent
     */
    float getHumidity() const {
        return humidity;
    }
};

/**
 * @class DisplayLCD
 * @brief 16x2 LCD Display Controller für Smart Building
 * @details Steuert ein 16x2 LCD Display über I2C für die Anzeige von
 *          Sensor-Daten, Menüs, Zeit und Statusmeldungen.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Hardware:
 * - 16x2 LCD Display mit I2C Backpack
 * - I2C Adresse (standardmäßig 0x27)
 * 
 */
class DisplayLCD : public ISmaBui_IO {
private:
    LiquidCrystal_I2C lcd;            ///< LCD Controller
    unsigned long lastUpdate;         ///< Zeitpunkt der letzten Aktualisierung
    const unsigned long updateInterval = 100;  ///< Update-Intervall in ms

public:
    DisplayLCD(uint8_t address) : lcd(address, 16, 2), lastUpdate(0) {}

    void begin() override {
        lcd.init();
        lcd.backlight();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(" Smart Building ");
        lcd.setCursor(0, 1);
        lcd.print(" startet... ");
        lastUpdate = millis();
    }

    void handle() override {
    }

    void showTempAndHumidity(float t, float h) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T: ");
        lcd.print(t, 1);
        lcd.print((char)223);
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("H: ");
        lcd.print(h, 1);
        lcd.print("%");
    }

    void showStatus(const char* msg) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(msg);
    }

    void showMenu(const char* const* items, int length, int selected) {
        lcd.clear();
        for (int i = 0; i < length && i < 2; ++i) {
            lcd.setCursor(0, i);
            if (i == selected) {
                lcd.print(">");
            } else {
                lcd.print(" ");
            }
            lcd.print(items[i]);
        }
    }

    void showTime(const struct tm& timeinfo) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Uhrzeit:");
        lcd.setCursor(0, 1);
        char buf[9];
        strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
        lcd.print(buf);
    }

    void showSingleValue(const char* label, float value, const char* unit) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(label);
        lcd.setCursor(0, 1);
        lcd.print(value, 1);
        lcd.print(unit);
    }
    
    void showOfflineMode() {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Offline Modus");
        lcd.setCursor(0, 1);
        lcd.print("Menu verfugbar");
    }

    void showMenuWithHeader(const char* header, const char* const* items, int length, int selected) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(header);
        lcd.setCursor(0, 1);
        lcd.print("> ");
        lcd.print(items[selected]);
    }

    void showLabelAndText(const char* label, const char* text) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(label);
        lcd.setCursor(0, 1);
        lcd.print(text);
    }
};

const char* ssid = "iPhone von Leard";
const char* password = "12345678";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;

SensorDHT sensorDHT(17);
DisplayLCD display(0x27);

#define MOTION_SENSOR_PIN 14
#define RGB_LED_PIN 26
#define RFID_SDA_ADDR 0x28
#define RFID_RST_PIN 2
#define SERVO_PIN 13

MotionSensorAutomation motionAutomation(MOTION_SENSOR_PIN, RGB_LED_PIN, 4);

MFRC522_I2C mfrc522(RFID_SDA_ADDR, RFID_RST_PIN);
Servo doorServo;
const byte authorizedUID[][4] = {
    {0x54, 0x53, 0x04, 0x50}
};
const int authorizedCount = sizeof(authorizedUID) / sizeof(authorizedUID[0]);
bool doorOpen = false;
unsigned long doorOpenTime = 0;

ISmaBui_IO* devices[] = { &sensorDHT, &display, &motionAutomation };
const size_t deviceCount = sizeof(devices) / sizeof(devices[0]);

bool wifiConnected = false;
bool timeSync = false;
bool webServerStarted = false;

const char* mqtt_server = "mqtt3.thingspeak.com";
const int mqtt_port = 1883;
const char* mqtt_user = "IBcELR4QGQEWKTMlFRoeLxs";
const char* mqtt_password = "U3YDD5RcKFKn1hiwAzmQRRrw";
const char* mqtt_clientID = "IBcELR4QGQEWKTMlFRoeLxs";
const char* mqtt_topic = "channels/3006112/publish";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastSend = 0;
const unsigned long sendInterval = 5 * 60 * 1000;

#define REGEN_SENSOR_PIN 34
#define BUTTON_NAV_PIN 16
#define BUTTON_SEL_PIN 27
#define LICHT_PIN 25
#define GAS_SENSOR_PIN 23
#define ALARM_LED_PIN 12
#define BUZZER_PIN 25

const char* menuItems[] = {
    "Temperatur",
    "Luftfeuchte",
    "Regenwert",
    "Gassensor",
    "IP Adresse"
};
const int menuLength = sizeof(menuItems) / sizeof(menuItems[0]);
bool menuActive = false;
bool valueDisplayed = false;
int menuIndex = 0;

bool lastNavState = HIGH, lastSelState = HIGH;
unsigned long lastNavPress = 0, lastSelPress = 0;
const unsigned long debounceDelay = 50;

bool alarmActive = false;
bool gasDetected = false;
unsigned long lastAlarmCheck = 0;
unsigned long lastBuzzerToggle = 0;
unsigned long lastLedToggle = 0;
bool buzzerState = false;
bool ledState = false;
const unsigned long alarmCheckInterval = 100;
const unsigned long buzzerToggleInterval = 300;
const unsigned long ledToggleInterval = 200;

enum DoorState { DOOR_CLOSED, DOOR_OPENING, DOOR_OPEN, DOOR_CLOSING };
DoorState doorState = DOOR_CLOSED;
unsigned long doorStateChangeTime = 0;
const unsigned long doorMoveTime = 1000;
const unsigned long doorOpenDuration = 5000;

enum LEDTestState { LED_TEST_IDLE, LED_TEST_RED, LED_TEST_GREEN, LED_TEST_OFF };
LEDTestState ledTestState = LED_TEST_IDLE;
unsigned long ledTestTimer = 0;
const unsigned long ledTestInterval = 1000;

enum RFIDDisplayState { RFID_IDLE, RFID_GRANTED, RFID_DENIED };
RFIDDisplayState rfidDisplayState = RFID_IDLE;
unsigned long rfidDisplayTimer = 0;
const unsigned long rfidDisplayTimeout = 2000;

enum StartupState { STARTUP_WIFI, STARTUP_TIME, STARTUP_COMPLETE };
StartupState startupState = STARTUP_WIFI;
unsigned long startupTimer = 0;

/**
 * @brief Erzeugt einen Signalton über den Buzzer
 * @param duration Dauer des Tons in Millisekunden
 * @details Gibt einen 1000Hz Ton für die angegebene Dauer aus
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
void buzzerBeep(int duration) {
    tone(BUZZER_PIN, 1000, duration);
}

/**
 * @brief Verarbeitet Menü-Aktionen basierend auf Auswahl
 * @param idx Index des ausgewählten Menüpunkts
 * @details Zeigt entsprechende Sensor-Werte oder Systeminformationen an:
 *          0=Temperatur, 1=Luftfeuchtigkeit, 2=Regenwert, 3=Gas-Status, 4=IP-Adresse
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
void handleMenuAction(int idx) {
    valueDisplayed = true;
    switch(idx) {
        case 0:
            display.showSingleValue("Temperatur:", sensorDHT.getTemperature(), "C");
            break;
        case 1:
            display.showSingleValue("Luftfeuchte:", sensorDHT.getHumidity(), "%");
            break;
        case 2: {
            int raw = analogRead(REGEN_SENSOR_PIN);
            int regenProzent = map(raw, 0, 4095, 0, 100);
            display.showSingleValue("Regenwert:", regenProzent, "%");
            break;
        }
        case 3: {
            bool gasStatus = digitalRead(GAS_SENSOR_PIN) == LOW;
            display.showSingleValue("Gas erkannt:", gasStatus ? 1.0 : 0.0, gasStatus ? " JA" : " NEIN");
            break;
        }
        case 4: {
            String ipStr = WiFi.localIP().toString();
            display.showLabelAndText("IP Adresse:", ipStr.c_str());
            break;
        }
    }
}

/**
 * @brief Überwacht Gas-Sensor und steuert Alarm-System
 * @details Liest Gas-Sensor kontinuierlich und aktiviert bei Gasdetektion
 *          Alarm mit blinkender LED und alternierendem Buzzer-Signal.
 *          Implementiert automatische Deaktivierung bei Normalisierung.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Verhalten:
 * - Gas erkannt: Aktiviert Alarm (LED blinkt, Buzzer alterniert)
 * - Kein Gas: Deaktiviert Alarm automatisch
 * - LED-Blink-Intervall: 200ms
 * - Buzzer-Intervall: 300ms
 * 
 */
void checkGasSensor() {
    unsigned long now = millis();
    if (now - lastAlarmCheck > alarmCheckInterval) {
        lastAlarmCheck = now;
        gasDetected = digitalRead(GAS_SENSOR_PIN) == LOW;
        if (gasDetected && !alarmActive) {
            alarmActive = true;
            digitalWrite(ALARM_LED_PIN, HIGH);
            buzzerBeep(200);
            ledState = true;
            buzzerState = true;
            lastLedToggle = now;
            lastBuzzerToggle = now;
        } else if (!gasDetected && alarmActive) {
            alarmActive = false;
            gasDetected = false;
            digitalWrite(ALARM_LED_PIN, LOW);
            noTone(BUZZER_PIN);
            ledState = false;
            buzzerState = false;
        }
    }
    if (alarmActive && (now - lastLedToggle > ledToggleInterval)) {
        lastLedToggle = now;
        ledState = !ledState;
        digitalWrite(ALARM_LED_PIN, ledState ? HIGH : LOW);
    }
    if (alarmActive && (now - lastBuzzerToggle > buzzerToggleInterval)) {
        lastBuzzerToggle = now;
        buzzerState = !buzzerState;
        if (buzzerState) {
            tone(BUZZER_PIN, 800);
        } else {
            noTone(BUZZER_PIN);
        }
    }
}

void checkButtons() {
    bool currentNav = digitalRead(BUTTON_NAV_PIN);
    bool currentSel = digitalRead(BUTTON_SEL_PIN);
    unsigned long now = millis();
    if (lastNavState == HIGH && currentNav == LOW && (now - lastNavPress) > debounceDelay) {
        lastNavPress = now;
        if (!menuActive && !valueDisplayed) {
            menuActive = true;
            menuIndex = 0;
        } else if (menuActive && !valueDisplayed) {
            menuIndex = (menuIndex + 1) % menuLength;
        }
    }
    if (lastSelState == HIGH && currentSel == LOW && (now - lastSelPress) > debounceDelay) {
        lastSelPress = now;
        if (menuActive && !valueDisplayed) {
            handleMenuAction(menuIndex);
            menuActive = false;
        } else if (valueDisplayed) {
            valueDisplayed = false;
        }
    }
    lastNavState = currentNav;
    lastSelState = currentSel;
}

/**
 * @brief Öffnet die Tür über Servo-Motor
 * @details Steuert Servo auf 90° Position und setzt Zustand auf DOOR_OPENING.
 *          Funktion wird nur ausgeführt wenn Tür geschlossen ist.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
void openDoor() {
    if (doorState == DOOR_CLOSED) {
        doorServo.write(90);
        doorState = DOOR_OPENING;
        doorStateChangeTime = millis();
    }
}

/**
 * @brief Schließt die Tür über Servo-Motor
 * @details Steuert Servo auf 0° Position und setzt Zustand auf DOOR_CLOSING.
 *          Funktion wird nur ausgeführt wenn Tür offen ist.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
void closeDoor() {
    if (doorState == DOOR_OPEN) {
        doorServo.write(0);
        doorState = DOOR_CLOSING;
        doorStateChangeTime = millis();
    }
}

void handleDoorStateMachine() {
    unsigned long now = millis();
    switch (doorState) {
        case DOOR_OPENING:
            if (now - doorStateChangeTime >= doorMoveTime) {
                doorState = DOOR_OPEN;
                doorStateChangeTime = now;
            }
            break;
        case DOOR_OPEN:
            if (now - doorStateChangeTime >= doorOpenDuration) {
                closeDoor();
            }
            break;
        case DOOR_CLOSING:
            if (now - doorStateChangeTime >= doorMoveTime) {
                doorState = DOOR_CLOSED;
            }
            break;
    }
}

bool isAuthorized(const byte* uid, byte uidSize) {
    for (int i = 0; i < authorizedCount; ++i) {
        bool match = true;
        for (byte j = 0; j < uidSize; ++j) {
            if (authorizedUID[i][j] != uid[j]) {
                match = false;
                break;
            }
        }
        if (match) return true;
    }
    return false;
}

/**
 * @brief Verarbeitet RFID-Karten für Zugangssteuerung
 * @details Liest RFID-Karten, überprüft Autorisierung und steuert Türöffnung.
 *          Zeigt Zugang gewährt (grün) oder verweigert (rot) über RGB-LEDs an.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Verhalten:
 * - Autorisierte Karte: Grüne LED, Tür öffnet, Display zeigt "Hereinspaziert!"
 * - Nicht autorisierte Karte: Rote LED, Display zeigt "Karte abgelehnt"
 * - Anzeige-Timeout: 2 Sekunden
 * 
 */
void handleRFID() {
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;
    
    if (isAuthorized(mfrc522.uid.uidByte, mfrc522.uid.size)) {
        motionAutomation.setRGBGreen();
        rfidDisplayState = RFID_GRANTED;
        rfidDisplayTimer = millis();
        openDoor();
    } else {
        motionAutomation.setRGBRed();
        rfidDisplayState = RFID_DENIED;
        rfidDisplayTimer = millis();
    }
    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
}

void handleRFIDDisplay() {
    unsigned long now = millis();
    if (rfidDisplayState != RFID_IDLE && now - rfidDisplayTimer >= rfidDisplayTimeout) {
        rfidDisplayState = RFID_IDLE;
        motionAutomation.setRGBOff();
    }
}

void connectToWiFi() {
    if (startupState != STARTUP_WIFI) return;
    
    static bool wifiInitialized = false;
    static unsigned long wifiStartTime = 0;
    const unsigned long wifiTimeout = 10000;
    
    if (!wifiInitialized) {
        display.showStatus("Verbinde WLAN...");
        Serial.println("[WLAN] Starte Verbindung...");
        WiFi.mode(WIFI_STA);
        WiFi.disconnect(true);
        WiFi.begin(ssid, password);
        wifiInitialized = true;
        wifiStartTime = millis();
        return;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("[WLAN] Verbunden!");
        Serial.print("[WLAN] IP-Adresse: ");
        Serial.println(WiFi.localIP());
        startupState = STARTUP_TIME;
        startupTimer = millis();
    } else if (millis() - wifiStartTime >= wifiTimeout) {
        wifiConnected = false;
        Serial.println("[WLAN] Verbindung fehlgeschlagen - Offline Modus");
        startupState = STARTUP_COMPLETE;
    }
}

void syncTime() {
    if (startupState != STARTUP_TIME || !wifiConnected) return;
    
    static bool timeInitialized = false;
    static unsigned long timeStartTime = 0;
    static int timeAttempts = 0;
    const unsigned long timeTimeout = 10000;
    
    if (!timeInitialized) {
        Serial.println("[NTP] Synchronisiere Zeit...");
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        timeInitialized = true;
        timeStartTime = millis();
        return;
    }
    
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        Serial.println("[NTP] Zeit erfolgreich synchronisiert");
        timeSync = true;
        startupState = STARTUP_COMPLETE;
    } else if (millis() - timeStartTime >= timeTimeout || timeAttempts >= 10) {
        Serial.println("[NTP] Timeout - Zeit nicht synchronisiert");
        timeSync = false;
        startupState = STARTUP_COMPLETE;
    } else {
        timeAttempts++;
    }
}

void handleStartup() {
    switch (startupState) {
        case STARTUP_WIFI:
            connectToWiFi();
            break;
        case STARTUP_TIME:
            syncTime();
            break;
        case STARTUP_COMPLETE:
            break;
    }
}

void connectToMQTT() {
    if (!wifiConnected || mqttClient.connected()) return;
    
    static unsigned long lastMQTTAttempt = 0;
    static int mqttAttempts = 0;
    const unsigned long mqttRetryInterval = 5000;
    
    if (millis() - lastMQTTAttempt < mqttRetryInterval) return;
    
    lastMQTTAttempt = millis();
    if (mqttAttempts < 3) {
        if (mqttClient.connect(mqtt_clientID, mqtt_user, mqtt_password)) {
            Serial.println("[MQTT] Verbunden!");
            mqttAttempts = 0;
        } else {
            mqttAttempts++;
            Serial.println("[MQTT] Verbindung fehlgeschlagen");
        }
    }
}

void checkWiFiStatus() {
    static unsigned long lastWiFiCheck = 0;
    unsigned long now = millis();
    if (now - lastWiFiCheck > 30000) {
        lastWiFiCheck = now;
        if (WiFi.status() != WL_CONNECTED && wifiConnected) {
            wifiConnected = false;
            timeSync = false;
            webServerStarted = false;
            Serial.println("[WLAN] Verbindung verloren - Wechsel zu Offline Modus");
        } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
            wifiConnected = true;
            Serial.println("[WLAN] Verbindung wiederhergestellt");
            Serial.print("ESP32 IP-Adresse: ");
            Serial.println(WiFi.localIP());
            startupState = STARTUP_TIME;
        }
    }
}

void handleLEDTest() {
    unsigned long now = millis();
    switch (ledTestState) {
        case LED_TEST_RED:
            if (now - ledTestTimer >= ledTestInterval) {
                motionAutomation.setRGBGreen();
                ledTestState = LED_TEST_GREEN;
                ledTestTimer = now;
            }
            break;
        case LED_TEST_GREEN:
            if (now - ledTestTimer >= ledTestInterval) {
                motionAutomation.setRGBOff();
                ledTestState = LED_TEST_OFF;
                ledTestTimer = now;
            }
            break;
        case LED_TEST_OFF:
            if (now - ledTestTimer >= ledTestInterval) {
                ledTestState = LED_TEST_IDLE;
            }
            break;
    }
}

void startLEDTest() {
    if (ledTestState == LED_TEST_IDLE) {
        motionAutomation.setRGBRed();
        ledTestState = LED_TEST_RED;
        ledTestTimer = millis();
    }
}

/**
 * @brief Sendet Sensor-Daten an ThingSpeak IoT-Plattform
 * @details Sammelt Temperatur, Luftfeuchtigkeit und Regenwerte und sendet
 *          sie über MQTT an ThingSpeak für Cloud-Logging und Visualisierung.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Übertragene Daten:
 * - field1: Temperatur (°C)
 * - field2: Luftfeuchtigkeit (%)
 * - field3: Regenwert (%)
 * 
 */
void sendToThingSpeak() {
    if (!wifiConnected || !mqttClient.connected()) {
        Serial.println("[ThingSpeak] Übersprungen - Kein WLAN/MQTT");
        return;
    }
    float t = sensorDHT.getTemperature();
    float h = sensorDHT.getHumidity();
    int raw = analogRead(REGEN_SENSOR_PIN);
    int regenProzent = map(raw, 0, 4095, 0, 100);
    String payload = "field1=" + String(t, 1) + "&field2=" + String(h, 1) + "&field3=" + String(regenProzent);
    Serial.print("Sende an ThingSpeak: ");
    Serial.println(payload);
    bool success = mqttClient.publish(mqtt_topic, payload.c_str());
    if (success) {
        Serial.println("MQTT-Upload erfolgreich!");
    }
}

/**
 * @brief Setup-Funktion - Initialisiert alle Hardware-Komponenten
 * @details Initialisiert seriellen Port, alle I/O-Geräte, Pins, SPIFFS,
 *          I2C, RFID-Reader, PWM-Timer, Servo-Motor und MQTT-Client.
 *          Gibt ein Bestätigungssignal über den Buzzer aus.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 */
void setup() {
    Serial.begin(115200);
    
    // Initialisiere alle registrierten Geräte
    for (size_t i = 0; i < deviceCount; ++i)
        devices[i]->begin();
    
    // Konfiguriere GPIO Pins
    pinMode(REGEN_SENSOR_PIN, INPUT);
    pinMode(BUTTON_NAV_PIN, INPUT_PULLUP);
    pinMode(BUTTON_SEL_PIN, INPUT_PULLUP);
    pinMode(GAS_SENSOR_PIN, INPUT);
    pinMode(ALARM_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(ALARM_LED_PIN, LOW);
    noTone(BUZZER_PIN);
    
    // Bestätigungssignal beim Start
    tone(BUZZER_PIN, 1000, 200);
    
    // Initialisiere Button-Zustände
    lastNavState = digitalRead(BUTTON_NAV_PIN);
    lastSelState = digitalRead(BUTTON_SEL_PIN);
    
    // Initialisiere SPIFFS für Webserver
    if(!SPIFFS.begin(true)){
        return;
    }
    
    // Initialisiere I2C und RFID
    Wire.begin();
    mfrc522.PCD_Init();
    
    // Konfiguriere PWM Timer für Servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    doorServo.setPeriodHertz(50);
    
    // Initialisiere Servo Motor
    bool servoAttached = doorServo.attach(SERVO_PIN, 500, 2400);
    if (!doorServo.attached()) {
        doorServo.detach();
        servoAttached = doorServo.attach(SERVO_PIN);
    }
    doorServo.write(0);  // Tür geschlossen
    
    // Konfiguriere MQTT Client
    mqttClient.setServer(mqtt_server, mqtt_port);
    lastSend = millis();
}

/**
 * @brief Hauptschleife - Verarbeitet alle Systemfunktionen
 * @details Führt alle zyklischen Aufgaben aus: Startup-Sequenz, serielle Befehle,
 *          Button-Handling, Sensor-Überwachung, RFID-Verarbeitung, WiFi/MQTT-Kommunikation
 *          und Display-Updates. Implementiert zeitgesteuerte Verarbeitung.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Verarbeitungsreihenfolge:
 * 1. Startup-Handling (WiFi, Zeit-Synchronisation)
 * 2. Serielle Befehle (o/O=Tür öffnen, c/C=Tür schließen, r/R/g/G/l/L=LED Test)
 * 3. Hardware-Überwachung (Buttons, Gas, RFID, Tür, LEDs)
 * 4. Netzwerk-Kommunikation (MQTT, WebSocket, ThingSpeak)
 * 5. Sensor-Updates (alle 2s)
 * 6. Display-Updates (alle 500ms)
 * 
 */
void loop() {
    // Startup-Sequenz (WiFi, Zeit-Synchronisation)
    handleStartup();

    // Serielle Befehle verarbeiten
    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'o': case 'O': openDoor(); break;
            case 'c': case 'C': closeDoor(); break;
            case 'r': case 'R': case 'g': case 'G': case 'l': case 'L':
                startLEDTest(); break;
        }
    }

    // Hardware-Überwachung
    checkButtons();
    checkGasSensor();
    checkWiFiStatus();
    handleRFID();
    handleRFIDDisplay();
    handleDoorStateMachine();
    handleLEDTest();

    // Netzwerk-Kommunikation (nur wenn WiFi verbunden)
    if (wifiConnected) {
        mqttClient.loop();
        connectToMQTT();
        
        // WebSocket Daten senden (alle 2s)
        static unsigned long lastWS = 0;
        if (millis() - lastWS > 2000) {
            lastWS = millis();
            sendSensorDataWS();
        }
        
        // ThingSpeak Upload (alle 5 Minuten)
        if (millis() - lastSend > sendInterval) {
            lastSend = millis();
            sendToThingSpeak();
        }
        
        // Webserver starten (nach komplettem Startup)
        if (!webServerStarted && startupState == STARTUP_COMPLETE) {
            setupWebServer();
            webServerStarted = true;
        }
    }
    
    // Sensor-Updates (alle 2s)
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 2000) {
        lastSensorUpdate = millis();
        for (size_t i = 0; i < deviceCount; ++i)
            devices[i]->handle();
    }

    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate > 500) {
        lastDisplayUpdate = millis();
        
        if (alarmActive) {
            display.showStatus("Gasalarm!");
        } else if (rfidDisplayState == RFID_GRANTED) {
            display.showStatus("Hereinspaziert!");
        } else if (rfidDisplayState == RFID_DENIED) {
            display.showStatus("Karte abgelehnt");
        } else if (menuActive) {
            display.showMenuWithHeader("Menu", menuItems, menuLength, menuIndex);
        } else if (valueDisplayed) {
        } else if (startupState != STARTUP_COMPLETE) {
        } else {
            if (wifiConnected && timeSync) {
                struct tm timeinfo;
                if (getLocalTime(&timeinfo)) {
                    display.showTime(timeinfo);
                } else {
                    display.showTempAndHumidity(sensorDHT.getTemperature(), sensorDHT.getHumidity());
                }
            } else if (wifiConnected && !timeSync) {
                display.showTempAndHumidity(sensorDHT.getTemperature(), sensorDHT.getHumidity());
            } else {
                display.showOfflineMode();
            }
        }
    }
}

void showMenu() {
    display.showMenu(menuItems, menuLength, menuIndex);
}

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char* http_username = "admin";
const char* http_password = "geheim";

const char* webPage = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
    <meta charset="UTF-8">
    <title>Smart Building Bedienoberfläche</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Roboto', Arial, sans-serif; background-color: #fafafa; color: #212121; line-height: 1.5; }
        .container { max-width: 1000px; margin: 0 auto; padding: 20px; }
        .header { text-align: center; margin-bottom: 32px; }
        .title { font-size: 32px; font-weight: 400; color: #212121; margin-bottom: 8px; }
        .subtitle { font-size: 16px; color: #757575; }
        .main-grid { display: grid; grid-template-columns: 2fr 1fr; gap: 24px; margin-bottom: 32px; }
        .status-panel { background: white; border-radius: 4px; padding: 24px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .controls-panel { background: white; border-radius: 4px; padding: 24px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .panel-title { font-size: 20px; font-weight: 500; color: #212121; margin-bottom: 16px; }
        .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; margin-bottom: 16px; }
        .status-item { background: #f5f5f5; border-radius: 4px; padding: 16px; text-align: center; }
        .status-label { font-size: 12px; color: #757575; margin-bottom: 4px; text-transform: uppercase; }
        .status-value { font-size: 24px; font-weight: 500; color: #1976d2; }
        .connection-status { background: #f5f5f5; border-radius: 4px; padding: 16px; display: flex; align-items: center; justify-content: center; gap: 8px; }
        .status-text { font-size: 14px; font-weight: 500; }
        .status-dot { width: 8px; height: 8px; border-radius: 50%; }
        .status-connected { background-color: #4caf50; }
        .status-disconnected { background-color: #f44336; }
        .button-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 16px; }
        .control-button { background: #1976d2; color: white; border: none; border-radius: 4px; padding: 12px 24px; font-size: 14px; font-weight: 500; cursor: pointer; transition: background-color 0.2s ease; text-transform: uppercase; }
        .control-button:hover { background: #1565c0; }
        .control-button:active { background: #0d47a1; }
        .control-button.secondary { background: #757575; }
        .control-button.secondary:hover { background: #616161; }
        .charts-container { display: flex; flex-direction: column; align-items: center; gap: 24px; }
        .chart-section { background: white; border-radius: 4px; padding: 24px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); width: 100%; max-width: 800px; }
        .chart-container { text-align: center; }
        .chart-title { font-size: 18px; font-weight: 500; color: #212121; margin-bottom: 16px; }
        iframe { width: 100%; height: 300px; border: none; border-radius: 4px; }
        @media (max-width: 768px) { .main-grid { grid-template-columns: 1fr; } .button-grid { grid-template-columns: 1fr; } .status-grid { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="title">Smart Building Bedienoberfläche</div>
        </div>
        <div class="main-grid">
            <div class="status-panel">
                <div class="panel-title">Sensordaten</div>
                <div class="status-grid">
                    <div class="status-item">
                        <div class="status-label">Temperatur</div>
                        <div class="status-value"><span id="temp">--</span>&deg;C</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">Luftfeuchtigkeit</div>
                        <div class="status-value"><span id="hum">--</span>%</div>
                    </div>
                </div>
                <div class="connection-status">
                    <span id="status" class="status-text">Verbinde...</span>
                    <span id="statusDot" class="status-dot status-disconnected"></span>
                </div>
            </div>
            <div class="controls-panel">
                <div class="panel-title">Steuerung</div>
                <div class="button-grid">
                    <button class="control-button" onclick="sendCommand('openDoor')">T&uuml;r &ouml;ffnen</button>
                    <button class="control-button secondary" onclick="sendCommand('closeDoor')">T&uuml;r schlie&szlig;en</button>
                </div>
                <button class="control-button" style="width: 100%;" onclick="sendCommand('ledTest')">LED Test starten</button>
            </div>
        </div>
        <div class="charts-container">
            <div class="chart-section">
                <div class="chart-container">
                    <div class="chart-title">Temperatur Verlauf</div>
                    <iframe src="https://thingspeak.com/channels/3006112/charts/1?bgcolor=%23ffffff&amp;color=%23f44336&amp;dynamic=true&amp;results=60&amp;type=line">
                    </iframe>
                </div>
            </div>
            <div class="chart-section">
                <div class="chart-container">
                    <div class="chart-title">Luftfeuchtigkeit Verlauf</div>
                    <iframe src="https://thingspeak.com/channels/3006112/charts/2?bgcolor=%23ffffff&amp;color=%231976d2&amp;dynamic=true&amp;results=60&amp;type=line">
                    </iframe>
                </div>
            </div>
            <div class="chart-section">
                <div class="chart-container">
                    <div class="chart-title">Regenwert Verlauf</div>
                    <iframe src="https://thingspeak.com/channels/3006112/charts/3?bgcolor=%23ffffff&amp;color=%234caf50&amp;dynamic=true&amp;results=60&amp;type=line">
                    </iframe>
                </div>
            </div>
        </div>
    </div>
    <script>
        let ws;
        let reconnectInterval;
        function connectWebSocket() {
            ws = new WebSocket('ws://' + location.hostname + '/ws');
            ws.onmessage = function(event) {
                try {
                    let data = JSON.parse(event.data);
                    document.getElementById('temp').innerText = data.temp || '--';
                    document.getElementById('hum').innerText = data.hum || '--';
                    updateStatus('Verbunden', true);
                } catch(e) {
                    console.error('JSON Fehler:', e);
                }
            };
            ws.onopen = function() {
                updateStatus('Verbunden', true);
                clearInterval(reconnectInterval);
            };
            ws.onclose = function() {
                updateStatus('Getrennt', false);
                startReconnect();
            };
            ws.onerror = function() {
                updateStatus('Fehler', false);
            };
        }
        function updateStatus(text, connected) {
            document.getElementById('status').innerText = text;
            let dot = document.getElementById('statusDot');
            dot.className = 'status-dot ' + (connected ? 'status-connected' : 'status-disconnected');
        }
        function startReconnect() {
            if (!reconnectInterval) {
                reconnectInterval = setInterval(function() {
                    connectWebSocket();
                }, 3000);
            }
        }
        function sendCommand(cmd) {
            if(ws && ws.readyState === WebSocket.OPEN) {
                ws.send(cmd);
                console.log('Befehl gesendet:', cmd);
            } else {
                alert('WebSocket nicht verbunden!');
            }
        }
        connectWebSocket();
    </script>
</body>
</html>
)HTML";

/**
 * @brief Initialisiert den Webserver für Smart Building Interface
 * @details Startet AsyncWebServer mit HTTP Basic Authentication,
 *          WebSocket-Unterstützung und interaktiver Bedienoberfläche.
 *          Bietet Echtzeit-Sensor-Daten und Steuerungsfunktionen.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * Features:
 * - HTTP Basic Auth (admin/geheim)
 * - WebSocket für Echtzeit-Daten
 * - Tür-Steuerung (öffnen/schließen)
 * - LED-Test-Funktion
 * - ThingSpeak Chart-Integration
 * - Responsive Design
 * 
 */
void setupWebServer() {
    Serial.println("Webserver wird gestartet...");
    server.on("/logout", HTTP_GET, [](AsyncWebServerRequest *request){
        AsyncWebServerResponse *response = request->beginResponse(401, "text/html", 
            "<html><head><script>window.location.href='/';</script></head><body>Logging out...</body></html>");
        response->addHeader("WWW-Authenticate", "Basic realm=\"Logout\"");
        response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        request->send(response);
    });
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        AsyncWebServerResponse *response;
        if(!request->hasHeader("Authorization")) {
            Serial.println("Keine Authentifizierung - Fordere Anmeldedaten");
            response = request->beginResponse(401, "text/plain", "Authentication Required");
            response->addHeader("WWW-Authenticate", "Basic realm=\"Smart Building - Session " + String(millis()) + "\"");
            response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
            response->addHeader("Pragma", "no-cache");
            response->addHeader("Expires", "0");
            request->send(response);
            return;
        }
        if(!request->authenticate(http_username, http_password)) {
            Serial.println("Authentifizierung fehlgeschlagen - Ungültige Daten");
            response = request->beginResponse(401, "text/plain", "Invalid Credentials");
            response->addHeader("WWW-Authenticate", "Basic realm=\"Smart Building - Session " + String(millis()) + "\"");
            response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
            request->send(response);
            return;
        }
        Serial.println("Authentifizierung erfolgreich - Sende Webseite");
        String pageWithLogout = webPage;
        pageWithLogout.replace("</body>", 
            "<div style='position:fixed;top:10px;right:10px;'>"
            "<button onclick=\"window.location.href='/logout'\" style='background:#f44336;color:white;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;'>Logout</button>"
            "</div></body>");
        response = request->beginResponse(200, "text/html", pageWithLogout);
        response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        response->addHeader("Pragma", "no-cache");
        response->addHeader("Expires", "0");
        request->send(response);
    });
    ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                  void *arg, uint8_t *data, size_t len) {
        if(type == WS_EVT_DATA){
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            String msg = "";
            for(size_t i=0; i < len; i++) {
                msg += (char) data[i];
            }
            if(msg == "openDoor") {
                openDoor();
            }
            else if(msg == "closeDoor") {
                closeDoor();
            }
            else if(msg == "ledTest") {
                startLEDTest();
            }
        }
    });
    
    server.addHandler(&ws);
    server.serveStatic("/", SPIFFS, "/");
    server.begin();
    Serial.println("Webserver gestartet!");
    Serial.println("Anmeldedaten: admin / geheim");
}

/**
 * @brief Sendet Sensor-Daten über WebSocket an alle Clients
 * @details Erstellt JSON-Payload mit aktuellen Temperatur- und Luftfeuchtigkeitswerten
 *          und sendet diese an alle verbundenen WebSocket-Clients für Echtzeit-Updates.
 * 
 * @author Leard
 * @author David
 * @version 1.0
 * @date 09.07.2025
 * 
 * JSON-Format:
 * ```json
 * {
 *   "temp": 23.5,
 *   "hum": 45.2
 * }
 * ```
 * 
 *
 */
void sendSensorDataWS() {
    String json = "{\"temp\":" + String(sensorDHT.getTemperature(), 1) +
                  ",\"hum\":" + String(sensorDHT.getHumidity(), 1) + "}";
    ws.textAll(json);
}