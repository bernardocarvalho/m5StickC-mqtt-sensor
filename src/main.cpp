/*
 * vim: sta:et:sw=4:ts=4:sts=4
 *  * @file
 *******************************************************************************
 * Copyright (c) 2021 by M5Stack
 *                  Equipped with M5StickC sample source code
 *                          配套  M5StickC 示例源代码
 * Visit for more information: https://docs.m5stack.com/en/core/m5stickc
 *
 * Describe:  NTP TIME.
 * Date: 2021/8/3
Host gps
    HostName 10.136.227.237
    User bernardo
 * https://docs.m5stack.com/en/core/m5stickc
 * https://docs.m5stack.com/en/unit/watering
 * https://github.com/m5stack/M5Stack/blob/master/examples/Unit/WATERING/WATERING.ino
 * https://console.hivemq.cloud/clients/arduino-esp8266?uuid=e6c3a2b784ad4434b0238f17f98eac34
 * https://github.com/m5stack/M5StickC/blob/master/examples/Advanced/MQTT/MQTT.ino
 ******************************************************************************
 */

#include <Arduino.h>

#include <EEPROM.h>
#include <M5StickC.h>
#include <WiFi.h>

//#include <NTPClient.h>
//#include <TZ.h>
//#include <FS.h>

#include "time.h"

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

int addr = 0;  // EEPROM Start number of an ADDRESS.  EEPROM
#define SIZE 32  // define the size of EEPROM(Byte).
                 //
// Set the name and password of the wifi to be connected.
#ifndef SECRET_SSID
#include "arduino_secrets.h"
#endif

const int WATER_AUTO = 2UL; // in sec. Auto water/ day

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

const char* mqtt_broker  = "test.mosquitto.org";
const int mqtt_port = 1883;
const int msgPeriod = 30 * 1000U;

#define INPUT_PIN 33 //  SCL
#define PUMP_PIN  32 //  SDA

#define NH2O_E_ADD    0x0
#define SUMH2O_E_ADD  0x4
#define CNTH2O_E_ADD  0x8
#define REBOOTS_E_ADD 0xC

WiFiClient espClient;
MqttClient mqttClient(espClient);

const char willTopic[] = "ipfn/m5stick/will";
const char inTopic[]   = "ipfn/m5stick/in";
const char outTopic[]  = "ipfn/m5stick/out";

const char* ntpServer = "ntp1.tecnico.ulisboa.pt";  // Set the connect NTP server.
const long gmtOffset_sec     = 0;
const int daylightOffset_sec = 0; // 3600;

#define MSG_BUFFER_SIZE 50
char msg[MSG_BUFFER_SIZE];

#define WIFI_RETRY 20
bool wifiOK = false;
bool ntpOK = false;

bool led_blink = false;
bool led_state = false;

int rawADC;
unsigned int sumWater = 0;

unsigned long stopPump = 0, nextWater;

void printLocalTime() {  // Output current time.
    struct tm timeinfo;
    M5.Lcd.setCursor(0, 20);
    if (!getLocalTime(&timeinfo)) {  // Return 1 when the time is successfully
        // obtained.
        M5.Lcd.println("Failed to obtain time");
        return;
    }
    M5.Lcd.println(&timeinfo,
            "%A, %B %d \n%Y %H:%M:%S");  // Screen prints date and time.
    Serial.println(&timeinfo,
            "%A, %B %d %Y %H:%M:%S");  //prints date and time.
    //strftime(msg, MSG_BUFFER_SIZE, "%A %B %d %Y %H:%M:%S", &timeinfo);
    // ISO 8601 UTC
    strftime(msg, MSG_BUFFER_SIZE, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

}
void reconnect_wifi() {
    if (WiFi.status() == WL_CONNECTED)
        return;
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("Connecting to %s.", ssid);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    wifiOK = false;

    WiFi.begin(ssid, password);
    led_state = false;
    led_blink = false;

    for (int i = 0; i < WIFI_RETRY; i++){
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print(".");
            led_state = not led_state;
            digitalWrite(M5_LED, led_state);
            delay(500);
           // continue;
        }
        else {
            wifiOK = true;
            led_blink = true;

            Serial.println("");
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
            M5.Lcd.setCursor(0, 0);
            M5.Lcd.printf("CONNECTED to %s.", ssid);
            break;
        }
    }
    //randomSeed(micros());

}

void onMqttMessage(int messageSize) {
    StaticJsonDocument<256> doc;
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', duplicate = ");
    Serial.print(mqttClient.messageDup() ? "true" : "false");
    Serial.print(", QoS = ");
    Serial.print(mqttClient.messageQoS());
    Serial.print(", retained = ");
    Serial.print(mqttClient.messageRetain() ? "true" : "false");
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    /* use the Stream interface to print the contents
    //
    while (mqttClient.available()) {
    Serial.print((char)mqttClient.read());
    }
    */
    deserializeJson(doc, mqttClient);

    Serial.print(", Pump: ");
    int pump = doc["pump"];
    unsigned long now = millis();
    if (pump == 1){
        nextWater = now + 3000UL;
    }
    Serial.print(pump);
    Serial.println();

}
void write_long_eeprom(int addr, unsigned long val) {
    unsigned int uval;
    for (int i = 0; i < 4; i++) {
        uval = (unsigned int) ( 0xFF & val);
        EEPROM.write(addr, val);
        val /= 256;
        addr++;
    }
}
unsigned long read_long_eeprom(int addr) {
    unsigned int uval;
    unsigned long val = 0;
    for (int i = 3; i >= 0; i--) {
        val = val << 4;
        uval = 0xFF * EEPROM.read(addr);
        val |= uval;
        addr++;
    }
    return val;
}
void setupMqtt() {
    String willPayload = "oh no!";
    bool willRetain = true;
    int willQos = 1;

    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();
        M5.Lcd.setCursor(0, 40);
    if (!mqttClient.connect(mqtt_broker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());

        while (1);
    }
        M5.Lcd.setCursor(0, 40);

    Serial.println("You're connected to the MQTT broker!");

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);
    Serial.println();

    // subscribe to a topic
    // the second parameter sets the QoS of the subscription,
    // the library supports subscribing at QoS 0, 1, or 2
    // QoS 1  guarantees that the message will be transferred successfully to the broker.
    int subscribeQos = 1;

    mqttClient.subscribe(inTopic, subscribeQos);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(inTopic);

}
void setup() {
    M5.begin();             // Init M5Stick.
    M5.Axp.EnableCoulombcounter();  // Enable Coulomb counter.
    pinMode(INPUT_PIN, INPUT);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    pinMode(M5_LED, OUTPUT);
    M5.Lcd.setRotation(3);  // Rotate the screen.
    Serial.begin(115200);
    if (!EEPROM.begin(SIZE)) {  // Request storage of SIZE size(success return
        Serial.println(
            "\nFailed to initialise EEPROM!");
   //     delay(1000000);
    }
    //Serial.println("\n\nPress BtnA to Write EEPROM");
    nextWater = read_long_eeprom(NH2O_E_ADD);
    sumWater = read_long_eeprom(SUMH2O_E_ADD);
    //reboo = read_long_eeprom(REBOOTS_E_ADD);
    
    reconnect_wifi();
    setupMqtt();

    configTime(gmtOffset_sec, daylightOffset_sec,
            ntpServer, "pool.ntp.org", "time.nist.gov");  // init and get the time .
    ntpOK = true;
    struct tm ntpTime;
    RTC_TimeTypeDef RTC_TimeStruct;
    M5.Rtc.GetTime(&RTC_TimeStruct);  // Gets the time in the real-time clock.
    snprintf(msg, MSG_BUFFER_SIZE, "RTC Time H:%02d M:%02d S: %02d\n", RTC_TimeStruct.Hours,
            RTC_TimeStruct.Minutes, RTC_TimeStruct.Seconds);
    Serial.println(msg);

    RTC_DateTypeDef RTC_DateStruct;
    if (getLocalTime(&ntpTime)) {  // Return 1 when the time is successfully
        // obtained.
        ntpOK = true;

        RTC_TimeStruct.Hours   = ntpTime.tm_hour;  // Set the time.  设置时间
        RTC_TimeStruct.Minutes = ntpTime.tm_min;
        RTC_TimeStruct.Seconds = ntpTime.tm_min;
        M5.Rtc.SetTime(&RTC_TimeStruct);  // and writes the set time to the real time
        // clock.
        //RTC_DateStruct.WeekDay = 3;  // Set the date.
        RTC_DateStruct.Month   = ntpTime.tm_mon;
        RTC_DateStruct.Date    = ntpTime.tm_mday;
        RTC_DateStruct.Year    = ntpTime.tm_year;
        M5.Rtc.SetData(&RTC_DateStruct);
    }
    else{
        Serial.println("Failed to obtain NTP time");
        ntpOK = false;
    }

    // printLocalTime();
    //WiFi.mode(WIFI_OFF);  // Set the wifi mode to off.
    unsigned long now = millis();
    nextWater =  now + 24UL * 3600UL * 1000UL; // start next day
    delay(20);
}
void loop() {
    static  unsigned long lastMsg = 0;
    static  unsigned long lastLed = 0;
    const int ledPeriod = 2 * 1000U;

    StaticJsonDocument<256> doc;

    if (wifiOK && (WiFi.status() != WL_CONNECTED)){
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.printf("Lost Conn to %s. 123456", ssid);
        wifiOK = false;
        led_blink = false;
        write_long_eeprom(NH2O_E_ADD, nextWater);
        write_long_eeprom(SUMH2O_E_ADD, sumWater);
    }
    unsigned long now = millis();
    if (now > nextWater){
        nextWater =  now + 24UL * 3600UL * 1000UL; // repeat next day
        stopPump = now + WATER_AUTO * 1000UL;
        sumWater += WATER_AUTO;
        led_state = true;
        digitalWrite(M5_LED, led_state);
        digitalWrite(PUMP_PIN, HIGH);
    }
    if (now > stopPump){
        digitalWrite(PUMP_PIN, LOW);
        led_state = false;
        digitalWrite(M5_LED, led_state);
    }
    //
    mqttClient.poll();
    now = millis();

    if(led_blink)
        if (now - lastLed > ledPeriod) {
            lastLed = now;
            led_state = not led_state;
            digitalWrite(M5_LED, led_state);
        }

    if (now - lastMsg > msgPeriod) {
        lastMsg = now;
        M5.Lcd.setCursor(0, 40);

        rawADC = analogRead(INPUT_PIN);
        //Serial.print(now/1000);
        snprintf(msg, MSG_BUFFER_SIZE, "%u, Humid ADC: %ld, ", now/1000, rawADC);
        Serial.print(msg);
        Serial.print(" Wifi: ");
        Serial.println(wifiOK);
        //M5.Lcd.setCursor(0, 25);  // Set cursor to (0,25).
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.print(msg);

        printLocalTime(); // Time string will be on msg
        doc["time"]  = msg;
        doc["Humid"] = rawADC;
        doc["sumWater"] = sumWater;
        doc["AXP_Temp"]   =  M5.Axp.GetTempInAXP192();
        doc["Bat_I"]   =  M5.Axp.GetBatCurrent();
        doc["Bat_V"]   =  M5.Axp.GetBatVoltage();
        doc["In5_V"]   =  M5.Axp.GetVinVoltage();
        doc["In5_I"]   =  M5.Axp.GetVinCurrent();
        doc["USB_V"]   =  M5.Axp.GetVBusVoltage();
        doc["USB_I"]   =  M5.Axp.GetVBusCurrent();
        bool retained = false;

        int publishQos = 1;
        bool dup = false;

        mqttClient.beginMessage(outTopic,  (unsigned long) measureJson(doc), retained, publishQos, dup);
        serializeJson(doc, mqttClient);
        mqttClient.endMessage();
    }
}

// vim: syntax=cpp ts=4 sw=4 sts=4 sr et

/*
#define SERIAL_PRINTF_MAX_BUFF      256
void serialPrintf(const char *fmt, ...);
void serialPrintf(const char *fmt, ...) {
    // Buffer for storing the formatted data
    char buff[SERIAL_PRINTF_MAX_BUFF];
    // pointer to the variable arguments list
    va_list pargs;
    // Initialise pargs to point to the first optional argument
    va_start(pargs, fmt);
    // create the formatted data and store in buff
    vsnprintf(buff, SERIAL_PRINTF_MAX_BUFF, fmt, pargs);
    va_end(pargs);
    Serial.print(buff);
}
*/
        
/*
         *
           display.printf("Bat:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetBatVoltage(),
           M5.Axp.GetBatCurrent());
           display.printf("USB:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetVBusVoltage(),
           M5.Axp.GetVBusCurrent());
           display.printf("5V-In:\r\n  V: %.3fv  I: %.3fma\r\n",
           M5.Axp.GetVinVoltage(), M5.Axp.GetVinCurrent());
           display.printf("Bat power %.3fmw", M5.Axp.GetBatPower());
*/
