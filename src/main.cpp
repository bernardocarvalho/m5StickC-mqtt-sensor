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
* https://docs.m5stack.com/en/core/m5stickc
* https://docs.m5stack.com/en/unit/watering
* https://github.com/m5stack/M5Stack/blob/master/examples/Unit/WATERING/WATERING.ino
* https://console.hivemq.cloud/clients/arduino-esp8266?uuid=e6c3a2b784ad4434b0238f17f98eac34
* https://github.com/m5stack/M5StickC/blob/master/examples/Advanced/MQTT/MQTT.ino
******************************************************************************
*/

#include <Arduino.h>

#include <M5StickC.h>
#include <WiFi.h>

//#include <NTPClient.h>
//#include <time.h>
//#include <TZ.h>
//#include <FS.h>
//#include <PubSubClient.h>

#include "time.h"

// Set the name and password of the wifi to be connected.

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>

#ifndef STASSID
#include "arduino_secrets.h"
#endif

const int WATER_AUTO = 2UL; // in sec. Auto water/ day

const char *ssid = STASSID;
const char *password = STAPSK;

const char* mqtt_server  = "test.mosquitto.org";
const char* mqtt_broker  = "test.mosquitto.org";
const int mqtt_port = 1883;

#define INPUT_PIN 33 //  SCL
#define PUMP_PIN  32 //  SDA

//const char* ssid     = "A52_BBC";
bool led_state = false;

WiFiClient espClient;
MqttClient mqttClient(espClient);
WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

const char willTopic[] = "ipfn/m5stick/will";
const char inTopic[]   = "ipfn/m5stick/in";
const char outTopic[]  = "ipfn/m5stick/out";

const char* ntpServer = "ntp1.tecnico.ulisboa.pt";  // Set the connect NTP server.
const long gmtOffset_sec     = 0;
const int daylightOffset_sec = 3600;
#define MSG_BUFFER_SIZE 50
char msg[MSG_BUFFER_SIZE];

int rawADC;
unsigned int sumWater = 0;

unsigned long stopPump = 0, nextWater;

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

void printLocalTime() {  // Output current time.
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {  // Return 1 when the time is successfully
        // obtained.
        M5.Lcd.println("Failed to obtain time");
        return;
    }
    M5.Lcd.println(&timeinfo,
            "%A, %B %d \n%Y %H:%M:%S");  // Screen prints date and time.
    Serial.println(&timeinfo,
            "%A, %B %d %Y %H:%M:%S");  //prints date and time.
    strftime(msg, MSG_BUFFER_SIZE, "%A, %B %d %Y %H:%M:%S", &timeinfo);

}
void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());
    //timeClient.begin();
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    M5.Lcd.println("\nCONNECTED!");
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
        nextWater = now + 2000UL;
    }
    Serial.print(pump);
    Serial.println();

}

void setup() {
    M5.begin();             // Init M5Stick.
    M5.Axp.EnableCoulombcounter();  // Enable Coulomb counter.
    pinMode(INPUT_PIN, INPUT);
    pinMode(PUMP_PIN, OUTPUT);
    digitalWrite(PUMP_PIN, LOW);
    pinMode(M5_LED, OUTPUT);
    M5.Lcd.setRotation(3);  // Rotate the screen.
    M5.Lcd.printf("\nConnecting to %s", ssid);
    Serial.begin(115200);
    setup_wifi();
    /*
       mqttClientP.setServer(mqtt_server,
       1883);  // Sets the server details.
       mqttClientP.setCallback(
       callback);  // Sets the message callback function.
       */
    String willPayload = "oh no!";
    bool willRetain = true;
    int willQos = 1;

    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();
    if (!mqttClient.connect(mqtt_broker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());

        while (1);
    }

    Serial.println("You're connected to the MQTT broker!");

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);
    Serial.println();

    // subscribe to a topic
    // the second parameter sets the QoS of the subscription,
    // the library supports subscribing at QoS 0, 1, or 2
    int subscribeQos = 1;

    mqttClient.subscribe(inTopic, subscribeQos);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(inTopic);

    configTime(gmtOffset_sec, daylightOffset_sec,
            ntpServer, "pool.ntp.org", "time.nist.gov");  // init and get the time.
    // printLocalTime();
    //WiFi.mode(WIFI_OFF);  // Set the wifi mode to off.
    unsigned long now = millis();
    nextWater =  now + 24UL * 3600UL * 1000UL; // start next day
    delay(20);
}
void loop() {
    static  unsigned long lastMsg = 0;
    StaticJsonDocument<256> doc;

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
    if (now - lastMsg > 2000) {
        lastMsg = now;

        rawADC = analogRead(INPUT_PIN);
        snprintf(msg, MSG_BUFFER_SIZE, "Watering ADC value: %ld",
                rawADC);
        Serial.println(msg);
        //Serial.print("Watering ADC value: ");
        //Serial.println(rawADC);
        M5.Lcd.setCursor(40, 100);
        M5.Lcd.print(msg);
        //M5.Lcd.print("ADC: " + String(rawADC));
        // topic.

        M5.Lcd.setCursor(0, 25);  // Set cursor to (0,25).
        //timeClient.update();

        //    Serial.println(timeClient.getFormattedTime());

        doc["msg"]   = msg;

        printLocalTime(); // Time string will be on msq
        doc["time"]  = msg;
        doc["Humid"] = rawADC;
        doc["sumWater"] = sumWater;
        doc["AXP_Temp"]   =  M5.Axp.GetTempInAXP192();
        doc["Bat_I"]   =  M5.Axp.GetBatCurrent();
        doc["Bat_V"]   =  M5.Axp.GetBatVoltage();
        doc["In5_V"]   =  M5.Axp.GetVinVoltage();
        doc["In5_I"]   =  M5.Axp.GetVinCurrent();
        doc["USB_I"]   =  M5.Axp.GetVBusCurrent();
        /*
           display.printf("Bat:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetBatVoltage(),
           M5.Axp.GetBatCurrent());
           display.printf("USB:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetVBusVoltage(),
           M5.Axp.GetVBusCurrent());
           display.printf("5V-In:\r\n  V: %.3fv  I: %.3fma\r\n",
           M5.Axp.GetVinVoltage(), M5.Axp.GetVinCurrent());
           display.printf("Bat power %.3fmw", M5.Axp.GetBatPower());
           */
        bool retained = false;
        int qos = 1;
        bool dup = false;

        mqttClient.beginMessage(outTopic,  (unsigned long)measureJson(doc), retained, qos, dup);
        serializeJson(doc, mqttClient);
        mqttClient.endMessage();
    }
}
