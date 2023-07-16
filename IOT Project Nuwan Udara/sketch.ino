// Library import
#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include "PubSubClient.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>

// PIN declaration
#define BUZZZER_PIN  2
#define DHT_PIN  12
#define LDR_PIN  34
#define SERVO_PIN 23


///// Sensor Variables
float temperature;
float humidity;
float lightIntensity;
int minimumAngle = 30;
float controllingFactor = 0.75;

///// MQTT Auth data
const char* mqttServer = "test.mosquitto.org";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* clientId = "MediboxClientUdara";

///// WIFI Auth data
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// MQTT client setup
WiFiClient espClient;
PubSubClient client(espClient);

// Sensor declaration
RTC_DS1307 rtc;
DHT dht(DHT_PIN, DHT22);
Servo servoMotor;
LiquidCrystal_I2C LCD = LiquidCrystal_I2C(0x27, 20, 4);

//default Data
bool master_enable = true;
bool alarmEnabled1 = false;
int alarmHour1 = 8;
int alarmMinute1 = 0;
bool alarmEnabled2 = false;
int alarmHour2 = 14;
int alarmMinute2 = 30;
bool alarmEnabled3 = false;
int alarmHour3 = 23;
int alarmMinute3 = 4;


DateTime Origin_data(2023, 7, 15); // Default date 
int number_of_days=5;

int isBuzzerEnabled = 1;
int alarm_mode= 0;
int alarm_frequency = 256;
int alarm_delay = 1;
int alarm_duration =4;

static int startHour = -1;
static int startMinute = -1;

// Function declaration

//beep function
void beep() {
  if (isBuzzerEnabled == 1) {
    tone(BUZZZER_PIN, alarm_frequency);
    delay(150);
    noTone(BUZZZER_PIN);
    delay(100);
  }
}

// Function to calculate the number of days between a given date and the current date
int calculateDays(DateTime date1) {
  // Get the current date and time
  DateTime currentDate = rtc.now();


  // Convert DateTime objects to UNIX timestamps
  time_t timestamp1 = date1.unixtime();
  time_t timestamp2 = currentDate.unixtime();

  // Calculate the difference in seconds between the two timestamps
  int64_t diffSeconds = timestamp2 - timestamp1;

  // Calculate the number of days by dividing the difference in seconds by the number of seconds in a day (86400)
  int numDays = diffSeconds / (60 * 60 * 24);

  return numDays;
}


// Function to check if current hour and minute matches with alarm time and alarm is enabled
void checkAndTriggerAlarm(bool& alarmEnabled, int alarmHour, int alarmMinute){
    // Get the current time
  DateTime currentDate = rtc.now();
  int currentSecond = currentDate.second();
  int currentMinute = currentDate.minute();
  int currentHour = currentDate.hour();

  // If alarm is not enabled return immediately 
  if(!alarmEnabled)
    return;

  // If it's the alarm time, start the alarm and store the time
  if (currentHour == alarmHour && currentMinute == alarmMinute) {
    startHour = currentHour;
    startMinute = currentMinute;
  }
  // If the alarm has started and the duration has not passed, ring the alarm
  if (startHour != -1 && currentHour >= startHour && currentMinute >= startMinute && currentMinute < startMinute + alarm_duration) {
    if (alarm_mode == 0) {
      beep();
    } else if (alarm_mode == 1 && currentSecond % alarm_delay == 0) {
      beep();
    }
  }
   // If the time exceeds the alarm_duration, reset the alarm 
   if (currentMinute >= startMinute + alarm_duration){
       startMinute = -1;
       startHour = -1;
   }
}

// Function to ring the alarm based on the mode
void ringAlarm() {
  // Get the number of days between the origin date and the current date
  int numDays = calculateDays(Origin_data);
  if (!master_enable){
    return;
  }
  if (numDays>number_of_days){
    return;
  }
  else {
    checkAndTriggerAlarm(alarmEnabled1, alarmHour1, alarmMinute1);
    checkAndTriggerAlarm(alarmEnabled2, alarmHour2, alarmMinute2);
    checkAndTriggerAlarm(alarmEnabled3, alarmHour3, alarmMinute3);

  }
  
}


// read sensor function
void readSensors() {
  lightIntensity = float(analogRead(LDR_PIN)) / 4096.0;
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  
}

// a simple waiting spinner
void spinner() {
  static int8_t counter = 0;
  const char* glyphs = "\xa1\xa5\xdb";
  LCD.setCursor(0, 1);
  LCD.print(glyphs[counter++]);
  if (counter == strlen(glyphs)) {
    counter = 0;
  }
}

//wifi connect function
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Connecting to wifi");
  Serial.print("Connecting to wifi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    spinner();
    Serial.print(".");
  }
  LCD.setCursor(1, 0);
  LCD.print("Connected!");
  LCD.clear();
  Serial.println('\n');
}

//update servo angle function
void updateServoAngle(float intensity) {
  int angle = minimumAngle + (180 - minimumAngle) * intensity * controllingFactor;
  servoMotor.write(angle);
}

//publish message function to MQTT
void publishMessage(const char* topic, const char* payload) {
  client.publish(topic, payload);
}

//send sensor data function to MQTT
void sendSensorData() {
  char temperatureStr[6];
  char humidityStr[6];
  char lightIntensityStr[6];
  sprintf(temperatureStr, "%.2f", temperature);
  sprintf(humidityStr, "%.2f", humidity);
  sprintf(lightIntensityStr, "%.2f", lightIntensity);

  publishMessage("mediboxUdara/temperature", temperatureStr);
  publishMessage("mediboxUdara/humidity", humidityStr);
  publishMessage("mediboxUdara/light_intensity", lightIntensityStr);
}

//wifi reconnect function
void wifiReconnect() {
  while (WiFi.status() != WL_CONNECTED) {
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Re-connecting to wifi...");
    LCD.setCursor(0, 1);
    LCD.print("SSID: " + String(ssid));
    Serial.print("Attempting to reconnect to SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    delay(5000);
  }
  LCD.clear();
  Serial.println("Connected to wifi");
}
//update time and date from MQTT
void updateTimeAndDateFromMQTT(const char* payload) {
  DynamicJsonDocument doc(512);
  // Deserialize the JSON payload
  DeserializationError error = deserializeJson(doc, payload);
  // Check for parsing errors
  if (error) {
    Serial.print("Failed to parse JSON payload: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract time and date
  int year = doc["y"];
  int month = doc["m"];
  int day = doc["d"];
  int hour = doc["h"];
  int minute = doc["mi"];
  int second = doc["s"];

  // Update RTC
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
}

//update data from payload
void updateDataFromPayload(const char* payload) {
  DynamicJsonDocument doc(512);

  // Deserialize the JSON payload
  DeserializationError error = deserializeJson(doc, payload);

  // Check for parsing errors
  if (error) {
    Serial.print("Failed to parse JSON payload: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract alarm data
  JsonArray alarms = doc["a"];
  if (alarms.size() >= 3) {
    // Alarm 1
    alarmEnabled1 = alarms[0]["e"];
    alarmHour1 = alarms[0]["h"];
    alarmMinute1 = alarms[0]["m"];
    
    // Alarm 2
    alarmEnabled2 = alarms[1]["e"];
    alarmHour2 = alarms[1]["h"];
    alarmMinute2 = alarms[1]["m"];
    
    // Alarm 3
    alarmEnabled3 = alarms[2]["e"];
    alarmHour3 = alarms[2]["h"];
    alarmMinute3 = alarms[2]["m"];
  }

  // Extract origin date and number of days
  master_enable = doc["me"];
  int year = doc["y"];
  int month = doc["m"];
  int day = doc["d"];
  Origin_data = DateTime(year, month, day);
  number_of_days = doc["n"];
}



//setup function for MQTT
void connectToMQTT() {
  client.setServer(mqttServer, mqttPort);
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Connecting to MQTT");
  while (!client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT broker");
      LCD.setCursor(0, 1);
      LCD.print("Connected!");
      // client.subscribe("mediboxUdara/buzzer");
      client.subscribe("mediboxUdara/alarm");
      client.subscribe("mediboxUdara/time");
      client.subscribe("mediboxUdara/settings");
      delay(500);
      LCD.clear();
    }
    else {
      LCD.clear();
      LCD.setCursor(0, 0);
      LCD.print("Failed to connect MQTT");
      Serial.print("Failed with MQTT error code: ");
      Serial.println(client.state());
      delay(2000);
      LCD.clear();
    }
  }
}


// MQTT callback function
void mqttCallback(char* topic, byte* message, unsigned int length) {
  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Command Recived!");
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String payload;
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    payload += (char)message[i];
  }
  Serial.println(String(topic));

  // update alarm data
  if (String(topic) == "mediboxUdara/alarm") {
    Serial.print("Recived Data for alarm..");
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Alarm Recived!");
    updateDataFromPayload(payload.c_str());
    sleep(2);
    LCD.clear();
  }

  // update time and date
  if (String(topic) == "mediboxUdara/time") {
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Time Recived!");
    Serial.print("Recived Data for time..");
    updateTimeAndDateFromMQTT(payload.c_str());
    sleep(2);
    LCD.clear();
  }

  if (String(topic) == "mediboxUdara/settings") {
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Settings Recived!");
    Serial.print("Recived Data for settings..");
    updateSettingsFromMQTT(payload.c_str());
    sleep(2);
    LCD.clear();
  }

}

//update settings from MQTT
void updateSettingsFromMQTT(const char* payload) {
  DynamicJsonDocument doc(512);
  // Deserialize the JSON payload
  DeserializationError error = deserializeJson(doc, payload);
  // Check for parsing errors
  if (error) {
    Serial.print("Failed to parse JSON payload: ");
    Serial.println(error.c_str());
    return;
  }

  // Extract settings
  isBuzzerEnabled = (int)doc["b"];
  minimumAngle = (int)doc["min_a"];
  controllingFactor = doc["cf"].as<float>();
  alarm_mode= (int)doc["am"];
  alarm_frequency = (int)doc["f"];
  alarm_delay = (int)doc["del"];
  alarm_duration =(int)doc["dur"];

}


//mqtt reconnect function
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    wifiReconnect();
    if (client.connect(clientId)) {
      Serial.print(clientId);
      Serial.println(" connected");
      // client.subscribe("mediboxUdara/buzzer");
      client.subscribe("mediboxUdara/alarm");
      client.subscribe("mediboxUdara/time");
      client.subscribe("mediboxUdara/settings");
      // client.subscribe("topicName/led");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void setup() {

  Serial.begin(115200);
  Serial.println("Wellcome to the MediBoX!");
  Wire.begin();
  LCD.init();
  LCD.backlight();
  wifiConnect(); //connect to WIFI 
  client.setCallback(mqttCallback);
  connectToMQTT(); // Iniitate MQTT connection
  rtc.begin(); // Initiate Clock

  servoMotor.attach(SERVO_PIN); 
  if (! rtc.isrunning()) {
  Serial.println("RTC is NOT running, let's set the time!");
  rtc.adjust(DateTime(2023, 7, 1, 12, 30, 0)); //default time
  };


}

void loop() {
  DateTime time = rtc.now();

  readSensors();
  ringAlarm();
  wifiReconnect();

  if (!client.connected()) {
    mqttReconnect();
  }
  sendSensorData();
  updateServoAngle(lightIntensity);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("Sensor Data:");
  Serial.println("-------------");
  Serial.println(String("Temp:\t")+ temperature);
  Serial.println(String("Humid:\t")+ humidity);
  Serial.println(String("Light:\t")+ lightIntensity);
  Serial.println("Current Date Time:");
  Serial.println("-------------");
  Serial.println(String("DateTime::TIMESTAMP_FULL:\t")+time.timestamp(DateTime::TIMESTAMP_FULL));
  Serial.println("Origin Date:");
  Serial.println("-------------");
  Serial.printf("%d/%d/%d\n", Origin_data.day(), Origin_data.month(), Origin_data.year());
  Serial.println("Alarm Data:");
  Serial.println("-------------");
  Serial.printf("1: Enabled=%s, Hour=%d, Minute=%d\n", alarmEnabled1 ? "true" : "false", alarmHour1, alarmMinute1);
  Serial.printf("2: Enabled=%s, Hour=%d, Minute=%d\n", alarmEnabled2 ? "true" : "false", ssssalarmHour2, alarmMinute2);
  Serial.printf("3: Enabled=%s, Hour=%d, Minute=%d\n", alarmEnabled3 ? "true" : "false", alarmHour3, alarmMinute3);

  LCD.setCursor(0, 0);
  LCD.print("T:" + String((int)round(temperature)) + "C ");
  LCD.print("H:" + String((int)round(humidity)) + "% ");
  LCD.print("Li:" + String(lightIntensity));
  LCD.setCursor(0, 1);
  LCD.print("D:" + String(time.day()) + "/" + String(time.month()) + "/" + String(time.year()) + " ");
  LCD.print(String(time.hour()) + ":" + String(time.minute()) + ":" + String(time.second()));
  LCD.setCursor(0, 2);
  LCD.print("A1:" + String(alarmEnabled1) + " " + String(alarmHour1) + ":" + String(alarmMinute1)+ " ");
  //LCD.setCursor(9, 2);
  LCD.print("A2:" + String(alarmEnabled2) + " " + String(alarmHour2) + ":" + String(alarmMinute2)+ " ");
  LCD.setCursor(0, 3);
  LCD.print("A3:" + String(alarmEnabled3) + " " + String(alarmHour3) + ":" + String(alarmMinute3)+ " ");
  //LCD.setCursor(9, 3);
  LCD.print("Left:" + String(number_of_days)+"days");
  Serial.println("\n");
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  client.loop();
  delay(10);
}
