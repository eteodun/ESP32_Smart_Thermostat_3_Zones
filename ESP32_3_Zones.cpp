#include <Arduino.h>
//Ver 1.3, problem with trv valves. updtae control heat
//  Update TRV status confirmation if heating mode is on or off.
//Ver 1.4:
//  Update condition to turn off heating  if mqtt or wifi is down.
//Ver 1.5 Update times running and 
// Version 1 6 desktop
// added from laptop
//  branch 1 
// Implemented hysteresis all
// implemented from desktop 11 47
//#include <DNSServer.h>
//#include <ESP8266HTTPUpdateServer.h>
#include <ArduinoJson.h>
#include <math.h>
#include <SPI.h>
#include <DHT.h>

#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#endif
#include <ESPAsyncWebServer.h>
//#include <AsyncEventSource.h>
#include <ESPAsyncWiFiManager.h>

#include <ESPAsyncTCP.h>
#include <WiFiUdp.h>  //For OTA
#include <ArduinoOTA.h> //For OTA

#include <PubSubClient.h>
#include <Adafruit_HDC1000.h>
#include <Adafruit_Sensor.h>

//Define ESP hostname on network
const char* ServerName = "thermostat";   // define hostname device.  e.g. If name = "thermostat" use http://thermostat.local/
///Credentials for MQTT server
///#define mqtt_user "homeassistant"
///#define mqtt_password "rie9AiYei5oox1Uiwiejiepi8bai6ahv3kai6ahgei7Cuupoa4ietoufies9xaeh"
///#define mqtt_server "192.168.100.133"  //is 15
//
const char* sw_version = "1.6-0";
const char* model = "3 Zones with TRV Valves, internal sensor, histeresis";
const char* mqtt_user = "mqtt_user";
const char* mqtt_password = "QAzx.123";
//#define mqtt_server "vpsmqtt.tehnocyber.ro"  //is 15
//
const char* mqtt_server = "homeassistant.local";    // Hostname of Homeassitant 
IPAddress serverIP;    //For find  Hostname of Homeassitant  
///
//Define times 
unsigned long OVERTIME_RUNNING = 100;  // Time in minutes maximu time to running 
///Time for running heating 
unsigned long intervalRunningOn = 9;  // Time in minutes to run one cicle of heat
///Time for running brake 
unsigned long intervalRunningOff = 6; // Time in minutes to brake heating
// 
// Define LOOPs time
const unsigned long SAMPLE_LOOP_TIME1 = 5;   //time in seconds
const unsigned long SAMPLE_LOOP_TIME2 = 9;   //time in seconds
const unsigned long SAMPLE_LOOP_TIME3 = 11;  //time in seconds
const unsigned long SAMPLE_LOOP_TIME4 = 4;   //time in seconds
//
// Define time to take in consideration temperature
#define TIME_SENSOR 7    // time in seconds to confirm temperatures
//
// Define relays pins ESP32
///const byte   RelayPIN1 = T8;   // Relay heater room 1 
///const byte   RelayPIN2 = T7;   // Relay heater room 2
///const byte   RelayPIN3 = T6;   // Relay heater room 3
///const byte   RelayPINHeater = T5;   // Relay heater control
/////END relays pins
///const byte   PINWifi = T1;   // Relay heater control
///const byte   PINAlarm = T2;   // Relay heater control
///#define LED 16
// Define relays pins ESP8266
const byte   RelayPIN1 = D8;   // Relay heater room 1 
const byte   RelayPIN2 = D7;   // Relay heater room 2
const byte   RelayPIN3 = D6;   // Relay heater room 3
const byte   RelayPINHeater = D5;   // Relay heater control
//END relays pins
const byte   PINWifi = D1;   // Relay heater control
const byte   PINAlarm = D2;   // Relay heater control
#define LED 16
//
// Define DHT22 sensor
#define DHTPIN D4  //Define data pin for DHT sensor 
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);
// END define DHT
//Definim timpi
//time sensor 1
unsigned long lowTempSensorTime1;
unsigned long lowlastReadTime1;  // Timestamp of the last time the analog input was read
//time sensor 2
unsigned long lowTempSensorTime2;
unsigned long lowlastReadTime2;  // Timestamp of the last time the analog input was read
//time sensor 3
unsigned long lowTempSensorTime3;
unsigned long lowlastReadTime3;  // Timestamp of the last time the analog input was read
// end time sensor
// time for high temp sensor 1
unsigned long highTempSensorTime1;
unsigned long highlastReadTime1;  // Timestamp of the last time the analog input was read
// time for high temp sensor 2
unsigned long highTempSensorTime2;
unsigned long highlastReadTime2;  // Timestamp of the last time the analog input was read
// time for high temp sensor 3
unsigned long highTempSensorTime3;
unsigned long highlastReadTime3;  // Timestamp of the last time the analog input was read

//Time to retry connect to mqtt server
unsigned long lastReconnectAttempt = 0; //Folosit in functia de reconectare MQTT. resetam timpul la 0
const unsigned long reconnectMQTTInterval = 5; //Intervalul între încercările de reconectare in secunde
//Loop time setting
unsigned long previousMillis1 = 0; //Folosit in loop. resetam timpul la 0
unsigned long previousMillis2 = 0; //Folosit in loop. resetam timpul la 0
unsigned long previousMillis3 = 0;
unsigned long previousMillis4 = 0;
unsigned long previousMillis5 = 0;

/// Overtime running millis
//unsigned long overRuningMillis = OVERTIME_RUNNING * 60 * 1000UL; //transformam in minute
unsigned long overRunningTime;
unsigned long overRunningLastReadTime;  // Timestamp of the last time the analog input was read
/// Running maxim time to heat or take a brake  
///Time for running heat
unsigned long previousMillis = 0;  //
//unsigned long intervalMillisOn = intervalRunningOn * 60 * 1000UL; //transformam in minute
///Time for running brake 
//unsigned long intervalMillisOff = intervalRunningOff * 60 * 1000UL; //transformam in minute
///

// define topics 
//Topics for  sensor name  on rooms
///Zone1
//Topic for sensor name 
const PROGMEM char* mqtt_topic_name_zone1 			      = "thermostat/zone1/name";
const PROGMEM char* mqtt_topic_temp_zone1 			      = "thermostat/zone1/temperature";
const PROGMEM char* mqtt_topic_target_temp_zone1 	    = "thermostat/zone1/target";
const PROGMEM char* mqtt_topic_TRV_status_zone1		    = "thermostat/zone1/availability/trv";
//const PROGMEM char* mqtt_topic_TRV_setting_zone1	  = "thermostat/zone1/TRV/set";  // Topic for TRV1
const PROGMEM char* mqtt_topic_TRV_setting_zone1	    = "thermostat/zone1/TRV/heating";  // Topic for TRV1
const PROGMEM char* mqtt_topic_maintenance_radiator_1 = "thermostat/zone1/maintenance/radiator";  	 // Topic for maintenance radiator1
const PROGMEM char* mqtt_topic_TRV_mode_zone1	    = "thermostat/zone1/status";  // Topic for TRV1 status confirmation
///Zone2
//Topic for sensor name 
const PROGMEM char* mqtt_topic_name_zone2 			      = "thermostat/zone2/name";
const PROGMEM char* mqtt_topic_temp_zone2 			      = "thermostat/zone2/temperature";
const PROGMEM char* mqtt_topic_target_temp_zone2 	    = "thermostat/zone2/target";
const PROGMEM char* mqtt_topic_TRV_status_zone2		    = "thermostat/zone2/availability/trv";
//const PROGMEM char* mqtt_topic_TRV_setting_zone2	    = "thermostat/zone2/TRV/set";  // Topic for TRV2
const PROGMEM char* mqtt_topic_TRV_setting_zone2	    = "thermostat/zone2/TRV/heating";  // Topic for TRV2
const PROGMEM char* mqtt_topic_maintenance_radiator_2 = "thermostat/zone2/maintenance/radiator";      // Topic for maintenance radiator_2
const PROGMEM char* mqtt_topic_TRV_mode_zone2	    = "thermostat/zone2/status";  // Topic for TRV2 status confirmation
///Zone3
//Topic for sensor name 
const PROGMEM char* mqtt_topic_name_zone3 			      = "thermostat/zone3/name";
const PROGMEM char* mqtt_topic_temp_zone3 			      = "thermostat/zone3/temperature";
const PROGMEM char* mqtt_topic_target_temp_zone3 	    = "thermostat/zone3/target";
const PROGMEM char* mqtt_topic_TRV_status_zone3		    = "thermostat/zone3/availability/trv";
//const PROGMEM char* mqtt_topic_TRV_setting_zone3	  = "thermostat/zone3/TRV/set"; 				 	// Topic for TRV3
const PROGMEM char* mqtt_topic_TRV_setting_zone3	    = "thermostat/zone3/TRV/heating"; 				 	// Topic for TRV3
const PROGMEM char* mqtt_topic_maintenance_radiator_3 = "thermostat/zone3/maintenance/radiator";   	// Topic for maintenance radiator_3
const PROGMEM char* mqtt_topic_TRV_mode_zone3	    = "thermostat/zone3/status";  // Topic for TRV3 status confirmation
// Define topics for boiler 
// Topic used at startup or reconnect ESP module
const PROGMEM char* MQTT_STARTUP_STATE 				    		= "thermostat/boiler/boot";
const PROGMEM char* MQTT_HVAC_MODE 							      = "thermostat/boiler/hvacmode";  			  			 // Topic for HVAC MODE ON or OFF
const PROGMEM char* MQTT_SENSOR_INTERNAL 				     	= "thermostat/boiler/temperature/internalTemp";
const PROGMEM char* MQTT_SENSOR_CORRECTION_INTERNAL		= "thermostat/boiler/correction_internal";    			 // Correction internal sensor
const PROGMEM char* MQTT_OVERRUNNING_TOPIC 					  = "thermostat/boiler/overruning/state";             	 // We send overrunning state to HomeAssistant
const PROGMEM char* MQTT_OVERRUNNING_RESET_TOPIC 			= "thermostat/boiler/maintenance/reset_overrunning";     // Topic to reset overruning state from HomeAssistant
const PROGMEM char* MQTT_HEATING_STATE 						    = "thermostat/boiler/heatingState"; 					 // Topic for ESP if heating or not 
const PROGMEM char* status_esp_keepalive 					    = "thermostat/boiler/status";   						 // Topic for send heartbeat to Homeassitant
const PROGMEM char* ESP_IP_TOPIC						          = "thermostat/boiler/ip";  								 //Publish ESP IP
const PROGMEM char* mqtt_topic_manual_heating				  = "thermostat/boiler/maintenance/manual_heating";   	 //  Topic for maintenance boiler
const PROGMEM char* mqtt_topic_maintenance            = "thermostat/boiler/maintenance/maintenance";  // Topic for maintenance
/////
const PROGMEM char* mqtt_boiler_overruning_time       = "thermostat/boiler/settings/overruning";  // Topic for maintenance
const PROGMEM char* mqtt_boiler_running_time          = "thermostat/boiler/settings/runningtime";  // Topic for maintenance
const PROGMEM char* mqtt_boiler_pause_time            = "thermostat/boiler/settings/pausetime";  // Topic for maintenance
/////// histerises 
const PROGMEM char* mqtt_boiler_histeresis_upper            = "thermostat/boiler/settings/histeresis_upper";  // Topic for histerises setting upper value

const PROGMEM char* mqtt_boiler_histeresis_down            = "thermostat/boiler/settings/histeresis_down";  // Topic for histerises for down value
///////
const PROGMEM char* MQTT_BRIDGE_STATE                 = "zigbee2mqtt/bridge/state";

// Define Booleans for status sensors 
bool TRV1 = false;
bool TRV2 = false;
bool TRV3 = false;
bool TRV1STATUS = false;
bool TRV2STATUS = false;
bool TRV3STATUS = false;
bool bridgeState = false;
// Define Booleans for maintenance
bool maintenance = false;
bool radiator_1 = false;
bool radiator_2 = false;
bool radiator_3 = false;
bool manual_heating = false;

// Define Boolean for HVAC mode 
bool hvacmode = false; // Used ig heating mode Is  On or Off
//
bool ON = true;  //Used for Radiator relay on or off
bool OFF = false; //Used for Radiator relay on or off
//
bool heatingActive = false;  //Used for heating state
bool pauseActive = false;   // Used for heating pause state
bool overrunning = false;  // Used for heating overrruning
//switch case
bool areAllTRVsOn = TRV1 && TRV2 && TRV3;
/// 
float  Temperature1, Temperature2, Temperature3;
float  Temperature1mqtt, Temperature2mqtt, Temperature3mqtt;
float TempInternal = 0;
///Target Temp variables 
float TargetTempRoom1 = 22.0;
float TargetTempRoom2 = 22.0;
float TargetTempRoom3 = 22.0;
float correction_internal = 0.0;
//
// Define histerises limit
float  histeresisup = 0.0;  // Grade of histeresis upper limit
//
float histeresisdown = 0.0; // Grade of histeresis lower limit
/// Define sensor names variables
String Senzor1_name = "";
String Senzor2_name = "";
String Senzor3_name = "";

String espip = "";
// Define relays state variables
String RelayState1          = "OFF";      // Current setting of the control/thermostat relay
String RelayState2          = "OFF";      // Current setting of the control/thermostat relay
String RelayState3          = "OFF";      // Current setting of the control/thermostat relay
String RelayHeaterState     = "OFF";      // Current setting of the control/thermostat relay
String TimerState           = "OFF";      // Current setting of the timer
String OverTime             = "OFF";      // Overtime running
// define variables for webpage 
String SytemAlarm           = "OFF";
String MQTT_CONNECT         = "OFF";
String SensorsConnected         = "ON";
/// Define actuating TRV VALVES 
const char TRVON[] = "{\"system_mode\": \"heat\"}";
const char TRVOFF[] = "{\"system_mode\": \"off\"}";
const char TRVTARGET[] = "{\"current_heating_setpoint\": \"28.0\"}";
///
 
///
String webpage              = "";    // Used for webpage  
AsyncWebServer server(80);
DNSServer dns;
AsyncWiFiManager wifiManager(&server,&dns);

WiFiClient espClient;  

PubSubClient client(espClient);  // Used to publish MQTT messages
static int restartNow = false;  // Defined to reboot ESP module
////String newHostname = "esp8266thermostat"; //Hostname on network
#define Refresh         true           // Set auto refresh page ON
#define noRefresh       false          // Set auto refresh page OFF

// setup HostName of device  . If name = "thermostat" use http://thermostat.local/
void SetupDeviceName(const char *DeviceName) {
  if (MDNS.begin(DeviceName)) { 
    Serial.println("mDNS responder started");
    Serial.print("Device name: ");
    Serial.println(DeviceName);
    MDNS.addService("n8i-mlp", "tcp", 23);
  }
  else
    Serial.println("Error setting up MDNS responder");
}
// Publish ESP IP to MQTT topic 
void publishIP() {  
    char ipMessage[30];
    sprintf(ipMessage, "IP: %s", WiFi.localIP().toString().c_str());
    client.publish(ESP_IP_TOPIC, ipMessage);
    //Serial.println("publishIP");
}
void subscribeToTopics() {
  // Abonarea la topic-uri după reconectare
  client.subscribe(mqtt_topic_name_zone1);
  client.subscribe(mqtt_topic_name_zone2);
  client.subscribe(mqtt_topic_name_zone3);

  client.subscribe(mqtt_topic_TRV_status_zone1);
  client.subscribe(mqtt_topic_TRV_status_zone2);
  client.subscribe(mqtt_topic_TRV_status_zone3);

  ///
  client.subscribe(mqtt_topic_temp_zone1);
  client.subscribe(mqtt_topic_temp_zone2);
  client.subscribe(mqtt_topic_temp_zone3);
 
  ///
  client.subscribe(mqtt_topic_target_temp_zone1);
  client.subscribe(mqtt_topic_target_temp_zone2);
  client.subscribe(mqtt_topic_target_temp_zone3);
  
  //
  client.subscribe(mqtt_topic_maintenance); 
  client.subscribe(mqtt_topic_maintenance_radiator_1);  
  client.subscribe(mqtt_topic_maintenance_radiator_2);  
  client.subscribe(mqtt_topic_maintenance_radiator_3);
  //
  client.subscribe(mqtt_topic_TRV_mode_zone1);
  client.subscribe(mqtt_topic_TRV_mode_zone2);
  client.subscribe(mqtt_topic_TRV_mode_zone3);
  //
  client.subscribe(mqtt_topic_TRV_setting_zone1);
  client.subscribe(mqtt_topic_TRV_setting_zone2);
  client.subscribe(mqtt_topic_TRV_setting_zone3);
  //boiler settings
  client.subscribe(mqtt_boiler_overruning_time);
  client.subscribe(mqtt_boiler_running_time);
  client.subscribe(mqtt_boiler_pause_time);
  client.subscribe(mqtt_boiler_histeresis_upper);
  client.subscribe(mqtt_boiler_histeresis_down);
  //
  client.subscribe(mqtt_topic_manual_heating);
  //
  client.subscribe(MQTT_HVAC_MODE);
  client.subscribe(MQTT_OVERRUNNING_RESET_TOPIC);
  client.subscribe(MQTT_BRIDGE_STATE);
  //
  client.subscribe(MQTT_SENSOR_CORRECTION_INTERNAL);
}
void reconnect() {  // Function to reconnect MQTT 
  if (!client.connected()){
    Temperature1 = 0;
    Temperature2 = 0;
    Temperature3 = 0;
    MQTT_CONNECT = "OFF";
  unsigned long currentMillis = millis();
  if (currentMillis - lastReconnectAttempt >= reconnectMQTTInterval * 1000) {   //incercam la 5 secunde sa ne reconectam la mqtt. nu am folosit while 
    lastReconnectAttempt = currentMillis;
    if (client.connect("thermostat", mqtt_user, mqtt_password)) {
      //Serial.println("MQTT_connected"); // pentru verificare
      MQTT_CONNECT = "ON";
      subscribeToTopics();  // We call function su subscribe topics
      client.publish(MQTT_STARTUP_STATE, String("started").c_str(), false);
      publishIP();
      if (WiFi.hostByName(mqtt_server, serverIP)) {
      Serial.print("IP address of HAS ");
      Serial.print(mqtt_server);
      Serial.print(": ");
      Serial.println(serverIP);
    } else {
      Serial.println("Failed to get IP address from DNS");
    }
    }
    else {
      ///daca nu se conecteaza, facem ce e mai jos
      Serial.print("Failed to connect MQTT");
      Serial.print(client.state());
      Serial.println("Reconnectin in 5 seconds");
      // Wait 5 seconds before retrying and again
      //delay(5000);
    }
   }
  }
}
void StartUp_ESP(){
  
  if ((Temperature1 == 0.00 && TRV1) ||
      (Temperature2 == 0.00 && TRV2) ||
      (Temperature3 == 0.00 && TRV3)) {
      client.publish(MQTT_STARTUP_STATE, String("started").c_str(), false);
      Serial.println("Am bootat: ");
  }
}

void checkSensor(){
  if (client.connected()){
    if (TRV1){
      Temperature1 = Temperature1mqtt;
      }
    else {
     Serial.println("TRV1: Offline");
     Temperature1 = 0;
    }
    if (TRV2){
      Temperature2 = Temperature2mqtt;
    }
    else {
     Serial.println("TRV2: Offline");
     Temperature2 = 0;
    }
    if (TRV3){
      Temperature3 = Temperature3mqtt;
      }
    else {
     Serial.println("TRV3: Offline");
     Temperature3 = 0;
    }
    if(!bridgeState){
     TRV1 =  false;
     TRV2 =  false;
     TRV3 =  false;
     Serial.println("Zigbee Device: Offline");
     SensorsConnected = "ON";
    }
    if(areAllTRVsOn){
      Serial.println("All sensors are up");
      SensorsConnected = "OFF";
    }
    else {
      SensorsConnected = "ON";
    }
  }
}

void updateTRVState(const char* topic,byte* payload, unsigned int length, bool& TRVState) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  DynamicJsonBuffer jsonBuffer;
  
  //Serial.print("Received payload: ");
  //Serial.println(msg);

  JsonObject& root = jsonBuffer.parseObject(msg);
  if (!root.success()) {
    Serial.println("Eroare la parsarea JSON-update_TRV_State");
    return;
  }
    String mesaj = root["state"];
    if (mesaj == "online"){
        //Serial.println("TRV2 online ");
        TRVState = true; 
      }
    if (mesaj == "offline"){
        //Serial.println("TRV2 offline ");
        TRVState = false;
      }
}

void handleTRVSTATUS(const char* topic,byte* payload, unsigned int length, bool& TRVSTATUS) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  DynamicJsonBuffer jsonBuffer;
  
  //Serial.print("Received payload: ");
  //Serial.println(msg);

  JsonObject& root = jsonBuffer.parseObject(msg);
  if (!root.success()) {
    Serial.println("Eroare la parsarea JSON-update_TRV_State");
    return;
  }
    String mesaj = root["state"];
    if (mesaj == "heatingon"){
        //Serial.println("TRV2 online ");
        TRVSTATUS = true; 
      }
    if (mesaj == "heatingoff"){
        //Serial.println("TRV2 offline ");
        TRVSTATUS = false;
      }
}
void handleTemperatureSensor(const char* topic, byte* payload, unsigned int length, float& temperatura) {
  DynamicJsonBuffer jsonBuffer;
  payload[length] = '\0';
  String msg = String((char*)payload);
  JsonObject& root = jsonBuffer.parseObject(msg);
  if (!root.success()) {
    Serial.println("Eroare la parsarea JSON-ului.");
    return;
  }
  // Verificăm dacă cheia "temperature" există în JSON și este un număr
  if (root.containsKey("temperature") && root["temperature"].is<float>()) {
    temperatura = root["temperature"];
  } else {
    Serial.println("Eroare: Cheia 'temperature' lipsește sau nu este un număr valid");
  }
}
void handleTargetTempRoom(const char* topic, byte* payload, unsigned int length, float& TargetTempRoom) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  TargetTempRoom = msg.toFloat();
}
void handleSenzor_name(const char* topic, byte* payload, unsigned int length, String& Senzor_name) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  Senzor_name = msg;
}
//Histerises handle
void handlehisterises(const char* topic, byte* payload, unsigned int length, float& histerises) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  histerises = msg.toFloat();
}
void handleRadiator(const char* topic, byte* payload, unsigned int length, bool& radiator) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(msg);
  if (!root.success()) {
    Serial.println("Eroare la parsarea JSON-ului_Bridge");
      return;
  }
  String mesaj = root["state"];
  if (mesaj == "on"){
  //Serial.println("Senzor_4 online ");
    radiator = true; 
  }
  if (mesaj == "off"){
     //Serial.println("Senzor_4 offline ");
    radiator = false;
  }
}
// boiler settings
void handleOVERTIMERUNNINGTIME(const char* topic, byte* payload, unsigned int length, long unsigned& OVERTIMERUNNINGTIME) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  OVERTIMERUNNINGTIME = msg.toInt();
}

void handleIntervalRunningTIME(const char* topic, byte* payload, unsigned int length, long unsigned& INTERVALRUNNINGTIME) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  INTERVALRUNNINGTIME = msg.toInt();
}

void handleIntervalRunningPauseTIME(const char* topic, byte* payload, unsigned int length, long unsigned& INTERVALPAUSETIME) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  INTERVALPAUSETIME = msg.toInt();
}
///
void ActuateHeating(bool demand) {
  unsigned long currentMillis = millis();
  if (OverTime == "OFF") {
    ///unsigned long intervalMillisOn = intervalRunningOn * 60 * 1000UL; //transformam in minute
    ///unsigned long intervalMillisOff = intervalRunningOff * 60 * 1000UL; 
    if (demand && !heatingActive && !pauseActive) {
      
      overrunning = true;
      // Porneste încălzirea
      RelayHeaterState = "ON";
      //Serial.println("Thermostat ON");
      digitalWrite(RelayPINHeater, HIGH);
      heatingActive = true;
      client.publish(MQTT_HEATING_STATE, String("heating").c_str(), true);
      previousMillis = currentMillis;  // Actualizează timpul ultimei acțiuni
    }

    if (heatingActive && currentMillis - previousMillis >= intervalRunningOn * 60 * 1000UL) {
      // Oprește încălzirea după 15 secunde și pregătește pauza
      RelayHeaterState = "OFF";
      //Serial.println("Thermostat OFF");
      digitalWrite(RelayPINHeater, LOW);
      heatingActive = false;
      pauseActive = true;
      client.publish(MQTT_HEATING_STATE, String("notHeating").c_str(), true);
      previousMillis = currentMillis;  // Actualizează timpul ultimei acțiuni
    }

    if (pauseActive && currentMillis - previousMillis >= intervalRunningOff * 60 * 1000UL) {
      // Porneste încălzirea din nou după 10 secunde de pauză și resetează variabilele
      RelayHeaterState = "ON";
      //Serial.println("Thermostat ON");
      digitalWrite(RelayPINHeater, HIGH);
      heatingActive = true;
      pauseActive = false;
      client.publish(MQTT_HEATING_STATE, String("heating").c_str(), true);
      previousMillis = currentMillis;  // Actualizează timpul ultimei acțiuni
     }
    } 
    else {
    // Dacă OverTime nu este "OFF", dezactivează încălzirea și resetează variabilele
    RelayHeaterState = "OFF";
    Serial.println("OverRunning Active");
    digitalWrite(RelayPINHeater, LOW);
    heatingActive = false;
    pauseActive = false;
    client.publish(MQTT_HEATING_STATE, String("notHeating").c_str(), true);    
    previousMillis = currentMillis;  // Actualizează timpul ultimei acțiuni
    overrunning = false;
  }
      // Oprirea încălzirii când demand devine false
    if (!demand && heatingActive) {
    RelayHeaterState = "OFF";
    //Serial.println("Thermostat OFF");
    digitalWrite(RelayPINHeater, LOW);
    heatingActive = false;
    pauseActive = false;
    client.publish(MQTT_HEATING_STATE, String("notHeating").c_str(), true);
    previousMillis = currentMillis;  // Actualizează timpul ultimei acțiuni
  }
}
//
void actuateHeating(int relayPin, const char* mqttTopic, bool demand, String& relayState,bool trvstatus1) {
  if (OverTime == "OFF") {
    pinMode(relayPin, OUTPUT);
    
    if (demand) {  
      relayState = "ON";
      if(trvstatus1){
      digitalWrite(relayPin, HIGH);
      }
      ///client.publish(mqttTopic, String(TRVON).c_str(), true);
      client.publish(mqttTopic, String("ON").c_str(), true);
      //client.publish(mqttTopic, String(TRVTARGET).c_str(), true);
      //Serial.println("relayState ON");
    } 
    else {
      relayState = "OFF";
      //Serial.println("OverRunning OFF");
      digitalWrite(relayPin, LOW);
      //client.publish(mqttTopic, String(TRVOFF).c_str(), true);
      client.publish(mqttTopic, String("OFF").c_str(), true);
    }
  } else {
    Serial.println("relayState =  OFF");
    relayState = "OFF";
    digitalWrite(relayPin, LOW);
    //client.publish(mqttTopic, String(TRVOFF).c_str(), true);
    client.publish(mqttTopic, String("OFF").c_str(), true);
  }
}
void ActuateHeating1(bool demand) {
  actuateHeating(RelayPIN1, mqtt_topic_TRV_setting_zone1, demand, RelayState1, TRV1STATUS);
}

void ActuateHeating2(bool demand) {
  actuateHeating(RelayPIN2, mqtt_topic_TRV_setting_zone2, demand, RelayState2, TRV2STATUS);
}

void ActuateHeating3(bool demand) {
  actuateHeating(RelayPIN3, mqtt_topic_TRV_setting_zone3, demand, RelayState3, TRV3STATUS);
}
///
void callback(char* topic, byte* payload, unsigned int length) { 
  payload[length] = '\0';
  String msg = String((char*)payload);
  String msg1 = String((char*)payload);
  //String msg1 = String((char*)payload);
  // Check TRV status 
  if (strcmp(topic, mqtt_topic_TRV_status_zone1) == 0) {
  updateTRVState(mqtt_topic_TRV_status_zone1, payload, length, TRV1);
  }
  else if (strcmp(topic, mqtt_topic_TRV_status_zone2) == 0) {
  updateTRVState(mqtt_topic_TRV_status_zone2, payload, length, TRV2);
  }
  else if (strcmp(topic, mqtt_topic_TRV_status_zone3) == 0) {
  updateTRVState(mqtt_topic_TRV_status_zone3, payload, length, TRV3);
  }
  else if  (strcmp(topic, mqtt_topic_TRV_mode_zone1) == 0) {
  handleTRVSTATUS(mqtt_topic_TRV_mode_zone1, payload, length, TRV1STATUS);
  }
  else if (strcmp(topic, mqtt_topic_TRV_mode_zone2) == 0) {
  handleTRVSTATUS(mqtt_topic_TRV_mode_zone2, payload, length, TRV2STATUS);
  }
  else if (strcmp(topic, mqtt_topic_TRV_mode_zone3) == 0) {
  handleTRVSTATUS(mqtt_topic_TRV_mode_zone3, payload, length, TRV3STATUS);
  }
  // Maintenance radiators
  else if (strcmp(topic, mqtt_topic_maintenance_radiator_1) == 0) {
  handleRadiator(mqtt_topic_maintenance_radiator_1, payload, length, radiator_1);
  }
  else if (strcmp(topic, mqtt_topic_maintenance_radiator_2) == 0) {
  handleRadiator(mqtt_topic_maintenance_radiator_2, payload, length, radiator_2);
  }  
  else if (strcmp(topic, mqtt_topic_maintenance_radiator_3) == 0) {
  handleRadiator(mqtt_topic_maintenance_radiator_3, payload, length, radiator_3);
  } 
  else if (strcmp(topic, MQTT_BRIDGE_STATE) == 0) {
     DynamicJsonBuffer jsonBuffer;
     JsonObject& root = jsonBuffer.parseObject(msg);
      if (!root.success()) {
      Serial.println("Eroare la parsarea JSON-ului_Bridge");
      return;
    }
    String mesaj = root["state"];
    if (mesaj == "online"){
        //Serial.println("Senzor_4 online ");
        bridgeState = true; 
      }
    if (mesaj == "offline"){
        //Serial.println("Senzor_4 offline ");
        bridgeState =  false;
      }

  }
  //temperature
  else if (strcmp(topic, mqtt_topic_temp_zone1) == 0) {
    handleTemperatureSensor(mqtt_topic_temp_zone1, payload, length, Temperature1mqtt);
  } 
  else if (strcmp(topic, mqtt_topic_temp_zone2) == 0) {
    handleTemperatureSensor(mqtt_topic_temp_zone2, payload, length, Temperature2mqtt);
  } 
  else if (strcmp(topic, mqtt_topic_temp_zone3) == 0) {
    handleTemperatureSensor(mqtt_topic_temp_zone3, payload, length, Temperature3mqtt);
  }
  //target
  else if (strcmp(topic, mqtt_topic_target_temp_zone1) == 0) {
    handleTargetTempRoom(mqtt_topic_target_temp_zone1, payload, length, TargetTempRoom1);
  }
  else if (strcmp(topic, mqtt_topic_target_temp_zone2) == 0) {
    handleTargetTempRoom(mqtt_topic_target_temp_zone2, payload, length, TargetTempRoom2);
  }
  else if (strcmp(topic, mqtt_topic_target_temp_zone3) == 0) {
    handleTargetTempRoom(mqtt_topic_target_temp_zone3, payload, length, TargetTempRoom3);
  }
  // histerises upper 
  else if (strcmp(topic, mqtt_boiler_histeresis_upper) == 0) {
    handlehisterises(mqtt_boiler_histeresis_upper, payload, length, histeresisup);
  }
  // histerises down 
  else if (strcmp(topic, mqtt_boiler_histeresis_down) == 0) {
    handlehisterises(mqtt_boiler_histeresis_down, payload, length, histeresisdown);
  }  
// Sensor_Name
  else if (strcmp(topic, mqtt_topic_name_zone1) == 0) {
    handleSenzor_name(mqtt_topic_name_zone1, payload, length, Senzor1_name);
  }
  else if (strcmp(topic, mqtt_topic_name_zone2) == 0) {
    handleSenzor_name(mqtt_topic_name_zone2, payload, length, Senzor2_name);
  }
  else if (strcmp(topic, mqtt_topic_name_zone3) == 0) {
    handleSenzor_name(mqtt_topic_name_zone3, payload, length, Senzor3_name);
  }
  ///
  else if (strcmp(topic, MQTT_SENSOR_CORRECTION_INTERNAL) == 0) {
     correction_internal = msg1.toFloat();
     //Serial.print("TargetTempRoom1: ");
     //Serial.println(TargetTempRoom1);
  }
  // boiler settings
  else if (strcmp(topic, mqtt_boiler_overruning_time) == 0) {
    handleOVERTIMERUNNINGTIME(mqtt_boiler_overruning_time, payload, length, OVERTIME_RUNNING);
  }
  else if (strcmp(topic, mqtt_boiler_running_time) == 0) {
    handleIntervalRunningTIME(mqtt_boiler_running_time, payload, length, intervalRunningOn);
  }
  else if (strcmp(topic, mqtt_boiler_pause_time) == 0) {
    handleIntervalRunningPauseTIME(mqtt_boiler_pause_time, payload, length, intervalRunningOff);
  }
/// Define maintenance mode
  else if (strcmp(topic, mqtt_topic_maintenance) == 0) {
     DynamicJsonBuffer jsonBuffer;
     JsonObject& root = jsonBuffer.parseObject(msg);
      if (!root.success()) {
      Serial.println("Eroare la parsarea JSON-ului_Bridge");
      return;
    }
    String mesaj = root["state"];
    if (mesaj == "on"){
        //Serial.println("Senzor_4 online ");
        maintenance = true; 
      }
    if (mesaj == "off"){
        //Serial.println("Senzor_4 offline ");
        maintenance = false;
        ActuateHeating(OFF);
        ActuateHeating1(OFF);
        ActuateHeating2(OFF);
        ActuateHeating3(OFF);
      }
  }
  // Define HVAC MODE
  else if (strcmp(topic, MQTT_HVAC_MODE) == 0) {
     DynamicJsonBuffer jsonBuffer;
     JsonObject& root = jsonBuffer.parseObject(msg);
      if (!root.success()) {
      Serial.println("Eroare la parsarea JSON-ului_Bridge");
      return;
    }
    String mesaj = root["state"];
    if (mesaj == "heatingon"){
        Serial.println("hvacmode  on ");
        hvacmode = true; 
      }
    if (mesaj == "heatingoff"){
        Serial.println("hvacmode off ");
        hvacmode = false;
      }
  }
    // Reset overruning state
  else if (strcmp(topic, MQTT_OVERRUNNING_RESET_TOPIC) == 0) {
     DynamicJsonBuffer jsonBuffer;
     JsonObject& root = jsonBuffer.parseObject(msg);
      if (!root.success()) {
      Serial.println("Eroare la parsarea JSON-ului_Bridge");
      return;
      }
    String mesaj = root["state"];
    if (mesaj == "reset"){
        //Serial.println("Senzor_1 online ");
        OverTime = "OFF";
        Serial.println("Overrunning reset"); 
      }
    } 
  else if (strcmp(topic, mqtt_topic_manual_heating) == 0) {
     DynamicJsonBuffer jsonBuffer;
     JsonObject& root = jsonBuffer.parseObject(msg);
      if (!root.success()) {
      Serial.println("Eroare la parsarea JSON-ului_Bridge");
      return;
    }
    String mesaj = root["state"];
    if (mesaj == "on"){
        //Serial.println("Senzor_4 online ");
        manual_heating = true; 
      }
    if (mesaj == "off"){
        manual_heating = false;
      }
  }
}

/// END CALLBACK

// define void maintenace_on
void maintenace_on() {
  if (maintenance) {
    //Serial.println("Maintenance_ON");
    //ActuateHeating(OFF);

    if (radiator_1) ActuateHeating1(ON);
    if (radiator_2) ActuateHeating2(ON);
    if (radiator_3) ActuateHeating3(ON);
    if (manual_heating) ActuateHeating(ON);

    if (!radiator_1) ActuateHeating1(OFF);
    if (!radiator_2) ActuateHeating2(OFF);
    if (!radiator_3) ActuateHeating3(OFF);
    if (!manual_heating) ActuateHeating(OFF);
  } 

}


void alarmsystem(){
if (areAllTRVsOn) {
  Serial.println("Toți senzorii și toate TRV-urile sunt online");
} else {
  for (int sensorNumber = 1; sensorNumber <= 3; ++sensorNumber) {
    bool currentSensorState;

    switch (sensorNumber) {
      case 1:
        currentSensorState = TRV1;
        break;
      case 2:
        currentSensorState = TRV2;
        break;
      case 3:
        currentSensorState = TRV3;
        break;
        break;
      default:
        currentSensorState = false; // O valoare implicită în cazul unui număr de senzor necunoscut
    }

    if (!currentSensorState) {
      Serial.print("Senzorul ");
      Serial.print(sensorNumber);
      Serial.println(" nu este online");
    }
  }

  if (!TRV1) {
    Serial.println("TRV1 nu este online");
  }
  if (!TRV2) {
    Serial.println("TRV2 nu este online");
  }
  if (!TRV3) {
    Serial.println("TRV3 nu este online");
  }
  if (!bridgeState) {
    Serial.println("Zigbee nu este online");
  }
 } 
}
//


void checkRoom(float roomTemp, float targetTemp, bool trv, unsigned long &lowTempSensorTime, unsigned long &lowLastReadTime, void (&actuateHeating)(bool), bool trvstatus, float histeresisdown) {
  if (roomTemp < targetTemp - histeresisdown && OverTime == "OFF" && roomTemp != 0 && trv) {
    //Serial.println("apelare checkRoom");
    lowTempSensorTime += millis() - lowLastReadTime;
    lowLastReadTime = millis();
    if (lowTempSensorTime >= TIME_SENSOR * 1000) {
      actuateHeating(ON);
      if(trvstatus){
        ActuateHeating(ON);
      }
    }
  } else {
    lowTempSensorTime = 0;
    lowLastReadTime = millis();
  }
}

void checkHighTempSensor(float roomTemp, float targetTemp, unsigned long &highTempSensorTime, unsigned long &highLastReadTime, void (&actuateHeating)(bool), bool trv, float histeresisup) {
  if (roomTemp > targetTemp + histeresisup || !trv ) {
    //Serial.println("apelare checkHighTempSensor");
    highTempSensorTime += millis() - highLastReadTime;
    highLastReadTime = millis();
    if (highTempSensorTime >= TIME_SENSOR * 1000) {
      actuateHeating(OFF);
    }
  } else {
    highTempSensorTime = 0;
    highLastReadTime = millis();
  }
}

void checkAllRelays() {
  if ((RelayState1 == "OFF" || !bridgeState || MQTT_CONNECT == "OFF") && (RelayState2 == "OFF" || !bridgeState || MQTT_CONNECT == "OFF") && (RelayState3 == "OFF" || !bridgeState || MQTT_CONNECT == "OFF")) {
    //Serial.println("apelare checkAllRelays");
    ActuateHeating(OFF);
    OverTime = "OFF";
    overrunning = false;
  }
}
void controlheat() {
  //unsigned long currentMillis = millis();
  //Serial.println("apelare controlheat");
  checkRoom( Temperature1, TargetTempRoom1, TRV1, lowTempSensorTime1, lowlastReadTime1, ActuateHeating1, TRV1STATUS, histeresisdown);
  checkRoom( Temperature2, TargetTempRoom2, TRV2, lowTempSensorTime2, lowlastReadTime2, ActuateHeating2, TRV2STATUS, histeresisdown);
  checkRoom( Temperature3, TargetTempRoom3, TRV3, lowTempSensorTime3, lowlastReadTime3, ActuateHeating3, TRV3STATUS, histeresisdown);

  checkHighTempSensor(Temperature1, TargetTempRoom1, highTempSensorTime1, highlastReadTime1, ActuateHeating1, TRV1, histeresisup);
  checkHighTempSensor(Temperature2, TargetTempRoom2, highTempSensorTime2, highlastReadTime2, ActuateHeating2, TRV2, histeresisup);
  checkHighTempSensor(Temperature3, TargetTempRoom3, highTempSensorTime3, highlastReadTime3, ActuateHeating3, TRV3, histeresisup);

  checkAllRelays();
}

void hvacmodevoid() {
  if (hvacmode && !maintenance) { 
   controlheat();
   //Serial.println("hvacmodevoid este ON");
  }
 else if (!hvacmode && !maintenance) {
  //Serial.println("hvacmodevoid este OFF");
  ActuateHeating(OFF);
  ActuateHeating1(OFF);
  ActuateHeating2(OFF);
  ActuateHeating3(OFF);
  overrunning = false;
  OverTime = "OFF";
  }
}
void OverRunning() {
  if (!maintenance){
  unsigned long currentMillis = millis();
  if (overrunning) {
   // Serial.println("ActuateHeating este ON");
   // Increment a counter variable
    ///unsigned long overRuningMillis = OVERTIME_RUNNING * 60 * 1000UL; //transformam in minute
    overRunningTime += currentMillis - overRunningLastReadTime;  // Increment by the time elapsed since the last reading
    overRunningLastReadTime = currentMillis;  // Update the last read time
    // Check if the counter variable has reached 15 minutes (900,000 milliseconds)
    if (overRunningTime >= OVERTIME_RUNNING * 60 * 1000UL){    // Time in minutes
      //Serial.println("A mers prea mult!");
      OverTime = "ON";
      ActuateHeating(OFF);
    }
  }

else {
    // Reset the counter variable if the analog value falls below the threshold
    overRunningTime = 0;
    overRunningLastReadTime = currentMillis;  // Update the last read time
    //Serial.println("Resetam OverRunning");
  }
 }
}
void system_alarm() {
  pinMode(PINAlarm, OUTPUT);

  if (Temperature1 == 0.00 || Temperature2 == 0.00 || Temperature3 == 0.00 || TempInternal == 0.00 ||
     !bridgeState || !TRV1 || !TRV2 || !TRV3 || OverTime == "ON") {
    digitalWrite(PINAlarm, HIGH);
    SytemAlarm = "ON";
  } else {
    digitalWrite(PINAlarm, LOW);
    SytemAlarm = "OFF";
  }
}

void publishData(float p_temperatureintern) {
  // create a JSON object
  // doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["TempInternal"] = (String)p_temperatureintern;
  //root["temperature2"] = (String)p_temperature2;
  //root["temperature3"] = (String)p_temperature3;
  //root.prettyPrintTo(Serial);
  //Serial.println("");
  /*
     {
        "temperature": "23.20" ,
        "humidity": "43.70"
     }
  */
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(MQTT_SENSOR_INTERNAL, data, true);
  yield();
}

void readsensor(){
  float temperature = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    //temperature = 0;
    return;
  }

  TempInternal = temperature;
  TempInternal = TempInternal + (correction_internal);
  /*
  Vo = analogRead(sensorInternal);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  TempInternal = T - 273.15;
  TempInternal = TempInternal + (correction1);
   
  TempInternal = 1 / (log(1 / (1023. / analogRead(sensorInternal) - 1)) / BETA + 1.0 / 298.15) - 273.15;
  TempInternal = TempInternal + (correction1);
  ///
  double temp = Thermister(analogRead(sensorInternal));  // Read sensor
  TempInternal = temp + (correction1);
  */
}

void printtemp(){
  Serial.print("Temperature_1 " + String(Senzor1_name) + ": ");
  Serial.print(Temperature1);
  Serial.println((" °C"));
  Serial.print("Temperature_2 " + String(Senzor2_name) + ": ");
  Serial.print(Temperature2);
  Serial.println((" °C"));
  Serial.print("Temperature_3 " + String(Senzor3_name) + ": ");
  Serial.print(Temperature3);
  Serial.println((" °C"));
  Serial.print("TempInternal : ");
  Serial.print(TempInternal);
  Serial.println((" °C"));
// print target temp  
  Serial.print("Target 1 " + String(Senzor1_name) + ": ");
  Serial.print(TargetTempRoom1);
  Serial.println((" °C"));
  Serial.print("Target 2 " + String(Senzor2_name) + ": ");
  Serial.print(TargetTempRoom2);
  Serial.println((" °C"));
  Serial.print("Target 3 " + String(Senzor3_name) + ": ");
  Serial.print(TargetTempRoom3);
  Serial.println((" °C"));
}

void loop () {
  
  unsigned long currentMillis = millis();  // folosim sa luam ceasul actual intern
  reconnect(); //folosim fara delay in caz ca se deconecteaza
  client.loop();  //folosim fara delay
  if (currentMillis - previousMillis1  >= SAMPLE_LOOP_TIME1 * 1000) { 
    previousMillis1  = millis();
  //alarmsystem();
    printtemp();
    readsensor();
    publishData(TempInternal);
    ArduinoOTA.handle();   
  }
  if (currentMillis - previousMillis2 >=  SAMPLE_LOOP_TIME2 * 1000) {
    previousMillis2 = millis();
    checkSensor();
    StartUp_ESP();
    client.publish(MQTT_OVERRUNNING_TOPIC, String(OverTime).c_str(), true);    
  }
  if (currentMillis - previousMillis3 >=  SAMPLE_LOOP_TIME3 * 1000) {
    previousMillis3 = millis(); 
    client.publish(status_esp_keepalive, String("online").c_str(), true);
    system_alarm();
    alarmsystem();
    
  }
  if (currentMillis - previousMillis4 >=  SAMPLE_LOOP_TIME4 * 1000) {
    previousMillis4  = millis();
    OverRunning();
    hvacmodevoid();
 
    //controlheat();
    maintenace_on();
    
  }
  if (restartNow){
    Serial.println("Restart");
    ESP.restart();
  }
  yield();
}
String WiFiSignal() {
  float Signal = WiFi.RSSI();
  Signal = 90 / 40.0 * Signal + 212.5;
  if (Signal > 100) Signal = 100;
  return " " + String(Signal, 0) + "%";
}
// Function for upload firmware file via webpage
static void handle_update_progress_cb(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  uint32_t free_space = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
  if (!index){
    Serial.println("Update");
    Update.runAsync(true);
    if (!Update.begin(free_space)) {
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }

  if (final) {
    if (!Update.end(true)){
      Update.printError(Serial);
    } else {
      restartNow = true;//Set flag so main loop can issue restart call
      Serial.println("Update complete");
    }
  }
}
///Function called to reboot system
void RebootSystem() {
  restartNow = true;
}
// Function called to delete wifi data 
void handleWiFiReset(){
  wifiManager.resetSettings();
}
///
//######################### HTTP section
// Header page and style
void append_HTML_header(bool refreshMode) {
  webpage = "<!DOCTYPE html>";
  webpage += "<html lang=\"en\">";
  webpage += "<head>";
  webpage += "<meta charset=\"UTF-8\">";
  webpage += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  webpage += "<title>SmartThermostat</title>";
   if (refreshMode) webpage += F("<meta http-equiv='refresh' content='15'>");
  webpage += "<style>";
  webpage += "	   body {font-family: Arial, Helvetica,display: flex; justify-content: center; align-items: center; sans-serif;  margin: 0;}";
  webpage += "	   table tr, td     {padding:0.2em 0.5em 0.2em 0.5em;font-size:1.0em;font-family:Arial,Helvetica,sans-serif;text-align:center;}";
  webpage += "h2               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.4em;text-align:center;}";
  webpage += "h3               {margin-top:0.3em;margin-bottom:0.3em;font-size:1.2em;text-align:center;}";
  webpage += "h4               {margin-top:0.3em;margin-bottom:0.3em;font-size:0.8em;}";
  webpage += ".centre          {margin-left:auto;margin-right:auto;text-align:right;}";
  webpage += "table.timer tr {padding:0.2em 0.5em 0.2em 0.5em;font-size:1.1em;font-family:Arial,Helvetica,sans-serif;blue;border: 1px solid black;}";
  webpage += "table.timer td {padding:0.2em 0.5em 0.2em 0.5em;font-size:1.1em;font-family:Arial,Helvetica,sans-serif;blue;border: 1px solid black;}";
  //  webpage += "	   col:first-child  {background:lightcyan}col:nth-child(2){background:#CCC}col:nth-child(8){background:#CCC}";
  // webpage += "	   tr:first-child   {background:lightcyan}";
  webpage += ".navbar {  overflow: hidden;  background-color: #333;}";
  webpage += ".navbar a {float: left;font-size: 16px;  color: white;  text-align: center;  padding: 14px 16px;  text-decoration: none;}";
  webpage += ".subnav {  float: left;  overflow: hidden;}";
  webpage += ".subnav .subnavbtn {  font-size: 16px;border: none;  outline: none;  color: white;  padding: 14px 16px;  background-color: inherit;  font-family: inherit;  margin: 0;}";
  webpage += ".navbar a:hover, .subnav:hover .subnavbtn {  background-color: red;}";
  webpage += ".subnav-content {  display: none;position: absolute;  left: 0;  background-color: red;  width: 100%;  z-index: 1;}";
  webpage += ".subnav-content a {  float: left;color: white;  text-decoration: none;}";
  webpage += ".subnav-content a:hover {  background-color: #eee;  color: black;}";
  webpage += ".subnav:hover .subnav-content {  display: block;}";
  webpage += "	   table.tabela1  {font-family: Arial, Helvetica, sans-serif; width:auto;margin-left:left;margin-right:auto;}";
  webpage += "	  .tabela1 th {border: 2px solid black;border-radius: 10px;padding:0.2em 0.5em 0.2em 0.5em; text-align:center;width:auto;}";
  webpage += "	   table.tabela2  {font-family: Arial, Helvetica, sans-serif;width:auto;margin-left:auto;margin-right:auto;}";
  webpage += "	  .tabela2 th {border:2px solid black;border-radius: 10px;padding:0.2em 0.5em 0.2em 0.5em;font-size:1.0em;background-color:lightgray;text-align:center;width:auto;}";
  webpage += "	   table.tabela3  {font-family: Arial, Helvetica, sans-serif;width:auto;margin-left:auto;margin-right:auto;}";
  webpage += "	  .tabela3 th {border: 2px solid black;border-radius: 10px;padding:0.2em 0.5em 0.2em 0.5em;background-color:lightgray;text-align:center;width:auto;}";
  webpage += "	  .tabela3 td  {border-radius: 10px;padding:0.2em 0.5em 0.2em 0.5em;text-align:center;color:#f2f2f2}";
  webpage += "	   table.tabela4  {font-family: Arial, Helvetica, width:auto; margin-left:auto;margin-right:auto;}";
  webpage += "	  .tabela4 th {border: 2px solid black;border-radius: 10px;padding:0.2em 0.5em 0.2em 0.5em;background-color:lightgray;text-align:center;width:auto;}";
  webpage += "    .onthermostat   {width:20%;color:red;}";
  webpage += "    .offthermostat  {width:20%;color:green;}";
  webpage += ".tabela1 .on {background-color: red;}";
  webpage += ".tabela1 .off {background-color: limegreen;}";
  webpage += ".tabela1 .faulton {background-color: lightgray;}";
  webpage += ".tabela3 .on {background-color: #EC8B36;}";
  webpage += ".tabela3 .off {background-color: #B0DC84;}";
  webpage += ".tabela4 .on {background-color: #EC8B36;}";
  webpage += ".tabela4 .off {background-color: #B0DC84;}";
  // define footer style
  webpage += "footer           {padding:0.08em;background-color:#000;font-size:1.1em;text-align: center;position: fixed; bottom: 0;  width: 100%;color:white;}";
  webpage += ".medium          {font-size:1.4em;padding:0;margin:0}";
  webpage += ".ps              {font-size:0.7em;padding:0;margin:0}";
  webpage += "</style>";
  webpage += "</head>";
  webpage += "<body>";
  webpage += "<div class=\"navbar\">";
  webpage += "<a href=\"homepage\">Home</a>";
  webpage += "<div class=\"subnav\">";
  webpage += "<button class=\"subnavbtn\">System<i class=\"fa fa-caret-down\"></i></button>";
  webpage += "<div class=\"subnav-content\">";
  webpage += "<a href=\"sysinfo\">System-Info</a>";
  webpage += "<a href=\"systemreboot\">Sytem Reboot</a>";
  webpage += "<a href=\"softwareupgrade\">Software Upgrade</a>";
  webpage += "<a href=\"resetwifi\">WiFi-Data-Reset</a>";
  webpage += "</div>";
  webpage += "</div> ";
  webpage += "</div>";
}
void append_HTML_footer() {
  webpage += "<!-- AICI este bara de jos-->";
  webpage += "<footer>";
  webpage += "<p class='medium'>WiFi Smart Thermostat</p>";
  webpage += "<p class='ps'><i>Copyright &copy; Teodoran_Duna</i></p>";
  webpage += "</footer>";
  webpage += "</body></html>";
}

void SysInfo() {
  append_HTML_header(noRefresh);
  webpage += "<h3>Sytem Information</h3><br> ";
  webpage += "<table class=\"centre timer\"> ";
  webpage += "<tr> ";
  webpage += "<td>Data</td><td>Value</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>SW Version: </td>";
  webpage += "<td>" + String(sw_version) + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>Model Type: </td>";
  webpage += "<td>" + String(model) + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>WiFi SSID: </td>";
  webpage += "<td>" + String(WiFi.SSID()) + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>IP Address: </td>";
  webpage += "<td>" + WiFi.localIP().toString() + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>WiFi Signal: </td> ";
  webpage += "<td>" + WiFiSignal() + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>HAS Hostname: </td>";
  webpage += "<td>" + String(mqtt_server) + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>HAS IP: </td>";
  webpage += "<td>" + String(serverIP.toString().c_str()) + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>MQTT User: </td>";
  webpage += "<td>" + String(mqtt_user) + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>MQTT PASS: </td> ";
  webpage += "<td>" + String(mqtt_password) + "</td>";
  webpage += "</tr> ";
  webpage += "<tr> ";
  webpage += "<td>Running Time (min): </td> ";
  webpage += "<td>" + String(intervalRunningOn) + "</td>";
  webpage += "</tr> ";
  webpage += "<td>Pause Time (min): </td> ";
  webpage += "<td>" + String(intervalRunningOff) + "</td>";
  webpage += "</tr> ";
  webpage += "<td>OverRunning Time (min): </td> ";
  webpage += "<td>" + String(OVERTIME_RUNNING) + "</td>";
  webpage += "</tr> ";
 // histerises 
  webpage += "<td>Histerises upper limit: </td> ";
  webpage += "<td>" + String(histeresisup) + "</td>";
  webpage += "</tr> ";
   webpage += "<td>Histerises lower limit: </td> ";
  webpage += "<td>" + String(histeresisdown) + "</td>";
  webpage += "</tr> ";
// Test histeresis
  webpage += "<td>Histerises upper target: </td> ";
  webpage += "<td>" + String(TargetTempRoom1 + histeresisup) + "</td>";
  webpage += "</tr> ";
   webpage += "<td>Histerises lower target: </td> ";
  webpage += "<td>" + String(TargetTempRoom1 - histeresisdown) + "</td>";
  webpage += "</tr> ";
  webpage += "</table>";
  append_HTML_footer();
}
// Sytem reboot webpage
void SytemReboot() {
    append_HTML_header(noRefresh);
    webpage += "<div style='text-align: center;'>"; // Centrarea textului
    webpage += "<h3>Press Reboot Button to restart system</h3>";
    webpage += "</div>";
    webpage += "<div style='text-align: center;'>"; // Centrarea butonului
    webpage += "<button onclick=\"RebootArduino()\">Reboot</button>";
    webpage += "<script>";
    webpage += "function RebootArduino() {";
    webpage += "  var xhr = new XMLHttpRequest();";
    webpage += "  xhr.open('GET', '/RebootSystemArduino', true);";
    webpage += "  xhr.send();";
    webpage += "  setTimeout(function() { window.location.href = '/homepage'; }, 10000);"; // Direcționare către pagina homepage după 10 secunde
    webpage += "}";
    webpage += "</script>";
    append_HTML_footer();
}
// Software upgrade page
void SoftwareUpgrade() {
    append_HTML_header(noRefresh);
    webpage += "<style>";
    webpage += "#uploadform { text-align: center; }"; // Aliniază formularul în centru
    webpage += "#fileupload { margin: 0 auto; display: block; }"; // Aliniază caseta de browse în centru
    webpage += "#submit { margin-top: 10px; }"; // Adaugă un spațiu între casetă și buton
    webpage += "</style>";
    webpage += "<h3>Please browse .bin file</h3>";
    webpage += "<form id=\"uploadform\" enctype=\"multipart/form-data\" method=\"post\" action=\"/upload\">";
    webpage += "<input id=\"fileupload\" name=\"inobinfile\" type=\"file\" />";
    webpage += "<input type=\"submit\" value=\"Submit\" id=\"submit\" />";
    webpage += "</form>";
    webpage += "<script>";
    webpage += "document.getElementById('uploadform').addEventListener('submit', function(event) {";
    webpage += "  event.preventDefault();"; // Oprire trimitere formular
    webpage += "  var xhr = new XMLHttpRequest();";
    webpage += "  xhr.open('POST', '/upload', true);";
    webpage += "  xhr.onload = function() {";
    webpage += "    if (xhr.status === 200) {";
    webpage += "      setTimeout(function() { window.location.href = '/homepage'; }, 10000);"; // Direcționare către pagina homepage după 10 secunde
    webpage += "    } else {";
    webpage += "      alert('Eroare la încărcare');"; // Afișare alertă în caz de eroare
    webpage += "    }";
    webpage += "  };";
    webpage += "  xhr.send(new FormData(this));"; // Trimitere fișier prin POST
    webpage += "});";
    webpage += "</script>";
    append_HTML_footer();
}


// Wifi data reset webpage
void WifiReset() {
  append_HTML_header(noRefresh);
  webpage += "<div style='text-align: center;'>"; // Centrarea textului
  webpage += "<h3>Press Reset Data Button to reset Wifi Credentials</h3>";
  webpage += "</div>";
  webpage += "<div style='text-align: center;'>"; // Centrarea butonului
  webpage += "<button onclick=\"ResetWifi()\">Reset Data</button>";
  webpage += "<script>";
  webpage += "function ResetWifi() {";
  webpage += "  window.location.href = '/homepage';";
  webpage += "  var xhr = new XMLHttpRequest();";
  webpage += "  xhr.open('GET', '/ResetWiFiData', true);";
  webpage += "  xhr.send();";
  webpage += "}";
  webpage += "</script>";
  append_HTML_footer();
}
// Home page
void Homepage() { 
  append_HTML_header(Refresh);
  webpage += "<br>";
  webpage += "<table class=\"tabela1\">";
  webpage += "<tr>";
  webpage += "<th class=" + String((SytemAlarm == "ON" ? "'on'>" : "'off'>")) + "System Alarm" + "</th>";
  webpage += "</tr>";
  webpage += "<tr>";
  webpage += "<th class=" + String((OverTime == "ON" ? "'on'>" : "'off'>")) + "OverRunning Alarm" + "</th>";
  webpage += "</tr>";
  webpage += "<th class=" + String((MQTT_CONNECT == "OFF" ? "'on'>" : "'off'>")) + "MQTT Server Alarm" + "</th>";
  webpage += "</tr>";
  webpage += "<th class=" + String((SensorsConnected == "OFF" ? "'on'>" : "'off'>")) + "Sensors Alarm" + "</th>";
  webpage += "</tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "<table class=\"tabela2\">";
  webpage += "<tr>";
  webpage += "<th>Temperature " + String(Senzor1_name) + ": " + String((Temperature1)) + " &deg;</th>";
  webpage += "<th>Temperature " + String(Senzor2_name) + ": " + String((Temperature2)) + " &deg;</th>";
  webpage += "<th>Temperature " + String(Senzor3_name) + ": " + String((Temperature3)) + " &deg;</th>";
  webpage += "</tr>";
  webpage += "<tr>";
  webpage += "<th>Target Temp " + String(Senzor1_name) + ": " + String((TargetTempRoom1)) + " &deg;</th>";
  webpage += "<th>Target Temp " + String(Senzor2_name) + ": " + String((TargetTempRoom2)) + " &deg;</th>";
  webpage += "<th>Target Temp " + String(Senzor3_name) + ": " + String((TargetTempRoom3)) + " &deg;</th>";
  webpage += "</tr>";
  webpage += "</table>";
  webpage += "<table class=\"tabela4\">";
  webpage += "<tr>";
  webpage += "<th>Internal sensor: " + String(TempInternal) + " &deg;</th>";
  webpage += "</tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "<br>";
  webpage += "<br>";
  webpage += "<table class=\"tabela3\">";
  webpage += "<tr class=\"tabela3\">";
  // radiator 1
  if ( TRV1STATUS)  {
    webpage +="<th class=\"on\"> Radiator " + String(Senzor1_name) + "</th>";
  } 
  else if (!TRV1STATUS) {
    webpage +="<th class=\"off\"> Radiator " + String(Senzor1_name) + "</th>";
  }
  if (!TRV1) {
    webpage +="<th class=\"faulton\"> Radiator " + String(Senzor1_name) + "</th>";
  }
// radiator 2
  if (TRV2STATUS)  {
    webpage +="<th class=\"on\"> Radiator " + String(Senzor2_name) + "</th>";
  } 
  else if (!TRV2STATUS) {
    webpage +="<th class=\"off\"> Radiator " + String(Senzor2_name) + "</th>";
  }
  if (!TRV2) {
    webpage +="<th class=\"faulton\"> Radiator " + String(Senzor2_name) + "</th>";
  }
  // radiator 3
  if (TRV3STATUS)  {
    webpage +="<th class=\"on\"> Radiator " + String(Senzor3_name) + "</th>";
  } 
  else if (!TRV3STATUS) {
    webpage +="<th class=\"off\"> Radiator " + String(Senzor3_name) + "</th>";
  }
  if (!TRV3) {
    webpage +="<th class=\"faulton\"> Radiator " + String(Senzor3_name) + "</th>";
  }
///webpage += "<th class=" + String((RelayState1 == "ON" ? "'on'>" : "'off'>")) + "Radiator " + String(Senzor1_name) +  "</th>";
///webpage += "<th class=" + String((RelayState2 == "ON" ? "'on'>" : "'off'>")) + "Radiator " + String(Senzor2_name) +  "</th>";
///webpage += "<th class=" + String((RelayState3 == "ON" ? "'on'>" : "'off'>")) + "Radiator " + String(Senzor3_name) +  "</th>";
//webpage += "<th>Radiator " + String(Senzor1_name) + "</th>";
//webpage += "<th>Radiator " + String(Senzor2_name) + "</th>";
//webpage += "<th>Radiator " + String(Senzor3_name) + "</th>";
  webpage += "</tr>";
  webpage += "</table>";
  webpage += "<br>";
  webpage += "<table class=\"tabela4\">";
  webpage += "<tr class=\"tabela4\">";
  webpage += "<th class=" + String((RelayHeaterState == "ON" ? "'on'>" : "'off'>")) + "Heating" +  "</th>";
  webpage += "</tr>";
  webpage += "</table>";
  ///webpage += "<FORM action=\"/handleroom\"> ";
  ///webpage += "<input  style=\"margin: 0;  position: absolute; left: 50%;  -ms-transform: translate(-50%, -50%);  transform: translate(-50%, -50%);\" type=\"submit\" value=\"Save data\"><br><br> ";
  ///webpage += "</form>";
  webpage += "</body>";
  webpage += "</html>";
  append_HTML_footer();
}
// Function to call webpages 
void serverInit(){
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->redirect("/homepage");       // Go to home page
  });
  server.on("/homepage", HTTP_GET, [](AsyncWebServerRequest * request) {
    Homepage();
    request->send(200, "text/html", webpage);
  });
  server.on("/sysinfo", HTTP_GET, [](AsyncWebServerRequest * request) {
  SysInfo();
    request->send(200, "text/html", webpage);
  });
  // Reboot system section
  server.on("/systemreboot", HTTP_GET, [](AsyncWebServerRequest * request) {
  SytemReboot();
    request->send(200, "text/html", webpage);
  });
  server.on("/RebootSystemArduino", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // Apelează funcția Arduino dorită aici (myFunction())
    // De exemplu: myFunction();
    RebootSystem();
    request->send(200, "text/plain", "Funcția Arduino a fost apelată");
});
  //Firmware upload Webpage
  server.on("/softwareupgrade", HTTP_GET, [](AsyncWebServerRequest * request) {
  SoftwareUpgrade();
    request->send(200, "text/html", webpage);
  });
  //Firmware upload file 
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
      request->send(200);
    }, handle_update_progress_cb);
  //
//////////////////////////
  server.on("/resetwifi", HTTP_GET, [](AsyncWebServerRequest * request) {
  WifiReset();
    request->send(200, "text/html", webpage);
  });
  server.on("/ResetWiFiData", HTTP_GET, [] (AsyncWebServerRequest *request) {
    // Apelează funcția Arduino dorită aici (myFunction())
    // De exemplu: myFunction();
    handleWiFiReset(); 
    request->send(200, "text/plain", "Funcția Arduino a fost apelată");
});

// Error page , not found
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404);
  });
  server.begin();
}

void setup() {
  Serial.begin(115200);
  //Setup_wifi();
  pinMode(PINWifi, OUTPUT);
  pinMode(RelayPIN1, OUTPUT);
  pinMode(RelayPIN2, OUTPUT);
  pinMode(RelayPIN3, OUTPUT);
  pinMode(RelayPINHeater, OUTPUT);
  digitalWrite(RelayPINHeater, LOW);
  digitalWrite(RelayPIN1, LOW);
  digitalWrite(RelayPIN2, LOW);
  digitalWrite(RelayPIN3, LOW);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH); 
  AsyncWiFiManager wifiManager(&server,&dns);
  wifiManager.autoConnect("SmartThermostatAP");
  serverInit();
  // Start find HA IP 
  if (WiFi.hostByName(mqtt_server, serverIP)) {
      Serial.print("IP address of ");
      Serial.print(mqtt_server);
      Serial.print(": ");
      Serial.println(serverIP);
  } else {
      Serial.println("Failed to get IP address from DNS");
  }
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(PINWifi, HIGH);
  }
  else {
    digitalWrite(PINWifi, LOW);    
  }
  //Serial.print(host);
  //Serial.print(": ");
  //Serial.println(serverIP);
  // end find HA IP 
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //Define relay pins
  dht.begin();
  client.publish(MQTT_STARTUP_STATE, String("started").c_str(), false);

  /// OTA definition
    ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.setHostname("ESP_1");
  ArduinoOTA.begin();
  // END OTA
  // publish local ip 
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Set hostname of ESP
  SetupDeviceName(ServerName); // Used to setup ESP hostname
  publishIP(); // Used to publish  ESP IP 

}
