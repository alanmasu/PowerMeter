#include <MQTTManager.h>
#include <WebServer.h>
#include <PowerMeter.h>

WiFiClient espClient;
PubSubClient client(espClient);
const char* mosquittoServer = "test.mosquitto.org";
String topics[2] = {"test/topic1",
                    "test/topic2"};

WebServer server(80);

MQTTManager connectionManager;

const char* ssid = "YOUR_SSID";
const char* pass = "YOUR_PASS";

//PowerMeter
SemaphoreHandle_t semaforo;
TaskHandle_t powerMeterTask;

PowerMeter powerMeter;

uint32_t t0 = 0;

void powerMeasureFunction(void *vParameters);


void rebootCallback() {
  Serial.println("Rebooot timeee!!...");
}

void callback(char* topic, byte* message, unsigned int length) {

  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");

  char messageChar[length];
  String messageTemp;
  String toSend;
  char toSendChar[12];

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
    messageChar[i] += (char)message[i];
  }
  Serial.println();

}

void setup() {
  //Start serial
  Serial.begin(115200);

  //Creatin a variable to store version of the project
  String ver = String(__FILE__) + " Time: " + String(__DATE__) + " " + String(__TIME__);
  Serial.println("Version: " + ver);

  //Setting version on WebServer /IP_ADD/info
  connectionManager.setVersion(ver);
  //Setting server
  // true -> whitDefaultHomePage
  // set olso /IP_ADD/reboot for call rebootCallback and after reboots the core
  //      and /IP_ADD/rebootOnly thats reboot ESP32 whitout calling callBack
  connectionManager.setServer(&server, true);
  //Connect to a WiFi for the first time
  connectionManager.startWiFi(ssid, pass);
  //For enable WPS may use:
  //connectionManager.startConnection(bool whitWPS = true);

  //Start WebServer and OTA
  connectionManager.startWebServer();
  connectionManager.setOTAHostname("ESP32-Wattmetro");
  connectionManager.startOTA();

  //Setting rebootCallback and reboot options
  connectionManager.setOnRebootCallback(rebootCallback);
  connectionManager.setRebootOptions(false, true, false, true);

  //Print the hostname
  Serial.println("You can reach me also at: " + connectionManager.getOTAHostname() + ".local/");
  
  //Setting MQTT server
  connectionManager.setMQTTServer(&client, "ESP32-Wattmetro", mosquittoServer);
  connectionManager.setTopics(topics, 2);
  connectionManager.setCallback(callback);
  connectionManager.startMQTT();

  //PowerMeter
  //Printing the ADC values
  powerMeter.printADCValues();
  Serial.println();

  //Setting offsets
  //powerMeter.setOffsets();

  //Disable debug
  powerMeter.setDebug(false);

  //Creating a variable to store version of the project
  String ver = String(__FILE__) + " Time: " + String(__DATE__) + " " + String(__TIME__);
  Serial.println("Version: " + ver);

  //Creating Mutex
  semaforo = xSemaphoreCreateMutex();

  //Creating measure task
  xTaskCreatePinnedToCore(
    powerMeasureFunction,   /* Task function. */
    "MEASURE TASK",         /* name of task. */
    10000,                  /* Stack size of task */
    NULL,                   /* parameter of the task */
    1,                      /* priority of the task */
    &powerMeterTask,       /* Task handle to keep track of created task */
    1);
}

void powerMeasureFunction(void *vParameters) {
  while (true) {
    xSemaphoreTake(semaforo, portMAX_DELAY);
    powerMeter.loop();
    xSemaphoreGive(semaforo);
  }
  vTaskDelete(NULL);
}

void loop() {
  //this update everything, connection LED (default on-board led), connection BUTTON (default on-board BOOT button), servers... 
  //ATTENTION: in case of MQTT connection falliture can block the prosess for several seconds
  connectionManager.loop();

  //Sending data to MQTT
  if (millis() - t0 > 1000) {
    t0 = millis();
    xSemaphoreTake(semaforo, portMAX_DELAY);
    Measure values = powerMeter.getMeasure();
    xSemaphoreGive(semaforo);
    Serial.println(values.toString());
    sendViaMQTT(&values);
  }
}

void sendViaMQTT(Measure *values){
  char str[10];
	sprintf(str, "%.2f", values->rmsVoltage);
	client.publish("wattmetro/RMSvoltage", str);
	sprintf(str, "%.2f", values->realPower);
	client.publish("wattmetro/realPower", str);
	sprintf(str, "%.2f", values->rmsCurrent);
	client.publish("wattmetro/RMScurrent", str);
	sprintf(str, "%.2f", values->realPowerSec);
	client.publish("wattmetro/realPowerSec", str);
	sprintf(str, "%.2f", values->rmsCurrentSec);
	client.publish("wattmetro/RMScurrentSec", str);
}
