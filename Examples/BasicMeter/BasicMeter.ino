#include <PowerMeter.h>
SemaphoreHandle_t semaforo;
TaskHandle_t powerMeterTask;

PowerMeter powerMeter;

uint32_t t0 = 0;

void powerMeasureFunction(void *vParameters);

void setup() {
  //Start serial
  Serial.begin(115200);

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
  if (millis() - t0 > 1000) {
    t0 = millis();
    xSemaphoreTake(semaforo, portMAX_DELAY);
    Serial.println(powerMeter.getMeasure().toString());
    xSemaphoreGive(semaforo);
  }
}
