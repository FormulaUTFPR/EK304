#include <Arduino_FreeRTOS.h>

#define BOTAO 2

TaskHandle_t eventoLed;

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BOTAO, INPUT_PULLUP);
  xTaskCreate(taskLed, "Blink", 128, NULL, 2, &eventoLed);
  xTaskCreate(taskTeclado, "Teclado", 128, NULL, 3, NULL);
}

void loop()
{

}

void taskLed(void* pvParameters)
{
  while (true)
  {
    if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void taskTeclado(void* pvParameters)
{
  while(true)
  {
    if(!digitalRead(BOTAO))
    {
      Serial.println("Botao pressionado!");
      xTaskNotifyGive(eventoLed);
      taskDelay(1000);
    }
    taskDelay(200);
  }
}

void taskDelay(int ms)
{
  vTaskDelay(ms / portTICK_PERIOD_MS);
}
