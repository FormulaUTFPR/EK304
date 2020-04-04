#include <Arduino_FreeRTOS.h>
#include <queue.h>

#define BOTAO 2

TaskHandle_t eventoLed;
TaskHandle_t eventoBotao;
QueueHandle_t filaBotao;

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BOTAO, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(BOTAO), botaoPressionado, FALLING);

  filaBotao = xQueueCreate(10, sizeof(int));
  
  xTaskCreate(taskLed, "Blink", 128, NULL, 2, &eventoLed);
  xTaskCreate(taskTeclado, "Teclado", 128, NULL, 3, &eventoBotao);
}

void loop()
{
}

void taskLed(void* pvParameters)
{
  int valor;
  
  while (true)
  {
    if(xQueueReceive(filaBotao, &valor, portMAX_DELAY) == pdPASS)
    {
      Serial.println(valor);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void taskTeclado(void* pvParameters)
{
  int contador = 0;

  while(true)
  {
    if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
    {
      contador++;
      xQueueSend(filaBotao, &contador, portMAX_DELAY);
      Serial.println("Botao pressionado!");
      taskDelay(1000);
    }
  }
}

void taskDelay(int ms)
{
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

void botaoPressionado()
{
  BaseType_t  xHigherPiorityTaskWoken = pdFALSE;
  Serial.println("Interrpcao");
  
  vTaskNotifyGiveFromISR(eventoBotao, &xHigherPiorityTaskWoken);
}
