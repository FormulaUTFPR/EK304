////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome do projeto: EK304.ECU03                                                                                                       //
// Desenvolvido por: Carlos Modinez e Renan Cintra                                                                                    //
// Data/versão: 13/11/2019 (v0.0.1)                                                                                                   //
// IDE utilizada: Visual Studio Code                                                                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************************************************/
/* CONFIGURAÇÕES DO PROJETO                                                                                                           */
/**************************************************************************************************************************************/
/*Posicao 0 e 1 do CAN_MSG = sensor de temperatura
 * Posicao 2 e 3 do CAN_MSG = sensor de pressao
 * Posicao 4 do CAN_MSG = posicao da marcha
 */

/**************************************************************************************************************************************/
/* DEFINIÇÃO DE PINOS                                                                                                                 */
/**************************************************************************************************************************************/
#define PRESSAO_DO_AR A0     //Analógico
#define TEMPERATURA_DO_AR A1 //Analogica - Verificar a porta que será utilizada

//Defines do sensor do indicador da marcha

#define INDICADOR_DA_PRIMEIRA_MARCHA 2
#define INDICADOR_DA_SEGUNDA_MARCHA 3
#define INDICADOR_DA_TERCEIRA_MARCHA 4
#define INDICADOR_DA_QUARTA_MARCHA 5
#define INDICADOR_DA_QUINTA_MARCHA 6
#define INDICADOR_DA_SEXTA_MARCHA 7

#define LED_CPU 8

#define CAN_SCK 13
#define CAN_SO 12
#define CAN_SI 11
#define CAN_CS 10

#define TMR_BASE 100000
#define TMR_CANSEND 1000000
#define TMR_BLINK 100000
#define TMR_PRESSAO_AR 1000000
#define TMR_TEMP_AR 1000000
#define TMR_INDICADOR_MARCHA 100000

/**************************************************************************************************************************************/
/* IMPORTAÇÃO DAS BIBLIOTECAS                                                                                                         */
/**************************************************************************************************************************************/
#include <mcp2515.h>
#include <SPI.h>
#include <EK304CAN.h>
#include <math.h>
#include <TimerOne.h>

/**************************************************************************************************************************************/
/* DECLARAÇÃO DAS TAREFAS, SEMÁFOROS E FILAS                                                                                          */
/**************************************************************************************************************************************/
//TAREFAS

void taskCanCommunication(void);
void taskTemperaturaDoAr(void);
void taskPressaoDoAr(void);
void taskIndicadorDaMarcha(void);
void taskScheduler(void);
void taskBlink(void);

/**************************************************************************************************************************************/
/* PROTÓTIPO DAS FUNÇÕES                                                                                                              */
/**************************************************************************************************************************************/

int gearSelect();
float turboPressure();

/**************************************************************************************************************************************/
/* DECLARAÇÃO DE TIPOS NÃO PRIMITIVOS                                                                                                 */
/**************************************************************************************************************************************/

CAN_Frame frame;
CAN_Frame frameRe;

/**************************************************************************************************************************************/
/* DECLARACAO DE VARIAVEIS GLOBAIS                                                                                                   */
/**************************************************************************************************************************************/
bool tmrCansendOverflow = false;
bool tmrCansendEnable = false;
int tmrCansendCount = 0;

bool tmrTemperaturaDoArOverflow = false;
bool tmrTemperaturaDoArEnable = false;
int tmrTemperaturaDoArCount = 0;

bool tmrPressaoDoArOverflow = false;
bool tmrPressaoDoArEnable = false;
int tmrPressaoDoArCount = 0;

bool tmrIndicadorDaMarchaOverflow = false;
bool tmrIndicadorDaMarchaEnable = false;
int tmrIndicadorDaMarchaCount = 0;

bool tmrBlinkOverflow = false;
bool tmrBlinkEnable = false;
int tmrBlinkCount = 0;

/**************************************************************************************************************************************/
/* CONFIGURAÇÕES DOS PERIFÉRICOS                                                                                                      */
/**************************************************************************************************************************************/
MCP2515 mcp2515(CAN_CS); //Pino 10 é o Slave

void setup()
{
  frame.id.endDestino = EK304CAN_ID_ADDRESS_GTW;
  frame.id.endOrigem = EK304CAN_ID_ADDRESS_THIS;
  frame.id.tipo = EK304CAN_ID_TYPE_SENSORDATA;

  frame.msg.variant = 0x00;
  frame.msg.length = 5;

  Serial.begin(9600);
  SPI.begin();

  pinMode(LED_CPU, OUTPUT);

  digitalWrite(LED_CPU, HIGH);
  CAN_Init(&mcp2515, CAN_1000KBPS);
  digitalWrite(LED_CPU, LOW);

  delay(1000);

  pinMode(PRESSAO_DO_AR, INPUT);
  pinMode(TEMPERATURA_DO_AR, INPUT);

  pinMode(INDICADOR_DA_PRIMEIRA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_SEGUNDA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_TERCEIRA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_QUARTA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_QUINTA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_SEXTA_MARCHA, INPUT_PULLUP);

  tmrCansendEnable = true;

  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);
}

void loop()
{
  taskCanCommunication();
  //taskTemperaturaDoAr();
  //taskPressaoDoAr();
  taskIndicadorDaMarcha();
  taskBlink();
}

/**************************************************************************************************************************************/
/* DEFINIÇÃO DAS TAREFAS                                                                                                              */
/**************************************************************************************************************************************/
//Comunicação com a CAN
void taskCanCommunication(void)
{
  if (CAN_ReceiveData(&mcp2515, &frameRe) == MCP2515::ERROR_OK)
  {
    if (frameRe.id.tipo == EK304CAN_ID_TYPE_ACK) //envia pacote do tipo ACK para o Gateway
    {
      if (frameRe.id.endDestino == EK304CAN_ID_ADDRESS_THIS)
      {
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_GTW);
        digitalWrite(LED_CPU, !digitalRead(LED_CPU));
      }
    }
  }

  if (tmrCansendOverflow)
  {
    CAN_SendData(&mcp2515, &frame);
    tmrCansendOverflow = false;
  }
}
/*
//Faz leitura do sensor da tempertarura do ar
void taskTemperaturaDoAr(void)ds
{
  if (tmrTemperaturaDoArOverflow)
  {

    float temperatura;
    unsigned int temp;
    unsigned char temperaturaEnviada[2];

    float tensao = analogRead(TEMPERATURA_DO_AR) * 5.0 / 1023.0;
    temperatura = ((1.0 / -0.0404) * log((tensao * 1000.0) / (7021.0 * (5.0 - tensao))));
    temp = (unsigned int)(temperatura);

    temperaturaEnviada[0] = temp >> 8;
    temperaturaEnviada[1] = temp & 0xFF;
    frame.msg.data[0] = temperaturaEnviada[0];
    frame.msg.data[1] = temperaturaEnviada[1];

    tmrTemperaturaDoArOverflow = false;
  }
}
*/
/*
//Faz a leitura do sensor da pressao do ar
void taskPressaoDoAr(void)
{
  if (tmrPressaoDoArOverflow)
  {
    float pressao;
    unsigned int pres;
    unsigned char pressaoEnviada[2];

    pressao = turboPressure();

    pres = (unsigned int)(pressao);

    pressaoEnviada[0] = pres >> 8;
    pressaoEnviada[1] = pres & 0xFF;

    frame.msg.data[2] = pressaoEnviada[0];
    frame.msg.data[3] = pressaoEnviada[1];

    tmrPressaoDoArOverflow = false;
  }
}
*/

//Faz a leitura da posição da marcha
void taskIndicadorDaMarcha(void)
{
  if (tmrIndicadorDaMarchaOverflow)
  {
    unsigned int gear;
    unsigned char gearSent;

    gear = gearSelect();

    gearSent = gear & 0xFF;

    frame.msg.data[4] = gearSent;

    tmrIndicadorDaMarchaOverflow = false;
  }
}

/**************************************************************************************************************************************/
/* FUNÇÕES DE TRATAMENTO DE INTERRUPÇÃO                                                                                               */
/**************************************************************************************************************************************/

/**************************************************************************************************************************************/
/* FUNÇÕES GERAIS                                                                                                                     */
/************************************************************************************************* *************************************/

int gearSelect()
{
  int count;
  int gear;

  for (count = INDICADOR_DA_PRIMEIRA_MARCHA; count <= INDICADOR_DA_SEXTA_MARCHA; count++)
  {
    //A ligação do hardware é importante para dar certo
    if (!digitalRead(count))
    {
      gear = count - (INDICADOR_DA_PRIMEIRA_MARCHA - 3);
    }
  }

  return gear;
}

/*
float turboPressure()
{
  float pressao;
  float a = 53.743;
  float b = -1.0935;

  pressao = ((5.0 * analogRead(PRESSAO_DO_AR)) / 1023.0);
  pressao = pressao * a + b;

  if (pressao < 0)
  {
    pressao = 0;
  }

  return pressao;
}
*/

void taskScheduler(void)
{
  if (tmrCansendEnable)
  {
    tmrCansendCount++;
    if (tmrCansendCount >= TMR_CANSEND / TMR_BASE)
    {
      tmrCansendCount = 0;
      tmrCansendOverflow = true;
    }
  }

  if (tmrBlinkEnable)
  {
    tmrBlinkCount++;
    if (tmrBlinkCount >= TMR_BLINK / TMR_BASE)
    {
      tmrBlinkCount = 0;
      tmrBlinkOverflow = true;
    }
  }

  /*

  if (tmrPressaoDoArEnable)
  {
    tmrPressaoDoArCount++;
    if (tmrPressaoDoArCount >= TMR_PRESSAO_AR / TMR_BASE)
    {
      tmrPressaoDoArCount = 0;
      tmrPressaoDoArOverflow = true;
    }
  }

  if (tmrTemperaturaDoArEnable)
  {
    tmrTemperaturaDoArCount++;
    if (tmrTemperaturaDoArCount >= TMR_TEMP_AR / TMR_BASE)
    {
      tmrTemperaturaDoArCount = 0;
      tmrTemperaturaDoArOverflow = true;
    }
  }

*/

  if (tmrIndicadorDaMarchaEnable)
  {
    tmrIndicadorDaMarchaCount++;
    if (tmrIndicadorDaMarchaCount >= TMR_INDICADOR_MARCHA / TMR_BASE)
    {
      tmrIndicadorDaMarchaCount = 0;
      tmrIndicadorDaMarchaOverflow = true;
    }
  }
}

void taskBlink(void)
{
  digitalWrite(LED_CPU, tmrBlinkEnable);
  if (tmrBlinkOverflow)
  {
    tmrBlinkOverflow = false;
    tmrBlinkEnable = false;
  }
}