////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome do projeto: EK304.TCANR                                                                                                       //
// Desenvolvido por: Rafael Ramalho | @RamalhoFael                                                                                    //
// Data/versão: 28/10/2019 (v0.0.1)                                                                                                   //
// IDE utilizada: Arduino v1.8.10                                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************************************************/
/* DEFINIÇÃO DE PINOS                                                                                                                 */
/**************************************************************************************************************************************/
#define CAN_INT 19      // define CAN_INT (interrupção de dado) no pino 19
#define CAN_SCK 52      // define CAN_SCK (clock do SPI bus) no pino 52
#define CAN_SI 51       // define CAN_SI  (slave input do SPI bus) no pino 51
#define CAN_SO 50       // define CAN_SO  (slave output do SPI bus) no pino 50
#define CAN_CS 10       // define CAN_CS  (slave select do SPI bus) no pino 53

/**************************************************************************************************************************************/
/* IMPORTAÇÃO DAS BIBLIOTECAS                                                                                                         */
/**************************************************************************************************************************************/
#include <SPI.h>
#include <mcp2515.h>
#include "EK304CAN.h"

/**************************************************************************************************************************************/
/* DEFINIÇÃO DE PINOS                                                                                                                 */
/**************************************************************************************************************************************/
#define LED 8

/**************************************************************************************************************************************/
/* DECLARAÇÃO DE VARIÁVEIS E CONSTANTES GLOBAIS                                                                                       */
/**************************************************************************************************************************************/
CAN_Frame frame, f;
unsigned long t;

/**************************************************************************************************************************************/
/* CONFIGURAÇÕES DOS PERIFÉRICOS                                                                                                      */
/**************************************************************************************************************************************/
MCP2515 mcp2515(CAN_CS);

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  SPI.begin();
  CAN_Init(&mcp2515, CAN_1000KBPS);
  digitalWrite(LED, LOW);

    frame.id.endOrigem = 0x2;
  frame.id.endDestino = 0xE;
  frame.id.tipo = 0x6;
  
  frame.msg.variant = 0x8E;
  frame.msg.length = 7;
  
  frame.msg.data[0] = 0x86;
  frame.msg.data[1] = 0x87;
  frame.msg.data[2] = 0x32;
  frame.msg.data[3] = 0xFA;
  frame.msg.data[4] = 0x26;
  frame.msg.data[5] = 0x8E;
  frame.msg.data[6] = 0xBE;
}

/**************************************************************************************************************************************/
/* LOOP PRINCIPAL                                                                                                                     */
/**************************************************************************************************************************************/
void loop() 
{
  if (CAN_ReceiveData(&mcp2515, &f) == MCP2515::ERROR_OK)
  {
    if(f.id.endDestino == EK304CAN_ID_ADDRESS_ECU15 && f.id.tipo == EK304CAN_ID_TYPE_ACK)
      CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_ECU15, f.id.endDestino);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    
    Serial.println();
  }

  if ((millis() - t)>1000)
  {
  frame.msg.data[6] = millis()/1000;
  CAN_SendData(&mcp2515, &frame);
  t = millis();
  }
}
