////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome do projeto: EK304.TCANR                                                                                                       //
// Desenvolvido por: Rafael Ramalho | @RamalhoFael                                                                                    //
// Data/versão: 26/10/2019 (v0.0.0)                                                                                                   //
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
CAN_Frame frame;


/**************************************************************************************************************************************/
/* CONFIGURAÇÕES DOS PERIFÉRICOS                                                                                                      */
/**************************************************************************************************************************************/
MCP2515 mcp2515(CAN_CS);

void setup()
{
  Serial.begin(115200);
  
  Serial.println("led acende");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  SPI.begin();
  Serial.println("SPI");
  CAN_Init(&mcp2515, CAN_1000KBPS);
  digitalWrite(LED, LOW);
  Serial.println("led apaga");
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

/**************************************************************************************************************************************/
/* LOOP PRINCIPAL                                                                                                                     */
/**************************************************************************************************************************************/
void loop() 
{
  if (CAN_ReceiveData(&mcp2515, &frame) == MCP2515::ERROR_OK)
  {
    Serial.print(frame.id.endOrigem, HEX);
    Serial.print(frame.id.endDestino, HEX);
    Serial.print(frame.id.tipo, HEX);

    Serial.print(" ");
    Serial.print(frame.msg.length);
    Serial.print(" ");
    Serial.print(frame.msg.variant, HEX);
    Serial.print(" ");

    for (int i = 0; i < frame.msg.length; i++)
    {
      Serial.print(frame.msg.data[i], HEX);
      Serial.print(" ");
    }

    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    
    Serial.println();
  }
}
