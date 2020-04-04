////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome da biblioteca: EK304CAN                                                                                                       //
// Nome do arquivo: EK304CAN.h                                                                                                        //
// Desenvolvido por: Rafael Ramalho | @RamalhoFael                                                                                    //
// Data/versão: 13/11/2019 (v0.1.3)                                                                                                   //
// IDE utilizada: Visual Studio Code & PlatformIO                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef EK304CAN_H_
#define EK304CAN_H_
#define EK304CAN_ECUS_AMOUNT 15

#define EK304CAN_ID_ADDRESS_GTW 0x00
#define EK304CAN_ID_ADDRESS_ECU01 0x01
#define EK304CAN_ID_ADDRESS_ECU02 0x02
#define EK304CAN_ID_ADDRESS_ECU03 0x03
#define EK304CAN_ID_ADDRESS_ECU04 0x04
#define EK304CAN_ID_ADDRESS_ECU05 0x05
#define EK304CAN_ID_ADDRESS_ECU06 0x06
#define EK304CAN_ID_ADDRESS_ECU07 0x07
#define EK304CAN_ID_ADDRESS_ECU08 0x08
#define EK304CAN_ID_ADDRESS_ECU09 0x09
#define EK304CAN_ID_ADDRESS_ECU10 0x0A
#define EK304CAN_ID_ADDRESS_ECU11 0x0B
#define EK304CAN_ID_ADDRESS_ECU12 0x0C
#define EK304CAN_ID_ADDRESS_ECU13 0x0D
#define EK304CAN_ID_ADDRESS_ECU14 0x0E
#define EK304CAN_ID_ADDRESS_ECU15 0x0F

#define EK304CAN_ID_TYPE_SENSORDATA 0x01
#define EK304CAN_ID_TYPE_ACK 0x02
#define EK304CAN_ID_TYPE_ERROR 0x03

#include <mcp2515.h>
#include <EK304ERRORS.h>
#include <EK304SETTINGS.h>

// estrutura do id da can
class CAN_ID
{
public:
    __u32 endOrigem;
    __u32 endDestino;
    __u32 tipo;

    CAN_ID()
    {
        endOrigem = 0x0;
        endDestino = 0x0;
        tipo = 0x0;
    }

    CAN_ID(__u32 o, __u32 d, __u32 t)
    {
        endOrigem = o;
        endDestino = d;
        tipo = t;
    }
};

// estrutura das mensagens da CAN
class CAN_Message
{
public:
    __u8 length;
    __u8 data[7];
    __u8 variant;

    CAN_Message()
    {
        length = 0;
        variant = 0x00;

        for (int i = 0; i < 7; i++)
        {
            data[i] = 0x00;
        }
    }
};

// estrutura dos dados da CAN
class CAN_Frame
{
public:
    CAN_ID id;
    CAN_Message msg;

    CAN_Frame()
    {
        id = CAN_ID();
        msg = CAN_Message();
    }

    CAN_Frame(CAN_ID i, CAN_Message m)
    {
        id = i;
        msg = m;
    }
};

CAN_Frame CAN_ChangeProtocol(can_frame low);                        // converte a mensagem para os protocolos da CAN: baixo nível -> alto nível
can_frame CAN_ChangeProtocol(CAN_Frame high);                       // converte a mensagem para os protocolos da CAN: alto nível -> baixo nível
MCP2515::ERROR CAN_ReceiveData(MCP2515 *mcp2515, CAN_Frame *frame); // armazena em um CAN_Frame os dados recebidos da CAN
MCP2515::ERROR CAN_SendData(MCP2515 *mcp2515, CAN_Frame *frame);    // envia os dados de um CAN_Frame na CAN
MCP2515::ERROR CAN_SendACK(MCP2515 *mcp2515, __u32 endDestino);     // envia um ACK na CAN
void CAN_Init(MCP2515 *mcp2515, const CAN_SPEED speed);             // configura a CAN na velocidade desejada

#endif