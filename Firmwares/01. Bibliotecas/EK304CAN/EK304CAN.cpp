////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome da biblioteca: EK304CAN                                                                                                       //
// Nome do arquivo: EK304CAN.cpp                                                                                                      //
// Desenvolvido por: Rafael Ramalho | @RamalhoFael                                                                                    //
// Data/versão: 28/10/2019 (v0.1.2)                                                                                                   //
// IDE utilizada: Visual Studio Code & PlatformIO                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "EK304CAN.h"

CAN_Frame CAN_ChangeProtocol(can_frame low)
{
    CAN_Frame high;
    __u32 o, d, t;

    o = (low.can_id & 0b11110000000) >> 7;
    d = (low.can_id & 0b00001111000) >> 3;
    t = (low.can_id & 0b00000000111);

    high.id.endOrigem = o;
    high.id.endDestino = d;
    high.id.tipo = t;

    high.msg.length = low.can_dlc - 1;
    high.msg.variant = low.data[0];

    for (int i = 0; i < low.can_dlc - 1; i++)
        high.msg.data[i] = (low.data[i + 1]);

    return high;
}

can_frame CAN_ChangeProtocol(CAN_Frame high)
{
    can_frame low;

    low.can_id = (high.id.endOrigem << 7) + (high.id.endDestino << 3) + (high.id.tipo);
    low.can_dlc = high.msg.length + 1;
    low.data[0] = high.msg.variant;
    for (int i = 0; i < high.msg.length; i++)
        low.data[i + 1] = high.msg.data[i];

    return low;
}

MCP2515::ERROR CAN_ReceiveData(MCP2515 *mcp2515, CAN_Frame *frame)
{
    can_frame canMsg = CAN_ChangeProtocol(*frame);
    MCP2515::ERROR sts = mcp2515->readMessage(&canMsg);

    if (sts == MCP2515::ERROR_OK)
        *frame = CAN_ChangeProtocol(canMsg);

    return sts;
}

MCP2515::ERROR CAN_SendData(MCP2515 *mcp2515, CAN_Frame *frame)
{
    can_frame canMsg = CAN_ChangeProtocol(*frame);
    return (mcp2515->sendMessage(&canMsg));
}

MCP2515::ERROR CAN_SendACK(MCP2515 *mcp2515, __u32 endDestino)
{
    CAN_ID id(EK304CAN_ID_ADDRESS_THIS, endDestino, EK304CAN_ID_TYPE_ACK);
    CAN_Frame frameACK(id, CAN_Message());
    return CAN_SendData(mcp2515, &frameACK);
}

void CAN_Init(MCP2515 *mcp2515, const CAN_SPEED speed)
{
    do
    {
        mcp2515->reset();
    } while (mcp2515->setBitrate(speed) != MCP2515::ERROR_OK);
    mcp2515->setNormalMode();
}
