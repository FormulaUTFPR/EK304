////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome da biblioteca: EK304CAN_ERRORS                                                                                                //
// Nome do arquivo: EK304CAN_ERRORS.h                                                                                                 //
// Desenvolvido por: Rafael Ramalho | @RamalhoFael                                                                                    //
// Data/versão: 13/11/2019 (v0.1.3)                                                                                                   //
// IDE utilizada: Visual Studio Code & PlatformIO                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef EK304CAN_ERRORS_H
#define EK304CAN_ERRORS_H

#define ERROR_AMOUNT 32

#define ERROR_CAN_INIT_CODE 0x0101
#define ERROR_CAN_ECU01_CODE 0x0101
#define ERROR_CAN_ECU02_CODE 0x0102
#define ERROR_CAN_ECU03_CODE 0x0103
#define ERROR_CAN_ECU04_CODE 0x0104
#define ERROR_CAN_ECU05_CODE 0x0105
#define ERROR_CAN_ECU06_CODE 0x0106
#define ERROR_CAN_ECU07_CODE 0x0107
#define ERROR_CAN_ECU08_CODE 0x0108
#define ERROR_CAN_ECU09_CODE 0x0109
#define ERROR_CAN_ECU10_CODE 0x010A
#define ERROR_CAN_ECU11_CODE 0x010B
#define ERROR_CAN_ECU12_CODE 0x010C
#define ERROR_CAN_ECU13_CODE 0x010D
#define ERROR_CAN_ECU14_CODE 0x010E
#define ERROR_CAN_ECU15_CODE 0x010F

#define ERROR_GPS_NOTCONNECTED_CODE 0x0201
#define ERROR_GPS_NOSIGNAL_CODE 0x0202
#define ERROR_GPS_LOWSIGNAL_CODE 0x0203

#define ERROR_ELECTRICAL_OVERVOLTAGE_CODE 0x0301
#define ERROR_ELECTRICAL_UNDERVOLTAGE_CODE 0x0302

#define ERROR_CAN_INIT_ID 0x0000
#define ERROR_CAN_ECU01_ID 0x001
#define ERROR_CAN_ECU02_ID 0x002
#define ERROR_CAN_ECU03_ID 0x003
#define ERROR_CAN_ECU04_ID 0x004
#define ERROR_CAN_ECU05_ID 0x005
#define ERROR_CAN_ECU06_ID 0x006
#define ERROR_CAN_ECU07_ID 0x007
#define ERROR_CAN_ECU08_ID 0x008
#define ERROR_CAN_ECU09_ID 0x009
#define ERROR_CAN_ECU10_ID 0x00A
#define ERROR_CAN_ECU11_ID 0x00B
#define ERROR_CAN_ECU12_ID 0x00C
#define ERROR_CAN_ECU13_ID 0x00D
#define ERROR_CAN_ECU14_ID 0x00E
#define ERROR_CAN_ECU15_ID 0x00F

#define ERROR_GPS_NOTCONNECTED_ID 0x10
#define ERROR_GPS_NOSIGNAL_ID 0x11
#define ERROR_GPS_LOWSIGNAL_ID 0x12
#define ERROR_ELECTRICAL_OVERVOLTAGE_ID 0x13
#define ERROR_ELECTRICAL_UNDERVOLTAGE_ID 0x14
#define ERROR_SD_CARD_INIT 0x15
#define ERROR_SD_CARD_FULL 0x16
#define ERROR_ID_PACKET 0x17

#endif