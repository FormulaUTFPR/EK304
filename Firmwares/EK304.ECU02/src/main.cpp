////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome do projeto: EK304.ECU02                                                                                                       //
// Nome do arquivo: main.cpp                                                                                                          //
// Desenvolvido por: Leonardo Alberton Men |   Gabriel Ferreira Nagamatsu                                                             //
// Data/versão: 21/11/2019 (v0.0.4)                                                                                                   //
// IDE utilizada: Visual Studio Code & PlatformIO                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define LED_CPU 8

#include <SPI.h>
#include <mcp2515.h>
#include <EK304CAN.h>
#include <TimerOne.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Definição das portas
#define SensorMAP A0
#define SensorSON A1
#define SensorTPS A2
#define SensorTEMPAG A3
#define SensorTEMPAR A4

#define CAN_SCK 13
#define CAN_SO 12
#define CAN_SI 11
#define CAN_CS 10

#define PIN_RPM 3 //Porta para o sensor de rotação

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Delaração das Variaveis Globais
float ReadValue[6] = {0, 0, 0, 0, 0, 0};
float LambdaFactor;
int SensorValue[6];
int PressureValue;
int TPSValue;
int WaterTemp;
int AirTemp;

//Declaração de variaveis
unsigned long InitialTime; //Tempo em Microsegundos em que ocorreu o pulso
unsigned long TimeDif;     //Valor da diferenca de tempo entre dois pulsos
unsigned int average;      //Media entre alguns RPMs para ter um valor com menos interferencias
unsigned long frequency;   //frequencia não suavizada
unsigned long RPM;         //RPM nao suavizado - o suavizado é a media
unsigned long sum;         //Soma dos periodos para fazer a media
int counter = 0;           //Contador para fazer a media

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Criação das tasks
void taskSensoresAnalogicos(void); //Cria task dos sensores TPS/MAP/TEMP AGUA/TEMP AR
void taskRotacao(void);            //Cria task do sensor de velocidade
void taskCANSend(void);            //Cria task para enviar dados
void taskScheduler(void);
void taskBlink(void);
void taskRPM(void);

//Protótipos de funções

void setupCAN();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Declaracao das flags

bool tmrCANSendEnable = false;
bool tmrCANSendOverflow = false;
int tmrCANSendCount = 0;

bool tmrCANSendAirTempEnable = false;
bool tmrCANSendAirTempOverflow = false;
int tmrCANSendAirTempCount = 0;

bool tmrCANSendLambdaEnable = false;
bool tmrCANSendLambdaOverflow = false;
int tmrCANSendLambdaCount = 0;

bool tmrCANSendMAPEnable = false;
bool tmrCANSendMAPOverflow = false;
int tmrCANSendMAPCount = 0;

bool tmrCANSendRPMEnable = false;
bool tmrCANSendRPMOverflow = false;
int tmrCANSendRPMCount = 0;

bool tmrCANSendTPSEnable = false;
bool tmrCANSendTPSOverflow = false;
int tmrCANSendTPSCount = 0;

bool tmrCANSendWaterTempEnable = false;
bool tmrCANSendWaterTempOverflow = false;
int tmrCANSendWaterTempCount = 0;

bool tmrBlinkEnable = false;
bool tmrBlinkOverflow = false;
int tmrBlinkCount = 0;

bool tmrSensoresAnalogicosEnable = false;
bool tmrSensoresAnalogicosOverflow = false;
int tmrSensoresAnalogicosCount = 0;

bool tmrRotacaoStoppedEnable = false;
bool tmrRotacaoStoppedOverflow = false;
int tmrRotacaoStoppedCount = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CAN
can_frame can_AirTemp;
can_frame can_lambda;
can_frame can_MAP;
can_frame can_RPM;
can_frame can_TPS;
can_frame can_WaterTemp;

MCP2515 mcp2515(CAN_CS);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Timers

#define TMR_BASE 100000
#define TMR_CANSEND 1000000
#define TMR_BLINK 100000
#define TMR_ANALOGICO 100000
#define TMR_AIR_TEMP 1000000
#define TMR_LAMBDA 50000
#define TMR_MAP 50000
#define TMR_RPM 20000
#define TMR_TPS 50000
#define TMR_WATER_TEMP 1000000
#define TMR_CHECK_RPM_STOP 1500000

//Outras variaveis globais

#define NUM_AMOSTRAGEM 5 //Numero de amostragens pra media do RPM
#define NUM_SPARKS 2     //Numero de centelhas por revolução

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(9600);
    //CAN

    setupCAN();

    pinMode(LED_CPU, OUTPUT);
    pinMode(PIN_RPM, INPUT_PULLUP);
    pinMode(A1, INPUT);

    // Sensor de RPM
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), taskRPM, RISING); //Quando o sensor passa de LOW pra HIGH, chama a funcao taskPulso

    Timer1.initialize(TMR_BASE);
    Timer1.attachInterrupt(taskScheduler);

    tmrBlinkEnable = true;
    tmrCANSendEnable = true;
    tmrSensoresAnalogicosEnable = false;
    tmrRotacaoStoppedEnable = true;

    tmrCANSendAirTempEnable = false;
    tmrCANSendLambdaEnable = false;
    tmrCANSendMAPEnable = false;
    tmrCANSendRPMEnable = true;
    tmrCANSendTPSEnable = false;
    tmrCANSendWaterTempEnable = false;
}

void loop()
{
    taskBlink();
    taskCANSend();
    taskRotacao();
    taskSensoresAnalogicos();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void taskRPM(void)
{
    if (counter >= NUM_AMOSTRAGEM)
    {
        TimeDif = micros() - InitialTime; //Calcula a diferenca de tempo entre dois pulsos
        frequency = 1000000 / TimeDif;    //Faz o perídodo virar frequencia
        RPM = frequency * 60;             //Multiplica por 60 a frequencia para ter rotacoes por MINUTO
        average = RPM * counter;          //Multiplica o RPM por (idealmente) NUM_AMOSTRAGEM para ter a media entre os NUM_AMOSTRAGEM periodos
        average = average / NUM_SPARKS;   //Divide a media pelo numero de centelhas numa revolução

        //average = average / CONST_DIV_RPM;

        can_RPM.data[1] = average & 0xFF << 8;
        can_RPM.data[0] = average & 0xFF;

        counter = 0;            //faz o counter voltar para 1
        InitialTime = micros(); //Armazena o valor atual para calcular a diferença a próxima vez que for chamado
    }
    else
    {
        counter++; //Incrementa o counter
    }

    tmrRotacaoStoppedCount = 0; //Deixa o counter dessa task em 0 pra evitar enviar 0 como RPM

} //Acaba a tarefa taskRPM

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Tasks

void taskSensoresAnalogicos(void)
{

    if (tmrSensoresAnalogicosEnable)
    {
        //Atribuição valores sensor
        SensorValue[0] = analogRead(SensorMAP);
        SensorValue[1] = analogRead(SensorSON);
        SensorValue[2] = analogRead(SensorTPS);
        SensorValue[3] = analogRead(SensorTEMPAG);
        SensorValue[4] = analogRead(SensorTEMPAR);

        for (int i = 0; i < 5; i++) //faz a conversão de bits para valores de tensão em Volts
        {
            ReadValue[i] = SensorValue[i] * (5.0 / 1023);
        }

        PressureValue = (ReadValue[0] - 0.204) * 0.018819;                                     //Converte para KPA
        LambdaFactor = (((ReadValue[1] - 0.2) * 0.65 / 4.6) + 0.65) * 100;                     //valor X100 para torna-lo inteiro //Converte a para fator Lambda
        TPSValue = (ReadValue[2] - 0.23) * (100 / 3.75);                                       //Converte para Porcentagem
        if (ReadValue[2] < 0)                                                                  //
        {                                                                                      //
            TPSValue = 0;                                                                      //
        }                                                                                      //
        else                                                                                   //
        {                                                                                      // Impede do valor em porcentagem ser negativo e maior que 100
            if (ReadValue[2] > 100)                                                            //
            {                                                                                  //
                TPSValue = 100;                                                                //
            }                                                                                  //
        }                                                                                      //
        WaterTemp = (-1 / 0.038) * log((ReadValue[3] * 1000) / (7656.8 * (5 - ReadValue[3]))); //Converte para °C
        AirTemp = (-1 / 0.04) * log((ReadValue[4] * 1000) / (7021 * (5 - ReadValue[4])));      //Converte para °C
        //Atribuição dos valores convertidos no pacote da CAN
        can_MAP.data[0] = PressureValue;
        can_lambda.data[0] = LambdaFactor;
        can_TPS.data[0] = TPSValue;
        can_WaterTemp.data[0] = WaterTemp;
        can_AirTemp.data[0] = AirTemp;

        tmrSensoresAnalogicosOverflow = false;
    }
}

void taskRotacao(void)
{
    if (tmrRotacaoStoppedEnable)
    {
        can_RPM.data[0] = 0 & 0xFF;
        can_RPM.data[1] = 0 & 0xFF;

        tmrRotacaoStoppedOverflow = false;
    }
}

void taskCANSend(void)
{
    /*
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
*/

    if (tmrCANSendAirTempOverflow)
    {
        int AirTempReadValue = analogRead(SensorTEMPAR);
        can_AirTemp.data[0] = (-1 / 0.04) * log((AirTempReadValue * 1000) / (7021 * (5 - AirTempReadValue))); //Converte para KPA

        mcp2515.sendMessage(&can_AirTemp);
        tmrCANSendAirTempOverflow = false;

        tmrBlinkEnable = true;
    }

    if (tmrCANSendLambdaOverflow)
    {
        int LambdaReadValue = analogRead(SensorSON);
        can_lambda.data[0] = (((LambdaReadValue - 0.2) * 0.65 / 4.6) + 0.65) * 100; //valor X100 para torna-lo inteiro //Converte a para fator Lambda

        mcp2515.sendMessage(&can_lambda);
        tmrCANSendLambdaOverflow = false;

        tmrBlinkEnable = true;
    }

    if (tmrCANSendMAPOverflow)
    {
        int MAPReadValue = analogRead(SensorMAP);
        int MAPvalue = MAPReadValue * 5 / 1023;

        can_MAP.data[0] = (MAPvalue - 0.204) * 0.018819;

        mcp2515.sendMessage(&can_MAP);
        tmrCANSendMAPOverflow = false;

        tmrBlinkEnable = true;
    }

    if (tmrCANSendRPMOverflow)
    {
        average = analogRead(A1) * 12;

        can_RPM.data[1] = average & 0xFF << 8;
        can_RPM.data[0] = average & 0xFF;

        mcp2515.sendMessage(&can_RPM);
        tmrCANSendRPMOverflow = false;

        tmrBlinkEnable = true;
    }

    if (tmrCANSendTPSOverflow)
    {
        int TPSReadValue = analogRead(SensorTPS);
        can_TPS.data[0] = (TPSReadValue - 0.23) * (100 / 3.75); //Converte para Porcentagem
        if (TPSReadValue < 0)                                   //
        {                                                       //
            can_TPS.data[0] = 0;                                //
        }                                                       //
        else                                                    //
        {                                                       // Impede do valor em porcentagem ser negativo e maior que 100
            if (TPSReadValue > 100)                             //
            {                                                   //
                can_TPS.data[0] = 100;                          //
            }                                                   //
        }

        mcp2515.sendMessage(&can_TPS);
        tmrCANSendTPSOverflow = false;

        tmrBlinkEnable = true;
    }

    if (tmrCANSendWaterTempOverflow)
    {
        int WaterTempReadValue = analogRead(SensorTEMPAG);
        can_WaterTemp.data[0] = (-1 / 0.038) * log((WaterTempReadValue * 1000) / (7656.8 * (5 - WaterTempReadValue))); //Converte para °C

        Serial.println("watertemp");

        mcp2515.sendMessage(&can_WaterTemp);
        tmrCANSendWaterTempOverflow = false;

        tmrBlinkEnable = true;
    }
}

void taskScheduler(void)
{
    if (tmrCANSendEnable)
    {
        tmrCANSendCount++;
        if (tmrCANSendCount >= TMR_CANSEND / TMR_BASE)
        {
            tmrCANSendCount = 0;
            tmrCANSendOverflow = true;
        }
    }

    if (tmrSensoresAnalogicosEnable)
    {
        tmrSensoresAnalogicosCount++;
        if (tmrSensoresAnalogicosCount >= TMR_ANALOGICO / TMR_BASE)
        {
            tmrSensoresAnalogicosCount = 0;
            tmrSensoresAnalogicosOverflow = true;
        }
    }

    if (tmrCANSendAirTempEnable)
    {
        tmrCANSendAirTempCount++;
        if (tmrCANSendAirTempCount >= TMR_AIR_TEMP / TMR_BASE)
        {
            tmrCANSendAirTempCount = 0;
            tmrCANSendAirTempOverflow = true;
        }
    }

    if (tmrCANSendLambdaEnable)
    {
        tmrCANSendLambdaCount++;
        if (tmrCANSendLambdaCount >= TMR_LAMBDA / TMR_BASE)
        {
            tmrCANSendLambdaCount = 0;
            tmrCANSendLambdaOverflow = true;
        }
    }

    if (tmrCANSendMAPEnable)
    {
        tmrCANSendMAPCount++;
        if (tmrCANSendMAPCount >= TMR_MAP / TMR_BASE)
        {
            tmrCANSendMAPCount = 0;
            tmrCANSendMAPOverflow = true;
        }
    }

    if (tmrCANSendRPMEnable)
    {
        tmrCANSendRPMCount++;
        if (tmrCANSendRPMCount >= TMR_RPM / TMR_BASE)
        {
            tmrCANSendRPMCount = 0;
            tmrCANSendRPMOverflow = true;
        }
    }

    if (tmrCANSendTPSEnable)
    {
        tmrCANSendTPSCount++;
        if (tmrCANSendTPSCount >= TMR_TPS / TMR_BASE)
        {
            tmrCANSendTPSCount = 0;
            tmrCANSendTPSOverflow = true;
        }
    }

    if (tmrCANSendWaterTempEnable)
    {
        tmrCANSendWaterTempCount++;
        if (tmrCANSendWaterTempCount >= TMR_WATER_TEMP / TMR_BASE)
        {
            tmrCANSendWaterTempCount = 0;
            tmrCANSendWaterTempOverflow = true;
        }
    }

    if (tmrRotacaoStoppedEnable)
    {
        tmrRotacaoStoppedCount++;
        if (tmrRotacaoStoppedCount >= TMR_CHECK_RPM_STOP / TMR_BASE)
        {
            tmrRotacaoStoppedCount = 0;
            tmrRotacaoStoppedOverflow = true;
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Funções

void setupCAN()
{
    digitalWrite(LED_CPU, HIGH);
    CAN_Init(&mcp2515, CAN_100KBPS);
    digitalWrite(LED_CPU, LOW);

    can_AirTemp.can_id = EK304CAN_ID_AIR_TEMP;
    can_AirTemp.can_dlc = 1;

    can_lambda.can_id = EK304CAN_ID_LAMBA;
    can_lambda.can_dlc = 1;

    can_MAP.can_id = EK304CAN_ID_MAP;
    can_MAP.can_dlc = 2;

    can_RPM.can_id = EK304CAN_ID_RPM;
    can_RPM.can_dlc = 2;

    can_TPS.can_id = EK304CAN_ID_TPS;
    can_TPS.can_dlc = 1;

    can_WaterTemp.can_id = EK304CAN_ID_WATER_TEMPERATURE;
    can_WaterTemp.can_dlc = 1;
}