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
float ValorLido[6] = {0, 0, 0, 0, 0, 0};
float FatorLambda;
int ValorSensor[6];
int ValorPre;
int ValorTPS;
int TempAG;
int TempAR;

//Declaração de variaveis
unsigned long tempoInicial; //Tempo em Microsegundos em que ocorreu o pulso
unsigned long difTempo;     //Valor da diferenca de tempo entre dois pulsos
unsigned int media;         //Media entre alguns RPMs para ter um valor com menos interferencias
unsigned long frequencia;   //Frequencia não suavizada
unsigned long RPM;          //RPM nao suavizado - o suavizado é a media
unsigned long soma;         //Soma dos periodos para fazer a media
int contador = 0;           //contador para fazer a media

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Criação das tasks
void taskSensoresAnalogicos(void); //Cria task dos sensores TPS/MAP/TEMP AGUA/TEMP AR
void taskRotacao(void);            //Cria task do sensor de velocidade
void taskCANSend(void);            //Cria task para enviar dados
void taskScheduler(void);
void taskBlink(void);
void taskRPM(void);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Declaracao das flags

bool tmrCANSendEnable = false;
bool tmrCANSendOverflow = false;
int tmrCANSendCount = 0;

bool tmrBlinkEnable = false;
bool tmrBlinkOverflow = false;
int tmrBlinkCount = 0;

bool tmrSensoresAnalogicosEnable = false;
bool tmrSensoresAnalogicosOverflow = false;
int tmrSensoresAnalogicosCount = 0;

bool tmrRotacaoEnable = false;
bool tmrRotacaoOverflow = false;
int tmrRotacaoCount = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CAN
CAN_Frame frame;
CAN_Frame frameRe;
MCP2515 mcp2515(CAN_CS);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Timers

#define TMR_BASE 100000
#define TMR_CANSEND 1000000
#define TMR_BLINK 100000
#define TMR_ANALOGICO 100000

//Outras variaveis globais

#define NUM_AMOSTRAGEM 5 //Numero de amostragens pra media do RPM
#define NUM_IMAS 9       //Numero de imãs na roda fônica
#define MAX_RPM 13000    //Numero max de RPM

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(9600);
    //CAN
    frame.id.endOrigem = EK304CAN_ID_ADDRESS_THIS;
    frame.id.endDestino = EK304CAN_ID_ADDRESS_GTW;
    frame.id.tipo = EK304CAN_ID_TYPE_SENSORDATA;

    frame.msg.variant = 0x00;
    frame.msg.length = 6;

    frame.msg.data[0] = ValorPre;
    frame.msg.data[1] = FatorLambda;
    frame.msg.data[2] = ValorTPS;
    frame.msg.data[3] = TempAG;
    frame.msg.data[4] = TempAR;
    frame.msg.data[5] = RPM;

    pinMode(LED_CPU, OUTPUT);
    pinMode(PIN_RPM, INPUT_PULLUP);

    digitalWrite(LED_CPU, HIGH);
    Serial.print("Conectando ao MCP2515... ");
    CAN_Init(&mcp2515, CAN_1000KBPS);
    Serial.println("Conectado!");
    digitalWrite(LED_CPU, LOW);

    // Sensor de RPM
    attachInterrupt(digitalPinToInterrupt(PIN_RPM), taskRPM, RISING); //Quando o sensor passa de LOW pra HIGH, chama a funcao taskPulso

    Timer1.initialize(TMR_BASE);
    Timer1.attachInterrupt(taskScheduler);

    tmrBlinkEnable = true;
    tmrCANSendEnable = true;
    tmrSensoresAnalogicosEnable = true;
    tmrRotacaoEnable = true;
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
    if (contador >= NUM_AMOSTRAGEM)
    {
        difTempo = micros() - tempoInicial; //Calcula a diferenca de tempo entre dois pulsos
        frequencia = 1000000 / difTempo;    //Faz o perídodo virar frequencia
        RPM = frequencia * 60;              //Multiplica por 60 a frequencia para ter rotacoes por MINUTO
        media = RPM * contador;             //Multiplica o RPM por (idealmente) NUM_AMOSTRAGEM para ter a media entre os NUM_AMOSTRAGEM periodos
        media = media / NUM_IMAS;           //Divide a media pelo numero de imãs na roda fonica

        media = map(media, 0, MAX_RPM, 0, 255);

        frame.msg.data[5] = media;

        contador = 0;            //faz o contador voltar para 1
        tempoInicial = micros(); //Armazena o valor atual para calcular a diferença a próxima vez que for chamado
    }
    else
    {
        contador++; //Incrementa o contador
    }
} //Acaba a tarefa taskRPM

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Tasks
void taskSensoresAnalogicos(void)
{

    if (tmrSensoresAnalogicosEnable)
    {
        //Atribuição valores sensor
        ValorSensor[0] = analogRead(SensorMAP);
        ValorSensor[1] = analogRead(SensorSON);
        ValorSensor[2] = analogRead(SensorTPS);
        ValorSensor[3] = analogRead(SensorTEMPAG);
        ValorSensor[4] = analogRead(SensorTEMPAR);

        for (int i = 0; i < 5; i++) //faz a conversão de bits para valores de tensão em Volts
        {
            ValorLido[i] = ValorSensor[i] * (5.0 / 1023);
        }

        ValorPre = (ValorLido[0] - 0.204) * 0.018819;                                       //Converte para KPA
        FatorLambda = (((ValorLido[1] - 0.2) * 0.65 / 4.6) + 0.65) * 100;                   //valor X100 para torna-lo inteiro //Converte a para fator Lambda
        ValorTPS = (ValorLido[2] - 0.23) * (100 / 3.75);                                    //Converte para Porcentagem
        if (ValorLido[2] < 0)                                                               //
        {                                                                                   //
            ValorTPS = 0;                                                                   //
        }                                                                                   //
        else                                                                                //
        {                                                                                   // Impede do valor em porcentagem ser negativo e maior que 100
            if (ValorLido[2] > 100)                                                         //
            {                                                                               //
                                                                                            //
                ValorTPS = 100;                                                             //
            }                                                                               //
        }                                                                                   //
        TempAG = (-1 / 0.038) * log((ValorLido[3] * 1000) / (7656.8 * (5 - ValorLido[3]))); //Converte para °C
        TempAR = (-1 / 0.04) * log((ValorLido[4] * 1000) / (7021 * (5 - ValorLido[4])));    //Converte para °C
        //Atribuição dos valores convertidos no pacote da CAN
        frame.msg.data[0] = ValorPre;
        frame.msg.data[1] = FatorLambda;
        frame.msg.data[2] = ValorTPS;
        frame.msg.data[3] = TempAG;
        frame.msg.data[4] = TempAR;

        tmrSensoresAnalogicosOverflow = false;
    }
}

void taskRotacao(void)
{
    if (tmrRotacaoEnable)
    {
        tmrRotacaoOverflow = false;
    }
}

void taskCANSend(void)
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
    if (tmrCANSendOverflow)
    {
        CAN_SendData(&mcp2515, &frame);
        tmrCANSendOverflow = false;
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