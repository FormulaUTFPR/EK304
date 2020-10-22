#include <EK304CAN.h>
#include <mcp2515.h>
#include <TimerOne.h>
#include <SPI.h>
#include <Wire.h>

#include <EK304ERRORS.h>
#include <EK304SETTINGS.h>
//Declaração de tarefas

unsigned int taskSpeed();
void taskPressure(void);
void taskCAN(void);
void taskAcc(void);
void taskSusp(void);
void taskTemp(void);
void taskScheduler(void);
void taskBlink(void);

//Protótipos de funções

void pulseHappened(void);
void setupInit();              //PinModes, inicializar outras coisas, entre outros
void taskVehicleStopped(void); //Cria task do sensor de velocidade
void setupCAN();               //Setup da CAN, pacotes e parâmetros
void setupACC();               //Setup dos acelerômetros

//Protótipos de funções de tratamento

void ISR_SPEED(); //Tratamento da interrupção do pino do leitor de velocidade

//Definição das portas

#define PIN_SPEED 3 //Porta para o sensor de rotação
#define LED_CPU 8   //Porta para o LED de resposta da ACK

//Portas analogicas

#define PIN_OIL_TEMP A1   //Porta para o sensor de temperatura do oleo
#define PIN_PRESSURE A0   //Porta para o sensor de pressão
#define PIN_RIGHT_SUSP A2 //Porta para o sensor da suspensao direita
#define PIN_LEFT_SUSP A3  //Porta para o sensor da suspensao esquerda

//Endereço do acelerometro
const int MPU1 = 0x68; //Endereço é importante para pinagem

// FLAGS

//Enable é para ativar/desativar a função
//Overflow é pra chamar a função
//Count é o contador para o overflow

bool tmrCANSendSpeedEnable = false;
bool tmrCANSendSpeedOverflow = false;
int tmrCANSendSpeedCount = 0;

bool tmrTempEnable = false;
bool tmrTempOverflow = false;
int tmrTempCount = 0;

bool tmrPressureEnable = false;
bool tmrPressureOverflow = false;
int tmrPressureCount = 0;

bool tmrSuspEnable = false;
bool tmrSuspOverflow = false;
int tmrSuspCount = 0;

bool tmrAccEnable = false;
bool tmrAccOverflow = false;
int tmrAccCount = 0;

bool tmrBlinkOverflow = false;
bool tmrBlinkEnable = false;
int tmrBlinkCount = 0;

bool flagSpeed = false; //Cria uma flag para dizer que o evento pulso do sensor de velocidade ocorreu
int numPulses = 0;      //Cria uma variável para armazenar a quantidade de pulsos que ocorreram antes da função ser tratada

//Pinos da CAN - pinagem do modulo mcp2515

#define CAN_SCK 13
#define CAN_SI 11
#define CAN_SO 12
#define CAN_CS 10

//Criar pacotes da CAN

can_frame canACEL; //Frame pro acelerometro
can_frame canGYRO; //Frame pro giroscopio
can_frame canOilPressure;
can_frame canOilTemp;
can_frame canSpeed;
can_frame canSuspRear;

//Variaveis globais

#define SAMPLE_QTTY 5     //Numero de amostragens pra media do calculo da velocidade
#define MAGNET_QTTY 9     //Numero de imãs na roda fônica
#define MAX_SPEED 10000   //Numero max de velocidade
#define MIN_READ_SUSP 117 //Minimo valor de leitura na porta analogica
#define MAX_READ_SUSP 914 //Maximo valor de leitura na porta analogica
#define MIN_READ_OIL 870  //Minimo valor de leitura na porta analogica
#define MAX_READ_OIL 216  //Maximo valor de leitura na porta analogica
#define MIN_READ_PRES 102 //Leitura minima de 0,5v --Ajustar
#define MAX_READ_PRES 921 //Leitura máxima de 4,5v --Ajustar
#define MIN_PRES 0        //Valor min em bar
#define MAX_PRES 10       //Valor máx em bar
#define NUM_CONST 1       //Valor para dividir o sensor de velocidade para obter a velocidade

#define TMR_BASE 100000     //Temporizador base para o Timer1
#define TMR_CANSEND 300000  //Chamar a função da CAN a cada 0,5 segundos
#define TMR_TEMP 1000000    //Leitura da temperatura a cada 0,1 segundo
#define TMR_PRESSURE 100000 //Leitura da pressão a cada 0,1 segundo
#define TMR_SUSP 100000     //Leitura dos dados da suspensão 0,1 segundo
#define TMR_ACC 1000000     //Leitura dos dados do acelerômetro a cada 0,1 segundo
#define TMR_BLINK 100000    //Chamar a função para piscar o LED do módulo

//Declaração de variaveis globais
unsigned long InitialTime; //Tempo em Microsegundos em que ocorreu o pulso
unsigned long TimeDif;     //Valor da diferenca de tempo entre dois pulsos

MCP2515 mcp2515(CAN_CS);

void setup()
{
  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  //Serial.begin(9600);
  //Serial.println("setup");

  setupInit(); //PinModes, inicializar outras coisas, entre outros
  setupCAN();  //Setup da CAN, pacotes e parâmetros
  //setupACC();  //Setup dos acelerômetros

  Timer1.initialize(TMR_BASE);           //Inicializar a biblioteca Timer1 com o tempo de TMR_BASE
  Timer1.attachInterrupt(taskScheduler); //Define o escalonador

  //Variaveis para ativação das tarefas

  tmrCANSendSpeedEnable = true;
  tmrTempEnable = false;
  tmrSuspEnable = false;
  tmrPressureEnable = false;
  tmrAccEnable = false;

  //Serial.println("Acabou o setup");

} //Aqui acaba o setup

void loop()
{
  taskCAN();
  taskAcc();
  taskPressure();
  taskSusp();
  taskTemp();
  taskBlink();
}

/*--------------------------------------------------*/
/*--------------------- TASKS ----------------------*/
/*--------------------------------------------------*/

unsigned int taskSpeed()
{
  unsigned long TimeDif; //Valor da diferenca de tempo entre dois pulsos
  unsigned int average;  //Media entre alguns RPMs para ter um valor com menos interferencias
  unsigned long RPM;     //RPM nao suavizado - o suavizado é a media
  unsigned int counter;  //Contador para fazer a media

  if (flagSpeed)
  {
    flagSpeed = false;   //Desativa a flag que chama a função
    counter = numPulses; //Coloca o valor de numPulses numa variável que não pertence ao interrupt
    numPulses = 0;       //Faz o numero de pulsos que ocorreu voltar para 0

    //Calcula os pulsos para virar rotação por min

    TimeDif = micros() - InitialTime; //Calcula a diferenca de tempo entre dois pulsos
    RPM = 60000000 / TimeDif;         //Faz o perídodo virar frequencia e multiplica por 60 a frequencia para ter rotacoes por MINUTO
    average = RPM * counter;          //Multiplica o RPM por (idealmente) NUM_AMOSTRAGEM para ter a media entre os NUM_AMOSTRAGEM periodos
    average = average / NUM_CONST;    //Divide a media pelo numero de centelhas numa revolução

    InitialTime = micros(); //Armazena o valor atual para calcular a diferença a próxima vez que for chamado
  }
  else
  {
    average = 0; //Caso a flag não esteja true é porque não teve pulso, ou seja, rotação
  }

  return average;
} //Acaba a getRPM

void taskTemp(void)
{
  if (tmrTempOverflow) //Checa a flag do overflow dessa tarefa
  {
    //Sensor MTE4053 (temperatura)

    unsigned int sender = int((-1 / 0.038) * log((analogRead(PIN_OIL_TEMP) * 1000) / (7656.8 * (5 - analogRead(PIN_OIL_TEMP)))));

    canOilTemp.data[0] = sender & 0xFF;

    mcp2515.sendMessage(&canOilTemp);

    //Checar se precisa alterar para enviar via CAN

    tmrTempOverflow = false; //Coloca a flag em falso novamente

    tmrBlinkEnable = true;
  }
}

void taskPressure(void)
{
  if (tmrPressureOverflow)
  {
    //Sensor de pressao

    float voltage;

    //pressao

    voltage = map(analogRead(PIN_PRESSURE), MIN_READ_PRES, MAX_READ_PRES, MIN_PRES, MAX_PRES); //Faz regra de  com o valor da leitura
    canOilPressure.data[0] = (3.0 * (voltage - 0.47));                                         //Faz os cálculos para converter a tensao lida em pressao

    mcp2515.sendMessage(&canOilPressure);

    tmrPressureOverflow = false;

    tmrBlinkEnable = true;
  }
}

void taskSusp(void)
{
  if (tmrSuspOverflow)
  {
    //Suspensao

    unsigned int sender1 = analogRead(PIN_LEFT_SUSP);

    unsigned int sender2 = analogRead(PIN_RIGHT_SUSP);

    canSuspRear.data[0] = (sender1>>8) & 0xFF;
    canSuspRear.data[1] = sender1 & 0x03;

    canSuspRear.data[2] = (sender2>>8) & 0xFF;
    canSuspRear.data[3] = sender2 & 0x03;

    mcp2515.sendMessage(&canSuspRear);

    tmrSuspOverflow = false;

    tmrBlinkEnable = true;
  }
}

void taskCAN(void)
{
  if (tmrCANSendSpeedOverflow) //Tirar essa parte depois da mudança dos ids da CAN
  {
    //Envios

    canSpeed.data[0] = taskSpeed();

    mcp2515.sendMessage(&canSpeed);

    tmrCANSendSpeedOverflow = false;

    tmrBlinkEnable = true;
  }
}

void taskAcc(void) //Tarefa do acelerometro
{
  if (tmrAccOverflow)
  {

    float AcX, AcY, AcZ, GyX, GyY, GyZ;
    unsigned int fAcx1, fAcy1, fAcz1;
    float Acx1, Acy1, Acz1, Gyx1, Gyy1, Gyz1;

    Wire.beginTransmission(MPU1);     //Transmissao
    Wire.write(0x3B);                 //Endereco 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);      //Finaliza transmissao
    Wire.requestFrom(MPU1, 14, true); //Solicita os dados do sensor

    //Armazenamento dos valores do acelerometro e giroscopio
    AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    //Wire.read() << 8 | Wire.read();       //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) <- Joga fora esses dados
    GyX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Acx1 = (AcX / 16384) * 100; //  Dividido por 16384 para converter os valores para 1G,
    Acy1 = (AcY / 16384) * 100; //  multiplicado por 100 para obter com precisao de duas
    Acz1 = (AcZ / 16384) * 100; //  casas decimais.
    Gyx1 = GyX / 131;           //  Dividido por 131 para converter os valores para graus/s
    Gyy1 = GyY / 131;
    Gyz1 = GyZ / 131;

    fAcx1 = (unsigned int)map((Acx1 + 105), 0, 220, 0, 200); // Nova escala de 0 a 200.             // Aproximadamente 100 se refere a 0 G.
    fAcy1 = (unsigned int)map((Acy1 + 105), 0, 220, 0, 200); // Essa escala se refere a -1 a 1 G.   // Aproximadamente 200 se refere a 1 G.
    fAcz1 = (unsigned int)map((Acz1 + 105), 0, 220, 0, 200); // Aproximadamente 0 se refere a -1 G.

    unsigned int fGyx1 = map((Gyx1 + 250), 0, 500, 0, 250); // Nova escala de 0 a 250.                     // Aproximadamente 125 se refere a 0 graus/s.
    unsigned int fGyy1 = map((Gyy1 + 250), 0, 500, 0, 250); // Essa escala se refere a -250 a 250 graus/s. // Aproximadamente 250 se refere a 250 graus/s.
    unsigned int fGyz1 = map((Gyz1 + 250), 0, 500, 0, 250); // Aproximadamente 0 se refere a -250 graus/s.

    canACEL.data[0] = (fAcx1>>8) & 0xFF;
    canACEL.data[1] = fAcx1 & 0x0F;
    canACEL.data[2] = (fAcy1>>8) & 0xFF;
    canACEL.data[3] = fAcy1 & 0x0F;
    canACEL.data[4] = (fAcz1>>8) & 0xFF;
    canACEL.data[5] = fAcz1 & 0x0F;


    canGYRO.data[0] = (fGyx1>>8) & 0xFF;
    canGYRO.data[1] = fGyx1 & 0x0F;
    canGYRO.data[2] = (fGyy1>>8) & 0xFF;
    canGYRO.data[3] = fGyy1 & 0x0F;
    canGYRO.data[4] = (fGyz1>>8) & 0xFF;
    canGYRO.data[5] = fGyz1 & 0x0F;

    mcp2515.sendMessage(&canACEL);

    mcp2515.sendMessage(&canGYRO);

    tmrAccOverflow = false;

    tmrBlinkEnable = true;
  }
}

//Funções do Setup

void setupInit()
{
  SPI.begin();
  Wire.begin(); //Inicia I2C
  //Serial.begin(9600);

  //Serial.println("Setup");

  //Modos das entradas

  pinMode(PIN_SPEED, INPUT_PULLUP); //Toda vez que tem uma subida chama a tarefa
  pinMode(PIN_PRESSURE, INPUT);
  pinMode(PIN_OIL_TEMP, INPUT);
  pinMode(PIN_RIGHT_SUSP, INPUT);
  pinMode(PIN_LEFT_SUSP, INPUT);
  pinMode(LED_CPU, OUTPUT);                                              //LED do módulo
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), ISR_SPEED, FALLING); //Quando o sensor passa de HIGH pra LOW, chama a funcao taskPulso
}

void setupCAN()
{

  digitalWrite(LED_CPU, HIGH); //Deixa o LED ligado enquanto está settando a CAN
  CAN_Init(&mcp2515, CAN_100KBPS);
  digitalWrite(LED_CPU, LOW); //Desliga o LED

  canACEL.can_id = EK304CAN_ID_ACC_03; //Define o id como o do acelerômetro 3 da CAN
  canACEL.can_dlc = 6;                 //Tamanho do pacote

  canOilPressure.can_id = EK304CAN_ID_OIL_PRESSURE;
  canOilPressure.can_dlc = 1;

  canOilTemp.can_id = EK304CAN_ID_OIL_TEMPERATURE;
  canOilTemp.can_dlc = 1;

  canSpeed.can_id = EK304CAN_ID_SPEED;
  canSpeed.can_dlc = 1;

  canSuspRear.can_id = EK304CAN_ID_SUSP_REAR;
  canSuspRear.can_dlc = 4;
}

void setupACC()
{
  //Programação dos acelerômetros

  Wire.beginTransmission(MPU1);
  Wire.write(0x6B);

  //Inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);
}

//Scheduler

void taskScheduler(void) //Aqui comeca o escalonador
{
  if (tmrCANSendSpeedEnable)
  {
    tmrCANSendSpeedCount++;                             //Incrementa o valor dessa variável todo ciclo
    if (tmrCANSendSpeedCount >= TMR_CANSEND / TMR_BASE) //Checa se o valor estorou
    {
      tmrCANSendSpeedCount = 0;       //Volta o valor para 0
      tmrCANSendSpeedOverflow = true; //Ativa o overflow para a tarefa ser ativada
    }
  }

  if (tmrTempEnable)
  {
    tmrTempCount++;
    if (tmrTempCount >= TMR_TEMP / TMR_BASE)
    {
      tmrTempCount = 0;
      tmrTempOverflow = true;
    }
  }

  if (tmrSuspEnable)
  {
    tmrSuspCount++;
    if (tmrSuspCount >= TMR_SUSP / TMR_BASE)
    {
      tmrSuspCount = 0;
      tmrSuspOverflow = true;
    }
  }

  if (tmrPressureEnable)
  {
    tmrPressureCount++;
    if (tmrPressureCount >= TMR_PRESSURE / TMR_BASE)
    {
      tmrPressureCount = 0;
      tmrPressureOverflow = true;
    }
  }

  if (tmrAccEnable)
  {
    tmrAccCount++;
    if (tmrAccCount >= TMR_ACC / TMR_BASE)
    {
      tmrAccCount = 0;
      tmrAccOverflow = true;
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

//Outras funções

// TRATAMENTO DE INTERRUPÇÕES

void ISR_SPEED(void) //ISR para quando há pulso
{
  flagSpeed = true;
  numPulses++; //É feio fazer isso em ISR heheh, ela pode interromper a função que está usando essa variável no momento e aí falha catastrófica
}