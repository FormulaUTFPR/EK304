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
void setupWire();              //Setup dos acelerômetros

//Protótipos de funções de tratamento

void ISR_SPEED(); //Tratamento da interrupção do pino do leitor de velocidade

//Definição das portas

#define PIN_SPEED 3 //Porta para o sensor de rotação
#define LED_CPU 8   //Porta para o LED de resposta da ACK

//Portas analogicas

#define PIN_PRESSURE A0   //Porta para o sensor de pressão
#define PIN_OIL_TEMP A1   //Porta para o sensor de temperatura do oleo
#define PIN_RIGHT_SUSP A1 //Porta para o sensor da suspensao direita
#define PIN_LEFT_SUSP A0  //Porta para o sensor da suspensao esquerda //A4 e A5 pro acelerômetro

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

bool flagSpeed = false;     //Cria uma flag para dizer que o evento pulso do sensor de velocidade ocorreu
volatile int numPulses = 0; //Cria uma variável para armazenar a quantidade de pulsos que ocorreram antes da função ser tratada

//  VARIÁVEIS ANTIGAS ACELERÔMETRO

/*
  float Ac1X, Ac1Y, Ac1Z, Gy1X, Gy1Y, Gy1Z;
  float Acx1, Acy1, Acz1, Gyx1, Gyy1, Gyz1;
  unsigned int fAcx1, fAcy1, fAcz1;
*/

//  VARIÁVEIS ATUALIZADAS DO ACELERÔMETRO

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;       //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error = 0;                          //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;       //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;         //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;             //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;

//Pinos da CAN - pinagem do modulo mcp2515

#define CAN_SCK 13
#define CAN_SI 11
#define CAN_SO 12
#define CAN_CS 10

//Criar pacotes da CAN

can_frame Modulo2Acc;  //Frame pro acelerometro
can_frame Modulo2Gyro; //Frame pro giroscopio
can_frame canOilPressure;
can_frame canOilTemp;
can_frame canSpeed;
can_frame canSuspRear;

//Variaveis globais

#define SAMPLE_QTTY 5     //Numero de amostragens pra media do calculo da velocidade
#define MAGNET_QTTY 1     //Numero de imãs na roda fônica
#define MAX_SPEED 10000   //Numero max de velocidade
#define MIN_READ_SUSP 117 //Minimo valor de leitura na porta analogica
#define MAX_READ_SUSP 914 //Maximo valor de leitura na porta analogica
#define MIN_READ_OIL 870  //Minimo valor de leitura na porta analogica
#define MAX_READ_OIL 216  //Maximo valor de leitura na porta analogica
#define MIN_READ_PRES 102 //Leitura minima de 0,5v --Ajustar
#define MAX_READ_PRES 921 //Leitura máxima de 4,5v --Ajustar
#define MIN_PRES 0        //Valor min em bar
#define MAX_PRES 10       //Valor máx em bar
#define NUM_CONST 8       //Valor para dividir o sensor de velocidade para obter a velocidade
#define WHEEL_SIZE 0.255  //Tamanho da roda
bool estadoLed = false;   //variavel que define o estado do led

#define TMR_BASE 100000     //Temporizador base para o Timer1
#define TMR_CANSEND 300000  //Chamar a função da CAN a cada 0,5 segundos
#define TMR_TEMP 1000000    //Leitura da temperatura a cada 0,1 segundo
#define TMR_PRESSURE 100000 //Leitura da pressão a cada 0,1 segundo
#define TMR_SUSP 100000     //Leitura dos dados da suspensão 0,1 segundo
#define TMR_ACC 100000      //Leitura dos dados do acelerômetro a cada 0,1 segundo
#define TMR_BLINK 100000    //Timer para piscar o led

//Declaração de variaveis globais
unsigned long InitialTime; //Tempo em Microsegundos em que ocorreu o pulso
unsigned long TimeDif;     //Valor da diferenca de tempo entre dois pulsos

unsigned int average;          //Media entre alguns RPMs para ter um valor com menos interferencias
unsigned long RPM;             //RPM nao suavizado - o suavizado é a media
volatile unsigned int counter; //Contador para fazer a media

MCP2515 mcp2515(CAN_CS);

void setup()
{
  //Serial.begin(9600);
  //Serial.println("setup");

  setupInit(); //PinModes, inicializar outras coisas, entre outros
  setupCAN();  //Setup da CAN, pacotes e parâmetros
  setupWire(); //Setup dos acelerômetros

  SPI.begin();

  Timer1.initialize(TMR_BASE);           //Inicializar a biblioteca Timer1 com o tempo de TMR_BASE
  Timer1.attachInterrupt(taskScheduler); //Define o escalonador

  //Variaveis para ativação das tarefas

  tmrCANSendSpeedEnable = true;
  tmrTempEnable = false;
  tmrSuspEnable = true;
  tmrPressureEnable = false;
  tmrAccEnable = true;

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
  if (flagSpeed)
  {
    flagSpeed = false;   //Desativa a flag que chama a função
    counter = numPulses; //Coloca o valor de numPulses numa variável que não pertence ao interrupt
    numPulses = 0;       //Faz o numero de pulsos que ocorreu voltar para 0

    //Calcula os pulsos para virar rotação por min

    TimeDif = micros() - InitialTime; //Calcula a diferenca de tempo entre dois pulsos
    RPM = 60 * 1000000 / TimeDif;     //Faz o perídodo virar frequencia e multiplica por 60 a frequencia para ter rotacoes por HORA, multiplica por 1000000 pra transformar microsegundos em segundos, divide por 100000 pra fazer cm pra km
    average = RPM * counter;          //Multiplica o RPM por (idealmente) NUM_AMOSTRAGEM para ter a media entre os NUM_AMOSTRAGEM periodos
    average = average / NUM_CONST;    //Divide a media pelo numero de centelhas numa revolução

    average = average * 2 * 3.14 * WHEEL_SIZE;

    average = average / 216;

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

    if (mcp2515.sendMessage(&canOilTemp) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }

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

    if (mcp2515.sendMessage(&canOilPressure) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }

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

    canSuspRear.data[0] = (sender1 >> 8) & 0xFF;
    canSuspRear.data[1] = sender1 & 0xFF;

    canSuspRear.data[2] = (sender2 >> 8) & 0xFF;
    canSuspRear.data[3] = sender2 & 0xFF;

    if (mcp2515.sendMessage(&canSuspRear) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }

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

    if (mcp2515.sendMessage(&canSpeed) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }

    tmrCANSendSpeedOverflow = false;

    tmrBlinkEnable = true;
  }
}

void taskAcc(void) //Tarefa do acelerometro
{
  if (tmrAccOverflow)
  { /*
    Wire.beginTransmission(MPU2);     //Transmissao
    Wire.write(0x3B);                 //Endereco 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);      //Finaliza transmissao
    Wire.requestFrom(MPU2, 14, true); //Solicita os dados do sensor

   //Wire.beginTransmission(MPU2);
   //Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
   //Wire.endTransmission(false);
   //Wire.requestFrom(0x68, 14, true);

   //Armazenamento dos valores do acelerometro e giroscopio
   Ac2X = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   Ac2Y = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   Ac2Z = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   //Wire.read() << 8 | Wire.read();        //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) <- Joga fora esses dados
   Gy2X = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   Gy2Y = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   Gy2Z = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

   Acx2 = (Ac2X / 16384) * 100; //  Dividido por 16384 para converter os valores para 1G,
   Acy2 = (Ac2Y / 16384) * 100; //  multiplicado por 100 para obter com precisao de duas
   Acz2 = (Ac2Z / 16384) * 100; //  casas decimais,
   Gyx2 = Gy2X / 131;           //  Dividido por 131 para converter os valores para graus/s.
   Gyy2 = Gy2Y / 131;
   Gyz2 = Gy2Z / 131;

   int iAcx2 = int(Acx2);    // Nova escala de 0 a 200
   int iAcy2 = int(Acy2);    // Essa escala se refere a -1 a 1 G
   int iAcz2 = int(Acz2);    // Aproximadamente 0 se refere a -1 G.
   int iiAcx2 = iAcx2 + 120; // Aproximadamente 100 se refere a 0 G.
   int iiAcy2 = iAcy2 + 120; // Aproximadamente 200 se refere a 1 G.
   int iiAcz2 = iAcz2 + 120;
   unsigned int fAcx2 = map(iiAcx2, 0, 215, 0, 200);
   unsigned int fAcy2 = map(iiAcy2, 0, 215, 0, 200);
   unsigned int fAcz2 = map(iiAcz2, 0, 205, 0, 200);

   int iGyx2 = int(Gyx2);    // Nova escala de 0 a 250.
   int iGyy2 = int(Gyy2);    // Essa escala se refere a -250 a 250 graus/s.
   int iGyz2 = int(Gyz2);    // Aproximadamente 0 se refere a -250 graus/s.
   int iiGyx2 = iGyx2 + 250; // Aproximadamente 125 se refere a 0 graus/s.
   int iiGyy2 = iGyy2 + 250; // Aproximadamente 250 se refere a 250 graus/s.
   int iiGyz2 = iGyz2 + 250;
   unsigned int fGyx2 = map(iiGyx2, 0, 500, 0, 250);
   unsigned int fGyy2 = map(iiGyy2, 0, 500, 0, 250);
   unsigned int fGyz2 = map(iiGyz2, 0, 500, 0, 250);

   Serial.println(Gyx2);
   Serial.println(Gyy2);
   Serial.println(Gyz2);
*/
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    Acc_rawX = (Wire.read() << 8 | Wire.read()) / 4096.0; //each value needs two registres
    Acc_rawY = (Wire.read() << 8 | Wire.read()) / 4096.0;
    Acc_rawZ = (Wire.read() << 8 | Wire.read()) / 4096.0;

    Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
    Wire.write(0x43);             //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true); //We ask for just 4 registers

    Gyr_rawX = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
    Gyr_rawY = Wire.read() << 8 | Wire.read();
    Gyr_rawZ = Wire.read() << 8 | Wire.read();

    int iAcx2 = int(Acc_rawX); // Nova escala de 0 a 200
    int iAcy2 = int(Acc_rawY); // Essa escala se refere a -1 a 1 G
    int iAcz2 = int(Acc_rawZ); // Aproximadamente 0 se refere a -1 G.

    int iGyx2 = int(Gyr_rawX); // Nova escala de 0 a 250.
    int iGyy2 = int(Gyr_rawY); // Essa escala se refere a -250 a 250 graus/s.
    int iGyz2 = int(Gyr_rawZ); // Aproximadamente 0 se refere a -250 graus/s.

    Modulo2Acc.data[0] = (iAcx2 >> 8) & 0xFF;
    Modulo2Acc.data[1] = iAcx2 & 0x0F;
    Modulo2Acc.data[2] = (iAcy2 >> 8) & 0xFF;
    Modulo2Acc.data[3] = iAcy2 & 0x0F;
    Modulo2Acc.data[4] = (iAcz2 >> 8) & 0xFF;
    Modulo2Acc.data[5] = iAcz2 & 0x0F;

    Modulo2Gyro.data[0] = (iGyx2 >> 8) & 0xFF;
    Modulo2Gyro.data[1] = iGyx2 & 0x0F;
    Modulo2Gyro.data[2] = (iGyy2 >> 8) & 0xFF;
    Modulo2Gyro.data[3] = iGyy2 & 0x0F;
    Modulo2Gyro.data[4] = (iGyz2 >> 8) & 0xFF;
    Modulo2Gyro.data[5] = iGyz2 & 0x0F;

    tmrAccOverflow = false;

    mcp2515.sendMessage(&Modulo2Acc);  // envia os dados de um CAN_Frame na CAN
    mcp2515.sendMessage(&Modulo2Gyro); // envia os dados de um CAN_Frame na CAN
  }
}

//Funções do Setup

void setupInit()
{
  pinMode(LED_CPU, OUTPUT);

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  analogReference(INTERNAL);

  //Modos das entradas

  pinMode(PIN_SPEED, INPUT_PULLUP); //Toda vez que tem uma subida chama a tarefa
  pinMode(PIN_PRESSURE, INPUT);
  pinMode(PIN_OIL_TEMP, INPUT);
  pinMode(PIN_RIGHT_SUSP, INPUT);
  pinMode(PIN_LEFT_SUSP, INPUT);                                        //LED do módulo
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), ISR_SPEED, RISING); //Quando o sensor passa de HIGH pra LOW, chama a funcao taskPulso
}

void setupCAN()
{

  digitalWrite(LED_CPU, HIGH); //Deixa o LED ligado enquanto está settando a CAN
  CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW); //Desliga o LED

  //MODULO 3
  Modulo2Acc.can_id = EK304CAN_ID_ACC_03;
  Modulo2Acc.can_dlc = 6;

  Modulo2Gyro.can_id = EK304CAN_ID_GYRO_03;
  Modulo2Gyro.can_dlc = 6;

  canOilPressure.can_id = EK304CAN_ID_OIL_PRESSURE;
  canOilPressure.can_dlc = 1;

  canOilTemp.can_id = EK304CAN_ID_OIL_TEMPERATURE;
  canOilTemp.can_dlc = 1;

  canSpeed.can_id = EK304CAN_ID_SPEED;
  canSpeed.can_dlc = 1;

  canSuspRear.can_id = EK304CAN_ID_SUSP_REAR;
  canSuspRear.can_dlc = 4;
}

void setupWire()
{
  Wire.begin();                 //begin the wire comunication
  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true); //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);             //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);   //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68); //Start communication with the address found during search.
  Wire.write(0x1C);             //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);             //Set the register bits as 00010000 (+/- 8g full scale range)
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
  if (tmrBlinkOverflow)
  {
    digitalWrite(LED_CPU, estadoLed);
    estadoLed != estadoLed;
    tmrBlinkOverflow = false;
  }
}

//Outras funções

// TRATAMENTO DE INTERRUPÇÕES

void ISR_SPEED(void) //ISR para quando há pulso
{
  flagSpeed = true;
  numPulses++; //É feio fazer isso em ISR heheh, ela pode interromper a função que está usando essa variável no momento e aí falha catastrófica
}