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
#define TMR_INDICADOR_MARCHA 200000
#define TMR_ACELE2 500000

/**************************************************************************************************************************************/
/* IMPORTAÇÃO DAS BIBLIOTECAS                                                                                                         */
/**************************************************************************************************************************************/
#include <mcp2515.h>
#include <SPI.h>
#include <EK304CAN.h>
#include <math.h>
#include <TimerOne.h>
#include <Wire.h>

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
void taskModu2(void);

/**************************************************************************************************************************************/
/* PROTÓTIPO DAS FUNÇÕES                                                                                                              */
/**************************************************************************************************************************************/

void setupCAN();
int gearSelect();
float turboPressure();
void setupWIRE();

/**************************************************************************************************************************************/
/* DECLARAÇÃO DE TIPOS NÃO PRIMITIVOS                                                                                                 */
/**************************************************************************************************************************************/

CAN_Frame frame;
CAN_Frame frameRe;

can_frame Modulo2Acc;
can_frame Modulo2Gyro;

can_frame can_gear;

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

bool tmrAcele2Overflow = false;
bool tmrAcele2Enable = false;
int tmrAcele2Count = 0;

bool estadoLed = false;

/**************************************************************************************************************************************/
/* DECLARACAO DE TIMERS                                                                                                   */
/**************************************************************************************************************************************/
#define TMR_BLINK 100000 //Timer para piscar o led

/**************************************************************************************************************************************/
/* CONFIGURAÇÕES DOS PERIFÉRICOS                                                                                                      */
/**************************************************************************************************************************************/
MCP2515 mcp2515(CAN_CS); //Pino 10 é o Slave

//endereços dos módulos
const int MPU2 = 0x68; // Se o pino ADO for conectado em 5V ou 3,3V o modulo assume esse endereço

//Declaração de variáveis do acelerômetro

float Ac2X, Ac2Y, Ac2Z, Tmp, Gy2X, Gy2Y, Gy2Z;
float Acx2, Acy2, Acz2, Gyx2, Gyy2, Gyz2;
int fAcx2, fAcy2, fAcz2;
int fGyx2, fGyy2, fGyz2;

void setup()
{

  pinMode(LED_CPU, OUTPUT);

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  Serial.begin(9600);

  SPI.begin();

  setupCAN();

  pinMode(PRESSAO_DO_AR, INPUT);
  pinMode(TEMPERATURA_DO_AR, INPUT);

  pinMode(INDICADOR_DA_PRIMEIRA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_SEGUNDA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_TERCEIRA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_QUARTA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_QUINTA_MARCHA, INPUT_PULLUP);
  pinMode(INDICADOR_DA_SEXTA_MARCHA, INPUT_PULLUP);

  tmrIndicadorDaMarchaEnable = true;
  tmrPressaoDoArEnable = false;     //NÃO ATIVAR -- SENSORES DESCONECTADOS
  tmrTemperaturaDoArEnable = false; //NÃO ATIVAR -- SENSORES DESCONECTADOS
  tmrAcele2Enable = true;
  tmrBlinkEnable = true;

  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  can_gear.can_id = EK304CAN_ID_GEAR_POSITION;
  can_gear.can_dlc = 1;

  //MODULO 2
  Modulo2Acc.can_id = EK304CAN_ID_ACC_02;
  Modulo2Acc.can_dlc = 6;

  Modulo2Gyro.can_id = EK304CAN_ID_GYRO_02;
  Modulo2Gyro.can_dlc = 6;

  //Serial.println("Acabou o Setup");
}

void loop()
{
  taskCanCommunication();
  taskTemperaturaDoAr();
  taskPressaoDoAr();
  taskIndicadorDaMarcha();
  taskBlink();
  taskModu2();
}

/**************************************************************************************************************************************/
/* DEFINIÇÃO DAS TAREFAS                                                                                                              */
/**************************************************************************************************************************************/
//Comunicação com a CAN
void taskCanCommunication(void)
{
}

//Faz leitura do sensor da tempertarura do ar
void taskTemperaturaDoAr(void)
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

//SEGUNDO MODULO GY-521
void taskModu2(void)
{
  if (tmrAcele2Overflow)
  {
    //Serial.println("Acele");
    Wire.beginTransmission(MPU2); //Transmissao
    //Serial.println("Acele2");
    Wire.write(0x3B);             //Endereco 0x3B (ACCEL_XOUT_H)
    //Serial.println("Acele3");
    Wire.endTransmission(false);
    //Serial.println("Acele4");
    Wire.requestFrom(MPU2, 14, true); //Solicita os dados do sensor
    //Serial.println("Acele5");

    //Armazenamento dos valores do acelerometro e giroscopio
    Ac2X = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    Ac2Y = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    Ac2Z = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    //Tmp=Wire.read()<<8|Wire.read();
    //Wire.read() << 8 | Wire.read();        //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) <- Joga fora esses dados
    Gy2X = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    Gy2Y = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    Gy2Z = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Acx2 = (Ac2X / 16384) * 100; //  Dividido por 16384 para converter os valores para 1G,
    //Serial.println(Acx2);
    Acy2 = (Ac2Y / 16384) * 100; //  multiplicado por 100 para obter com precisao de duas
    //Serial.println(Acy2);
    Acz2 = (Ac2Z / 16384) * 100; //  casas decimais,
    //Serial.println(Acz2);
    Gyx2 = Gy2X / 131;           //  Dividido por 131 para converter os valores para graus/s.
    Gyy2 = Gy2Y / 131;
    Gyz2 = Gy2Z / 131;

    int iAcx2 = int(Acx2);    // Nova escala de 0 a 200
    //Serial.println(iAcx2);
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

   /* Serial.println(fAcx2);
    Serial.println(fAcy2);
    Serial.println(fAcz2);*/

    Modulo2Acc.data[0] = (fAcx2 >> 8) & 0xFF;
    Modulo2Acc.data[1] = fAcx2 & 0x0F;
    Modulo2Acc.data[2] = (fAcy2 >> 8) & 0xFF;
    Modulo2Acc.data[3] = fAcy2 & 0x0F;
    Modulo2Acc.data[4] = (fAcz2 >> 8) & 0xFF;
    Modulo2Acc.data[5] = fAcz2 & 0x0F;

    double teste1 = Modulo2Acc.data[1] * 256;
    double teste2 = teste1 + Modulo2Acc.data[0];
    Serial.println(teste2);

    Modulo2Gyro.data[0] = (fGyx2 >> 8) & 0xFF;
    Modulo2Gyro.data[1] = fGyx2 & 0x0F;
    Modulo2Gyro.data[2] = (fGyy2 >> 8) & 0xFF;
    Modulo2Gyro.data[3] = fGyy2 & 0x0F;
    Modulo2Gyro.data[4] = (fGyz2 >> 8) & 0xFF;
    Modulo2Gyro.data[5] = fGyz2 & 0x0F;

    tmrAcele2Overflow = false;

    if (mcp2515.sendMessage(&Modulo2Acc) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    } 
    if (mcp2515.sendMessage(&Modulo2Gyro) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }
  }
}

//Faz a leitura da posição da marcha
void taskIndicadorDaMarcha(void)
{
  if (tmrIndicadorDaMarchaOverflow)
  {
    unsigned int gear;
    unsigned char gearSent;

    gear = gearSelect();

    gearSent = gear & 0xFF;

    can_gear.data[0] = gearSent;

    tmrIndicadorDaMarchaOverflow = false;

    mcp2515.sendMessage(&can_gear);

    if (mcp2515.sendMessage(&can_gear) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }
    tmrBlinkOverflow = true;
  }
}

/**************************************************************************************************************************************/
/* FUNÇÕES DE TRATAMENTO DE INTERRUPÇÃO                                                                                               */
/**************************************************************************************************************************************/

/**************************************************************************************************************************************/
/* FUNÇÕES GERAIS                                                                                                                     */
/************************************************************************************************* *************************************/

void setupCAN()
{

  digitalWrite(LED_CPU, HIGH);
  CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);
}

int gearSelect()
{
  int count;
  int gear;
  int engaged = 0;

  for (count = INDICADOR_DA_PRIMEIRA_MARCHA; count <= INDICADOR_DA_SEXTA_MARCHA; count++)
  {
    //A ligação do hardware é importante para dar certo
    if (!digitalRead(count))
    {     //oi
      gear = count - (INDICADOR_DA_PRIMEIRA_MARCHA - 1);
    }
    else
    {
      engaged++;
    }
  }
  if (engaged == 6)
  {
    gear = 0;
    engaged = 0;
  }

  return gear;
}

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

void taskScheduler(void)
{
  if (tmrBlinkEnable)
  {
    tmrBlinkCount++;
    if (tmrBlinkCount >= TMR_BLINK / TMR_BASE)
    {
      tmrBlinkCount = 0;
      tmrBlinkOverflow = true;
    }
  }

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

  if (tmrIndicadorDaMarchaEnable)
  {
    tmrIndicadorDaMarchaCount++;
    if (tmrIndicadorDaMarchaCount >= TMR_INDICADOR_MARCHA / TMR_BASE)
    {
      tmrIndicadorDaMarchaCount = 0;
      tmrIndicadorDaMarchaOverflow = true;
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
  }else
  {
    //Serial.println("Deu ruim,");
  }
  

  if (tmrAcele2Enable)
  {
    tmrAcele2Count++;
    if (tmrAcele2Count >= TMR_ACELE2 / TMR_BASE)
    {
      tmrAcele2Count = 0;
      tmrAcele2Overflow = true;
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

void setupWIRE()
{  
  //Programação dos acelerômetros

  Wire.begin();
  Wire.beginTransmission(MPU2);
  Wire.write(0x6B); 
   
  //Inicializa o MPU-6050
  Wire.write(0); 
  Wire.endTransmission(true);
/*
  Wire.beginTransmission(MPU2); //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(false); //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU2); //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);             //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(false);   //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU2); //Start communication with the address found during search.
  Wire.write(0x1C);             //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);             //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  */
}