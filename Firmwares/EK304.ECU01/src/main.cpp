////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome do projeto: EK304.ECU01                                                                                                       //
// Nome do arquivo: main.cpp                                                                                                          //
// Desenvolvido por: Leonardo Cavagnari Guimaraes  |   Vitor Gervasi Adao                                                             //
// Data/versao: 12/11/2019 (v0.0.2)                                                                                                   //
// IDE utilizada: Visual Studio Code & PlatformIO                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <EK304CAN.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <TimerOne.h>

//ENDERECOS DOS MODULOS
const int MPU1 = 0x68; // Se o pino ADO for conectado em GND o modulo assume esse endereço
const int MPU2 = 0x69; // Se o pino ADO for conectado em 5V ou 3,3V o modulo assume esse endereço

//PROTOTIPOS DE FUNCOES

void setupCAN();
void setupWIRE();

//CRIACAO DE TASKS
void taskModu1(void); //Cria task do modulo 1
void taskModu2(void); //Cria task do modulo 2
void taskSusp(void);  //Cria a task para leitura e calcular a pressao
void taskCAN(void);   //Cria a task para o envio de dados
void taskScheduler(void);
void taskBlink(void);

//DEFINICAO DAS PORTAS
#define PIN_SUSP_DIREITA A2  //Porta para o sensor da suspensao direita
#define PIN_SUSP_ESQUERDA A3 //Porta para o sensor da suspensao esquerda
#define LED_CPU 8            //Porta para o LED do módulo

#define CAN_SCK 13
#define CAN_SO 12
#define CAN_SI 11
#define CAN_CS 10

//VARIAVEIS GLOBAIS
#define VALOR_MIN_LEITURA_SUSP 126 //Minimo valor de leitura na porta analogica
#define VALOR_MAX_LEITURA_SUSP 876 //Maximo valor de leitura na porta analogica

// TIMERS

#define TMR_BASE 100000     //Clock base para os multiplicadores
#define TMR_CANSEND 500000 //Timer para envios da can
#define TMR_SUSP 100000    //Timer para gravar dados da suspensão
#define TMR_ACELE1 100000  //Timer para gravar e enviar dados do acelerômetro 1
#define TMR_ACELE2 100000  //Timer para gravar e enviar dados do acelerômetro 2
#define TMR_BLINK 100000   //Timer para piscar o led

//Variáveis Globais

float Ac1X, Ac1Y, Ac1Z, Gy1X, Gy1Y, Gy1Z;
float Acx1, Acy1, Acz1, Gyx1, Gyy1, Gyz1;
float Ac2X, Ac2Y, Ac2Z, Gy2X, Gy2Y, Gy2Z;
float Acx2, Acy2, Acz2, Gyx2, Gyy2, Gyz2;
float posicaoSuspDireita, posicaoSuspEsquerda;
int fAcx1, fAcy1, fAcz1, fAcx2, fAcy2, fAcz2;
int fGyx1, fGyy1, fGyz1, fGyx2, fGyy2, fGyz2;
bool estadoLed = false;

//Variáveis para controle de Tarefas

bool tmrCansendOverflow = false;
bool tmrCansendEnable = true;
int tmrCansendCont = 0;

bool tmrBlinkOverflow = false;
bool tmrBlinkEnable = true;
int tmrBlinkCount = 0;

bool tmrSuspOverflow = false;
bool tmrSuspEnable = true;
int tmrSuspCount = 0;

bool tmrAcele1Overflow = false;
bool tmrAcele1Enable = true;
int tmrAcele1Count = 0;

bool tmrAcele2Overflow = false;
bool tmrAcele2Enable = false;
int tmrAcele2Count = 0;

//CAN
can_frame Modulo1Acc;
can_frame Modulo2Acc;
can_frame Modulo1Gyro;
can_frame Modulo2Gyro;
can_frame Suspensao;
can_frame Frameack;

MCP2515 mcp2515(CAN_CS); //Pino 10 é o Slave

void setup()
{
  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  setupCAN();
  
  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  setupWIRE();
  
  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  SPI.begin();
  //Serial.begin(9600);

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);

  //Configura o TimerOne
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  tmrCansendEnable = true;
  tmrSuspEnable = true;

  digitalWrite(LED_CPU, HIGH);
  delay(100);
  digitalWrite(LED_CPU, LOW);
  delay(100);
}

void loop()
{
  taskModu1();
  taskModu2();
  taskSusp();
  taskCAN();
  taskBlink();
}

void taskScheduler(void)
{
  if (tmrCansendEnable)
  {
    tmrCansendCont++;
    if (tmrCansendCont >= TMR_CANSEND / TMR_BASE)
    {
      tmrCansendOverflow = true;
      tmrCansendCont = 0;
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

  if (tmrAcele1Enable)
  {
    tmrAcele1Count++;
    if (tmrAcele1Count >= TMR_ACELE1 / TMR_BASE)
    {
      tmrAcele1Count = 0;
      tmrAcele1Overflow = true;
    }
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

  if (tmrBlinkEnable)
  {
    tmrBlinkCount++;
    if (tmrBlinkCount >= TMR_BLINK / TMR_BASE)
    {
      tmrBlinkCount = 0;
      tmrBlinkOverflow = true;
    }
  }
  else
  {
    tmrBlinkCount++;
    if (tmrBlinkCount >= 10 * TMR_BLINK / TMR_BASE)
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

//PRIMEIRO MODULO GY-521
void taskModu1(void)
{
  if (tmrAcele1Overflow)
  {
    Wire.beginTransmission(MPU1); //Transmissao
    Wire.write(0x3B);             //Endereco 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1, 14, true); //Solicita os dados do sensor

    //Armazenamento dos valores do acelerometro e giroscopio
    Ac1X = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    Ac1Y = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    Ac1Z = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    //Wire.read() << 8 | Wire.read();        //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) <- Joga fora esses dados
    Gy1X = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    Gy1Y = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    Gy1Z = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Acx1 = (Ac1X / 16384) * 100; //  Dividido por 16384 para converter os valores para G,
    Acy1 = (Ac1Y / 16384) * 100; //  multiplicado por 100 para obter com precisao de duas
    Acz1 = (Ac1Z / 16384) * 100; //  casas decimais.
    Gyx1 = Gy1X / 131;           //  Dividido por 131 para converter os valores para graus/s
    Gyy1 = Gy1Y / 131;
    Gyz1 = Gy1Z / 131;

    int iAcx1 = int(Acx1);    // Nova escala de 0 a 200.
    int iAcy1 = int(Acy1);    // Essa escala se refere a -1 a 1 G.
    int iAcz1 = int(Acz1);    // Aproximadamente 0 se refere a -1 G.
    int iiAcx1 = iAcx1 + 105; // Aproximadamente 100 se refere a 0 G.
    int iiAcy1 = iAcy1 + 105; // Aproximadamente 200 se refere a 1 G.
    int iiAcz1 = iAcz1 + 105;
    unsigned int fAcx1 = map(iiAcx1, 0, 220, 0, 200);
    unsigned int fAcy1 = map(iiAcy1, 0, 220, 0, 200);
    unsigned int fAcz1 = map(iiAcz1, 0, 220, 0, 200);

    int iGyx1 = int(Gyx1);    // Nova escala de 0 a 250.
    int iGyy1 = int(Gyy1);    // Essa escala se refere a -250 a 250 graus/s.
    int iGyz1 = int(Gyz1);    // Aproximadamente 0 se refere a -250 graus/s.
    int iiGyx1 = iGyx1 + 250; // Aproximadamente 125 se refere a 0 graus/s.
    int iiGyy1 = iGyy1 + 250; // Aproximadamente 250 se refere a 250 graus/s.
    int iiGyz1 = iGyz1 + 250;
    unsigned int fGyx1 = map(iiGyx1, 0, 500, 0, 250);
    unsigned int fGyy1 = map(iiGyy1, 0, 500, 0, 250);
    unsigned int fGyz1 = map(iiGyz1, 0, 500, 0, 250);

    Modulo1Acc.data[0] = (fAcx1 >> 8) & 0xFF;
    Modulo1Acc.data[1] = fAcx1 & 0x0F;
    Modulo1Acc.data[2] = (fAcy1 >> 8) & 0xFF;
    Modulo1Acc.data[3] = fAcy1 & 0x0F;
    Modulo1Acc.data[4] = (fAcz1 >> 8) & 0xFF;
    Modulo1Acc.data[5] = fAcz1 & 0x0F;

    Modulo1Gyro.data[0] = (fGyx1 >> 8) & 0xFF;
    Modulo1Gyro.data[1] = fGyx1 & 0x0F;
    Modulo1Gyro.data[2] = (fGyy1 >> 8) & 0xFF;
    Modulo1Gyro.data[3] = fGyy1 & 0x0F;
    Modulo1Gyro.data[4] = (fGyz1 >> 8) & 0xFF;
    Modulo1Gyro.data[5] = fGyz1 & 0x0F;

    tmrAcele1Overflow = false;

    mcp2515.sendMessage(&Modulo1Acc);  // envia os dados de um CAN_Frame na CAN
    mcp2515.sendMessage(&Modulo1Gyro); // envia os dados de um CAN_Frame na CAN
  }
}

//SEGUNDO MODULO GY-521
void taskModu2(void)
{
  if (tmrAcele2Overflow)
  {
    Wire.beginTransmission(MPU2);     //Transmissao
    Wire.write(0x3B);                 //Endereco 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);      //Finaliza transmissao
    Wire.requestFrom(MPU2, 14, true); //Solicita os dados do sensor

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

    Modulo2Acc.data[0] = (fAcx1 >> 8) & 0xFF;
    Modulo2Acc.data[1] = fAcx1 & 0x0F;
    Modulo2Acc.data[2] = (fAcy1 >> 8) & 0xFF;
    Modulo2Acc.data[3] = fAcy1 & 0x0F;
    Modulo2Acc.data[4] = (fAcz1 >> 8) & 0xFF;
    Modulo2Acc.data[5] = fAcz1 & 0x0F;

    Modulo2Gyro.data[0] = (fGyx1 >> 8) & 0xFF;
    Modulo2Gyro.data[1] = fGyx1 & 0x0F;
    Modulo2Gyro.data[2] = (fGyy1 >> 8) & 0xFF;
    Modulo2Gyro.data[3] = fGyy1 & 0x0F;
    Modulo2Gyro.data[4] = (fGyz1 >> 8) & 0xFF;
    Modulo2Gyro.data[5] = fGyz1 & 0x0F;

    tmrAcele2Overflow = false;

    mcp2515.sendMessage(&Modulo2Acc);  // envia os dados de um CAN_Frame na CAN
    mcp2515.sendMessage(&Modulo2Gyro); // envia os dados de um CAN_Frame na CAN
  }
}

//SUSPENSAO
void taskSusp(void)
{
  if (tmrSuspOverflow)
  {
    //Suspensao

    posicaoSuspDireita = (unsigned int)map(analogRead(PIN_SUSP_DIREITA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 255);
    posicaoSuspEsquerda = (unsigned int)map(analogRead(PIN_SUSP_ESQUERDA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 255);

    Suspensao.data[0] = (unsigned int)posicaoSuspDireita & 0xFF;  //Armazena o valor da leitura no primeiro byte do frame da suspensao
    Suspensao.data[1] = (unsigned int)posicaoSuspEsquerda & 0xFF; //Armazena o valor da leitura no segundo byte do frame da suspensao
                                                                                                                                                                    
    tmrSuspOverflow = false;

    if (mcp2515.sendMessage(&Suspensao) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }
  }
}

//ENVIO CAN
void taskCAN(void)
{
}

//Funções

void setupCAN()
{
  //Configura a CAN
  digitalWrite(LED_CPU, HIGH);
  CAN_Init(&mcp2515, CAN_500KBPS);
  digitalWrite(LED_CPU, LOW);

  //MODULO 1
  Modulo1Acc.can_id = EK304CAN_ID_ACC_01;
  Modulo1Acc.can_dlc = 6;

  Modulo1Gyro.can_id = EK304CAN_ID_GYRO_01;
  Modulo1Gyro.can_dlc = 6;

  //MODULO 2
  Modulo2Acc.can_id = EK304CAN_ID_ACC_02;
  Modulo2Acc.can_dlc = 6;

  Modulo2Gyro.can_id = EK304CAN_ID_GYRO_02;
  Modulo2Gyro.can_dlc = 6;

  //SUSPENSAO
  Suspensao.can_id = EK304CAN_ID_SUSP_FRONT;
  Suspensao.can_dlc = 2;
}

void setupWIRE()
{

  Wire.begin(); //Inicia I2C

  //------MPU1

  Wire.beginTransmission(MPU1); //Inicia transmissao para o endereco do Modulo 1  Wire.write(0x6B);
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true); //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU1); //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);             //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);   //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU1); //Start communication with the address found during search.
  Wire.write(0x1C);             //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);             //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(false);

/*
  //------MPU2
  Wire.beginTransmission(MPU2); //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true); //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU2); //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);             //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);   //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU2); //Start communication with the address found during search.
  Wire.write(0x1C);             //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);             //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(false);
  */
}
