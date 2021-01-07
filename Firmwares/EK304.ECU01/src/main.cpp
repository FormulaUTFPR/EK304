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
void taskScheduler(void);
void taskBlink(void);

//DEFINICAO DAS PORTAS
#define PIN_SUSP_DIREITA A1  //Porta para o sensor da suspensao direita
#define PIN_SUSP_ESQUERDA A0 //Porta para o sensor da suspensao esquerda
#define LED_CPU 8            //Porta para o LED do módulo

#define CAN_SCK 13
#define CAN_SO 12
#define CAN_SI 11
#define CAN_CS 10

//VARIAVEIS GLOBAIS
#define VALOR_MIN_LEITURA_SUSP 126 //Minimo valor de leitura na porta analogica
#define VALOR_MAX_LEITURA_SUSP 876 //Maximo valor de leitura na porta analogica

// TIMERS

#define TMR_BASE 100000   //Clock base para os multiplicadores
#define TMR_SUSP 100000   //Timer para gravar dados da suspensão
#define TMR_ACELE1 100000 //Timer para gravar e enviar dados do acelerômetro 1
#define TMR_ACELE2 100000 //Timer para gravar e enviar dados do acelerômetro 2
#define TMR_BLINK 100000  //Timer para piscar o led

//Variáveis Globais

float Ac1X, Ac1Y, Ac1Z, Gy1X, Gy1Y, Gy1Z;
float Acx1, Acy1, Acz1, Gyx1, Gyy1, Gyz1;
float Ac2X, Ac2Y, Ac2Z, Gy2X, Gy2Y, Gy2Z;
float Acx2, Acy2, Acz2, Gyx2, Gyy2, Gyz2;
float posicaoSuspDireita, posicaoSuspEsquerda;
int fAcx1, fAcy1, fAcz1, fAcx2, fAcy2, fAcz2;
int fGyx1, fGyy1, fGyz1, fGyx2, fGyy2, fGyz2;
bool estadoLed = false;

//Acc Variables -- VARIÁVEIS NOVAS -- ATUALIZAR
int acc_error = 0;                          //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180 / 3.141592654;       //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;         //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;             //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;

//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;       //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

// AQUI ACABAM AS VARIÁVEIS NOVAS

//Variáveis para controle de Tarefas

bool tmrBlinkOverflow = false;
bool tmrBlinkEnable = false;
int tmrBlinkCount = 0;

bool tmrSuspOverflow = false;
bool tmrSuspEnable = false;
int tmrSuspCount = 0;

bool tmrAcele1Overflow = false;
bool tmrAcele1Enable = false;
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

MCP2515 mcp2515(CAN_CS); //Pino 10 é o Slave

void setup()
{
  pinMode(LED_CPU, OUTPUT);

  pinMode(PIN_SUSP_DIREITA, INPUT);
  pinMode(PIN_SUSP_ESQUERDA, INPUT);

  setupCAN();

  setupWIRE();

  SPI.begin();
  //Serial.begin(9600); //Usar somente para teste

  //Configura o TimerOne
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);

  tmrSuspEnable = true;
  tmrBlinkEnable = false;
  tmrAcele1Enable = false;
  tmrAcele2Enable = true;

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
  taskBlink();
}

void taskScheduler(void)
{
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
  { /*
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

    if (mcp2515.sendMessage(&Modulo1Acc) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    } 
    if (mcp2515.sendMessage(&Modulo1Gyro) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    
    
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);

    Ac1X = (Wire.read() << 8 | Wire.read()) / 4096.0; //each value needs two registres
    Ac1Y = (Wire.read() << 8 | Wire.read()) / 4096.0;
    Ac1Z = (Wire.read() << 8 | Wire.read()) / 4096.0;

    Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
    Wire.write(0x43);             //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true); //We ask for just 4 registers

    Gy1X = Wire.read() << 8 | Wire.read(); //Once again we shif and sum
    Gy1Y = Wire.read() << 8 | Wire.read();
    Gy1Z = Wire.read() << 8 | Wire.read();

    int iAcx1 = int(Ac1X); // Nova escala de 0 a 200
    int iAcy1 = int(Ac1Y); // Essa escala se refere a -1 a 1 G
    int iAcz1 = int(Ac1Z); // Aproximadamente 0 se refere a -1 G.

    int iGyx1 = int(Gy1X); // Nova escala de 0 a 250.
    int iGyy1 = int(Gy1Y); // Essa escala se refere a -250 a 250 graus/s.
    int iGyz1 = int(Gy1Z); // Aproximadamente 0 se refere a -250 graus/s.

    Modulo2Acc.data[0] = (iAcx1 >> 8) & 0xFF;
    Modulo2Acc.data[1] = iAcx1 & 0x0F;
    Modulo2Acc.data[2] = (iAcy1 >> 8) & 0xFF;
    Modulo2Acc.data[3] = iAcy1 & 0x0F;
    Modulo2Acc.data[4] = (iAcz1 >> 8) & 0xFF;
    Modulo2Acc.data[5] = iAcz1 & 0x0F;

    Modulo2Gyro.data[0] = (iGyx1 >> 8) & 0xFF;
    Modulo2Gyro.data[1] = iGyx1 & 0x0F;
    Modulo2Gyro.data[2] = (iGyy1 >> 8) & 0xFF;
    Modulo2Gyro.data[3] = iGyy1 & 0x0F;
    Modulo2Gyro.data[4] = (iGyz1 >> 8) & 0xFF;
    Modulo2Gyro.data[5] = iGyz1 & 0x0F;
  
    

    if (mcp2515.sendMessage(&Modulo2Acc) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    } 
    if (mcp2515.sendMessage(&Modulo2Gyro) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }*/
    tmrAcele2Overflow = false;
  }
}

//SEGUNDO MODULO GY-521
void taskModu2(void)
{
  if (tmrAcele2Overflow)
  {
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

//SUSPENSAO
void taskSusp(void)
{
  if (tmrSuspOverflow)
  {
    /*
    //Suspensao

    posicaoSuspDireita = (unsigned int)map(analogRead(PIN_SUSP_DIREITA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 255);
    posicaoSuspEsquerda = (unsigned int)map(analogRead(PIN_SUSP_ESQUERDA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 255);

    Suspensao.data[0] = (unsigned int)posicaoSuspDireita & 0xFF;  //Armazena o valor da leitura no primeiro byte do frame da suspensao
    Suspensao.data[1] = (unsigned int)posicaoSuspEsquerda & 0xFF; //Armazena o valor da leitura no segundo byte do frame da suspensao

    tmrSuspOverflow = false;

    mcp2515.sendMessage(&Suspensao);
    */

    //Suspensao

    unsigned int sender1 = analogRead(PIN_SUSP_DIREITA);

    unsigned int sender2 = analogRead(PIN_SUSP_ESQUERDA);

    Suspensao.data[0] = (sender1 >> 8) & 0xFF;
    Suspensao.data[1] = sender1 & 0xFF;

    Suspensao.data[2] = (sender2 >> 8) & 0xFF;
    Suspensao.data[3] = sender2 & 0xFF;

    if (mcp2515.sendMessage(&Suspensao) != MCP2515::ERROR::ERROR_OK)
    { // envia os dados de um CAN_Frame na CAN
      tmrBlinkEnable = false;
    }

    tmrSuspOverflow = false;
  }
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
  Modulo2Acc.can_id = EK304CAN_ID_ACC_01;
  Modulo2Acc.can_dlc = 6;

  Modulo2Gyro.can_id = EK304CAN_ID_GYRO_01;
  Modulo2Gyro.can_dlc = 6;

  //SUSPENSAO
  Suspensao.can_id = EK304CAN_ID_SUSP_FRONT;
  Suspensao.can_dlc = 4;
}

void setupWIRE()
{
  /*    USAR SOMENTE PARA VALIDAÇÃO EM PROTOBOARD -- TESTAR ANTES
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

  Wire.begin();                 //begin the wire comunication
  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);             //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(false); //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68); //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);             //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);             //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(false);  //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68); //Start communication with the address found during search.
  Wire.write(0x1C);             //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);             //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
}
