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

//CRIACAO DE TASKS
//void taskModu1(void); //Cria task do modulo 1
//void taskModu2(void); //Cria task do modulo 2
void taskSusp(void); //Cria a task para leitura e calcular a pressao
void taskCAN(void);  //Cria a task para o envio de dados
void taskScheduler(void);
void taskBlink(void);

//DEFINICAO DAS PORTAS
#define PIN_MODULO1 A0       //Porta para o modulo 1
#define PIN_MODULO2 A1       //Porta para o modulo 2
#define PIN_SUSP_DIREITA A2  //Porta para o sensor da suspensao direita
#define PIN_SUSP_ESQUERDA A3 //Porta para o sensor da suspensao esquerda
#define LED_CPU 8            //Porta para o LED do módulo

//VARIAVEIS GLOBAIS
#define VALOR_MIN_LEITURA_SUSP 126 //Minimo valor de leitura na porta analogica
#define VALOR_MAX_LEITURA_SUSP 876 //Maximo valor de leitura na porta analogica
#define TMR_BASE 100000            //Clock base para os multiplicadores
#define TMR_CANSEND 500000         //Timer para envios da can
#define TMR_BLINK 100000

float Ac1X, Ac1Y, Ac1Z, Gy1X, Gy1Y, Gy1Z;
float Acx1, Acy1, Acz1, Gyx1, Gyy1, Gyz1;
float Ac2X, Ac2Y, Ac2Z, Gy2X, Gy2Y, Gy2Z;
float Acx2, Acy2, Acz2, Gyx2, Gyy2, Gyz2;
float posicaoSuspDireita, posicaoSuspEsquerda;
int fAcx1, fAcy1, fAcz1, fAcx2, fAcy2, fAcz2;
int fGyx1, fGyy1, fGyz1, fGyx2, fGyy2, fGyz2;
bool tmrCansendOverflow = false;
bool tmrCansendEnable = false;
int tmrCansendCont = 0;
int alterna = 0;
bool tmrBlinkOverflow = false;
bool tmrBlinkEnable = false;
int tmrBlinkCount = 0;

//CAN
CAN_Frame Modulo1;
CAN_Frame Modulo2;
CAN_Frame Suspensao;
CAN_Frame Frameack;
MCP2515 mcp2515(10);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  SPI.begin();
  Wire.begin(); //Inicia I2C
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, HIGH);
  CAN_Init(&mcp2515, CAN_1000KBPS);
  digitalWrite(LED_BUILTIN, LOW);
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);
  tmrCansendEnable = true;

  //MODULO 1
  Modulo1.id.endOrigem = EK304CAN_ID_ADDRESS_ECU01;
  Modulo1.id.endDestino = EK304CAN_ID_ADDRESS_GTW;
  Modulo1.id.tipo = EK304CAN_ID_TYPE_SENSORDATA;
  Modulo1.msg.variant = 0x00;
  Modulo1.msg.length = 6;
  /*
  Modulo1.msg.data[0] = fAcx1;
  Modulo1.msg.data[1] = fAcy1;
  Modulo1.msg.data[2] = fAcz1;
  Modulo1.msg.data[3] = fGyx1;
  Modulo1.msg.data[4] = fGyy1;
  Modulo1.msg.data[5] = fGyz1;
  */

  //MODULO 2
  Modulo2.id.endOrigem = EK304CAN_ID_ADDRESS_ECU01;
  Modulo2.id.endDestino = EK304CAN_ID_ADDRESS_GTW;
  Modulo2.id.tipo = EK304CAN_ID_TYPE_SENSORDATA;
  Modulo2.msg.variant = 0x01;
  Modulo2.msg.length = 6;

  /*
  Modulo2.msg.data[0] = fAcx2;
  Modulo2.msg.data[1] = fAcy2;
  Modulo2.msg.data[2] = fAcz2;
  Modulo2.msg.data[3] = fGyx2;
  Modulo2.msg.data[4] = fGyy2;
  Modulo2.msg.data[5] = fGyz2;
  */

  //SUSPENSAO
  Suspensao.id.endOrigem = EK304CAN_ID_ADDRESS_ECU01;
  Suspensao.id.endDestino = EK304CAN_ID_ADDRESS_GTW;
  Suspensao.id.tipo = EK304CAN_ID_TYPE_SENSORDATA;
  Suspensao.msg.variant = 0x02;
  Suspensao.msg.length = 0x02;

  /*
  Suspensao.msg.data[0] = posicaoSuspDireita;
  Suspensao.msg.data[1] = posicaoSuspEsquerda;
  */

  /*
  Wire.beginTransmission(MPU1); //Inicia transmissao para o endereco do Modulo 1
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU2); //Inicia transmissao para o endereco do Modulo 2
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  */
}

void loop()
{
  //taskModu1();
  //taskModu2();
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

//PRIMEIRO MODULO GY-521
void taskModu1(void)
{
  Wire.beginTransmission(MPU1);     //Transmissao
  Wire.write(0x3B);                 //Endereco 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);      //Finaliza transmissao
  Wire.requestFrom(MPU1, 14, true); //Solicita os dados do sensor

  //Armazenamento dos valores do acelerometro e giroscopio
  Ac1X = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Ac1Y = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Ac1Z = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Gy1X = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Gy1Y = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Gy1Z = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Acx1 = (Ac1X / 16384) * 100; //  Dividido por 16384 para converter os valores para 1G,
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
  int fAcx1 = map(iiAcx1, 0, 220, 0, 200);
  int fAcy1 = map(iiAcy1, 0, 220, 0, 200);
  int fAcz1 = map(iiAcz1, 0, 220, 0, 200);

  int iGyx1 = int(Gyx1);    // Nova escala de 0 a 250.
  int iGyy1 = int(Gyy1);    // Essa escala se refere a -250 a 250 graus/s.
  int iGyz1 = int(Gyz1);    // Aproximadamente 0 se refere a -250 graus/s.
  int iiGyx1 = iGyx1 + 250; // Aproximadamente 125 se refere a 0 graus/s.
  int iiGyy1 = iGyy1 + 250; // Aproximadamente 250 se refere a 250 graus/s.
  int iiGyz1 = iGyz1 + 250;
  int fGyx1 = map(iiGyx1, 0, 500, 0, 250);
  int fGyy1 = map(iiGyy1, 0, 500, 0, 250);
  int fGyz1 = map(iiGyz1, 0, 500, 0, 250);

  Modulo1.msg.data[0] = fAcx1;
  Modulo1.msg.data[1] = fAcy1;
  Modulo1.msg.data[2] = fAcz1;
  Modulo1.msg.data[3] = fGyx1;
  Modulo1.msg.data[4] = fGyy1;
  Modulo1.msg.data[5] = fGyz1;

  //PARA TESTE:
  Serial.print("AcX = ");
  Serial.print(fAcx1);
  Serial.print("   AcY = ");
  Serial.print(fAcy1);
  Serial.print("   AcZ = ");
  Serial.print(fAcz1);
  Serial.print("   GyX = ");
  Serial.print(fGyx1);
  Serial.print("   GyY = ");
  Serial.print(fGyy1);
  Serial.print("   GyZ = ");
  Serial.println(fGyz1);
  Serial.println(" ");
}

//SEGUNDO MODULO GY-521
void taskModu2(void)
{
  Wire.beginTransmission(MPU2);     //Transmissao
  Wire.write(0x3B);                 //Endereco 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);      //Finaliza transmissao
  Wire.requestFrom(MPU2, 14, true); //Solicita os dados do sensor

  //Armazenamento dos valores do acelerometro e giroscopio
  Ac2X = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Ac2Y = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Ac2Z = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
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
  int fAcx2 = map(iiAcx2, 0, 215, 0, 200);
  int fAcy2 = map(iiAcy2, 0, 215, 0, 200);
  int fAcz2 = map(iiAcz2, 0, 205, 0, 200);

  int iGyx2 = int(Gyx2);    // Nova escala de 0 a 250.
  int iGyy2 = int(Gyy2);    // Essa escala se refere a -250 a 250 graus/s.
  int iGyz2 = int(Gyz2);    // Aproximadamente 0 se refere a -250 graus/s.
  int iiGyx2 = iGyx2 + 250; // Aproximadamente 125 se refere a 0 graus/s.
  int iiGyy2 = iGyy2 + 250; // Aproximadamente 250 se refere a 250 graus/s.
  int iiGyz2 = iGyz2 + 250;
  int fGyx2 = map(iiGyx2, 0, 500, 0, 250);
  int fGyy2 = map(iiGyy2, 0, 500, 0, 250);
  int fGyz2 = map(iiGyz2, 0, 500, 0, 250);

  Modulo2.msg.data[0] = fAcx2;
  Modulo2.msg.data[1] = fAcy2;
  Modulo2.msg.data[2] = fAcz2;
  Modulo2.msg.data[3] = fGyx2;
  Modulo2.msg.data[4] = fGyy2;
  Modulo2.msg.data[5] = fGyz2;

  //PARA TESTE:
  Serial.print("Ac2X = ");
  Serial.print(fAcx2);
  Serial.print("   Ac2Y = ");
  Serial.print(fAcy2);
  Serial.print("   Ac2Z = ");
  Serial.print(fAcz2);
  Serial.print("   Gy2X = ");
  Serial.print(fGyx2);
  Serial.print("   Gy2Y = ");
  Serial.print(fGyy2);
  Serial.print("   Gy2Z = ");
  Serial.println(fGyz2);
}

//SUSPENSAO
void taskSusp(void)
{
  posicaoSuspDireita = map(analogRead(PIN_SUSP_DIREITA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 90);
  posicaoSuspEsquerda = map(analogRead(PIN_SUSP_ESQUERDA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 90);

  Suspensao.msg.data[0] = posicaoSuspDireita;  //Armazena o valor da leitura no primeiro byte do frame da suspensao
  Suspensao.msg.data[1] = posicaoSuspEsquerda; //Armazena o valor da leitura no segundo byte do frame da suspensao
}

//ENVIO CAN
void taskCAN(void)
{
  if (tmrCansendOverflow)
  {
    alterna = !alterna;
    if (alterna)
    {
      CAN_SendData(&mcp2515, &Modulo1); // envia os dados de um CAN_Frame na CAN
      CAN_SendData(&mcp2515, &Modulo2); // envia os dados de um CAN_Frame na CAN
    }
    else
    {
      CAN_SendData(&mcp2515, &Suspensao); // envia os dados de um CAN_Frame na CAN
    }
    tmrCansendOverflow = false;
  }

  if (CAN_ReceiveData(&mcp2515, &Frameack) == MCP2515::ERROR_OK) // armazena em um CAN_Frame os dados recebidos da CAN
  {
    if (Frameack.id.tipo == EK304CAN_ID_TYPE_ACK)
    {
      if (Frameack.id.endDestino == EK304CAN_ID_ADDRESS_THIS)
      {
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_GTW); // envia um ACK na CAN
        tmrBlinkEnable = true;
      }
    }
  }
}