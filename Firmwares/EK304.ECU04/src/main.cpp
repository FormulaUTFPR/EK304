#include <EK304CAN.h>
#include <mcp2515.h>
#include <TimerOne.h>
#include <SPI.h>
#include <Wire.h>

//Protótipos de funções

void pulsoOcorreu(void);
void taskVelocidade(void);
void taskPressao(void);
void taskCAN(void);
void taskAcele(void);
void taskSusp(void);
void taskTemp(void);
void taskScheduler(void);
void taskBlink(void);

//Definição das portas

#define PIN_VELOCIDADE 3 //Porta para o sensor de rotação
#define LED_CPU 8        //Porta para o LED de resposta da ACK

//Portas analogicas

#define PIN_TEMP_OLEO A1     //Porta para o sensor de temperatura do oleo
#define PIN_PRESSAO A0       //Porta para o sensor de pressão
#define PIN_SUSP_DIREITA A2  //Porta para o sensor da suspensao direita
#define PIN_SUSP_ESQUERDA A3 //Porta para o sensor da suspensao esquerda

//Endereço do acelerometro
const int MPU1 = 0x68; //Endereço é importante para pinagem

// FLAGS

//Enable é para ativar/desativar a função
//Overflow é pra chamar a função
//Count é o contador para o overflow

bool tmrCANSendEnable = false;
bool tmrCANSendOverflow = false;
int tmrCANSendCount = 0;

bool tmrTempEnable = false;
bool tmrTempOverflow = false;
int tmrTempCount = 0;

bool tmrPressaoEnable = false;
bool tmrPressaoOverflow = false;
int tmrPressaoCount = 0;

bool tmrSuspEnable = false;
bool tmrSuspOverflow = false;
int tmrSuspCount = 0;

bool tmrAceleEnable = false;
bool tmrAceleOverflow = false;
int tmrAceleCount = 0;

bool tmrBlinkOverflow = false;
bool tmrBlinkEnable = false;
int tmrBlinkCount = 0;

//Pinos da CAN - pinagem do modulo mcp2515

#define CAN_SCK 13
#define CAN_SI 11
#define CAN_SO 12
#define CAN_CS 10

//Setup da CAN

CAN_Frame canOUTROS; //Frame de Velocidade, suspensao, temperatura do oleo, pressao do oleo, posicao das suspensoes traseiras.
CAN_Frame canACEL;   //Frame pro acelerometro
CAN_Frame canACK;    //Frame ACK

//Variaveis globais

#define NUM_AMOSTRAGEM 5           //Numero de amostragens pra media do calculo da velocidade
#define NUM_IMAS 9                 //Numero de imãs na roda fônica
#define MAX_VELOCIDADE 10000       //Numero max de velocidade
#define VALOR_MIN_LEITURA_SUSP 117 //Minimo valor de leitura na porta analogica
#define VALOR_MAX_LEITURA_SUSP 914 //Maximo valor de leitura na porta analogica
#define VALOR_MIN_LEITURA_OLEO 870 //Minimo valor de leitura na porta analogica
#define VALOR_MAX_LEITURA_OLEO 216 //Maximo valor de leitura na porta analogica
#define VALOR_MIN_LEITURA_PRES 102 //Leitura minima de 0,5v --Ajustar
#define VALOR_MAX_LEITURA_PRES 921 //Leitura máxima de 4,5v --Ajustar
#define VALOR_MIN_PRES 0           //Valor min em bar
#define VALOR_MAX_PRES 10          //Valor máx em bar

#define TMR_BASE 100000     //Temporizador base para o Timer1
#define TMR_CANSEND 500000  //Chamar a função da CAN a cada 0,5 segundos
#define TMR_TEMP 1000000    //Leitura da temperatura a cada 0,1 segundo
#define TMR_PRESSAO 1000000 //Leitura da pressão a cada 0,1 segundo
#define TMR_SUSP 1000000    //Leitura dos dados da suspensão 0,1 segundo
#define TMR_ACELE 1000000   //Leitura dos dados do acelerômetro a cada 0,1 segundo
#define TMR_BLINK 100000    //Chamar a função para piscar o LED do módulo

//Declaração de variaveis globais

unsigned long long difTempo; //Valor da diferenca de tempo entre dois pulsos
unsigned int media;          //Media entre alguns velocidades para ter um valor com menos interferencias
unsigned long frequencia;    //Frequencia não suavizada
unsigned long velocidade;    //Velocidade nao suavizada - o suavizado é a media
unsigned long soma;          //Soma dos periodos para fazer a media
int contador;                //contador para fazer a media
unsigned long tempoInicial;  //Tempo em Microsegundos em que ocorreu o pulso

MCP2515 mcp2515(CAN_CS);

void setup()
{

  SPI.begin();
  Wire.begin(); //Inicia I2C
  Serial.begin(9600);

  //Modos das entradas

  pinMode(PIN_VELOCIDADE, INPUT_PULLUP); //Toda vez que tem uma subida chama a tarefa
  pinMode(PIN_PRESSAO, INPUT_PULLUP);    // ------------------------------------------------ checar o motivo de ser pullup
  pinMode(PIN_TEMP_OLEO, INPUT);
  pinMode(PIN_SUSP_DIREITA, INPUT);
  pinMode(PIN_SUSP_ESQUERDA, INPUT);
  pinMode(LED_CPU, OUTPUT); //LED do módulo

  //Definicao da CAN

  digitalWrite(LED_CPU, HIGH); //Deixa o LED ligado enquanto está settando a CAN
  CAN_Init(&mcp2515, CAN_1000KBPS);
  digitalWrite(LED_CPU, LOW); //Desliga o LED

  //Definicao dos ids e tamanhos e sensores

  canOUTROS.id.endOrigem = EK304CAN_ID_ADDRESS_THIS; //Endereço de origem do módulo 4
  canOUTROS.id.endDestino = EK304CAN_ID_ADDRESS_GTW; //Para o módulo 0
  canOUTROS.id.tipo = EK304CAN_ID_TYPE_SENSORDATA;   //Tipo de dado "dados"
  canOUTROS.msg.length = 5;                          //5 bytes
  canOUTROS.msg.variant = 0x00;                      //Pacote 1

  canACEL.id.endOrigem = EK304CAN_ID_ADDRESS_THIS; //Endereço de origem - módulo 4
  canACEL.id.endDestino = EK304CAN_ID_ADDRESS_GTW; //Para o módulo 0
  canACEL.id.tipo = EK304CAN_ID_TYPE_SENSORDATA;   //Tipo de dados "Dados"
  canACEL.msg.length = 6;                          //Tamanho do pacote
  canACEL.msg.variant = 0x01;                      //Pacote 2

  //Outros

  attachInterrupt(digitalPinToInterrupt(PIN_VELOCIDADE), taskVelocidade, RISING); //Quando o sensor passa de LOW pra HIGH, chama a funcao taskPulso

  Timer1.initialize(TMR_BASE);           //Inicializar a biblioteca Timer1 com o tempo de TMR_BASE
  Timer1.attachInterrupt(taskScheduler); //Define o escalonador

  //Variaveis globais para ativação das tarefas

  tmrCANSendEnable = true;
  tmrTempEnable = true;
  tmrSuspEnable = true;
  tmrPressaoEnable = true;
  tmrAceleEnable = true;

  Serial.println("Acabou o setup");

} //Aqui acaba o setup

void loop()
{
  taskCAN();
  taskAcele();
  taskPressao();
  taskSusp();
  taskTemp();
  taskBlink();
}

/*--------------------------------------------------*/
/*-------------------- Funções ---------------------*/
/*--------------------------------------------------*/

void taskVelocidade(void)
{
  if (contador >= NUM_AMOSTRAGEM) //Checa se o contador estorou
  {
    difTempo = micros() - tempoInicial; //Calcula a diferenca de tempo entre dois pulsos
    frequencia = 1000000 / difTempo;    //Faz o perídodo virar frequencia
    velocidade = frequencia * 60;       //Multiplica por 60 a frequencia para ter rotacoes por MINUTO
    media = velocidade * contador;      //Multiplica a velocidade por (idealmente) NUM_AMOSTRAGEM para ter a media entre os NUM_AMOSTRAGEM periodos
    media = media / NUM_IMAS;           //Divide a media pelo numero de imãs na roda fonica

    //media = media / NUM_IMAS; //Caso seja necessario, colocar numero de imas na roda em que as leituras são feitas

    //Serial.println(media);

    media = map(media, 0, MAX_VELOCIDADE, 0, 255); //Faz uma regra de 3 com a variável media

    canOUTROS.msg.data[0] = media; //Coloca o valor da média no pacote da CAN

    contador = 0;            //faz o contador voltar para 1
    tempoInicial = micros(); //Armazena o valor atual para calcular a diferença a próxima vez que for chamado
  }
  else
  {
    contador++; //Incrementa o contador
  }
} //Acaba a tarefa taskVelocidade

void taskTemp(void)
{
  if (tmrTempOverflow) //Checa a flag do overflow dessa tarefa
  {
    //Sensor MTE4053 (temperatura)

    canOUTROS.msg.data[1] = int((-1 / 0.038) * log((analogRead(PIN_TEMP_OLEO) * 1000) / (7656.8 * (5 - analogRead(PIN_TEMP_OLEO)))));

    //Checar se precisa alterar para enviar via CAN

    tmrTempOverflow = false; //Coloca a flag em falso novamente
  }
}

void taskPressao(void)
{
  if (tmrPressaoOverflow)
  {
    //Sensor de pressao

    float voltage;

    //pressao

    voltage = map(analogRead(PIN_PRESSAO), VALOR_MIN_LEITURA_PRES, VALOR_MAX_LEITURA_PRES, VALOR_MIN_PRES, VALOR_MAX_PRES); //Faz regra de  com o valor da leitura
    canOUTROS.msg.data[2] = (3.0 * (voltage - 0.47));                                                                       //Faz os cálculos para converter a tensao lida em pressao

    //Checar se precisa alterar o valor para a transmissão via CAN

    tmrPressaoOverflow = false;
  }
}

void taskSusp(void)
{
  if (tmrSuspOverflow)
  {
    //Suspensao

    canOUTROS.msg.data[3] = map(analogRead(PIN_SUSP_DIREITA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 90); //Leitura do valor do TPS e regra de 3 para enviar via CAN

    canOUTROS.msg.data[4] = map(analogRead(PIN_SUSP_ESQUERDA), VALOR_MIN_LEITURA_SUSP, VALOR_MAX_LEITURA_SUSP, 0, 90); //Análogo ao de cima

    tmrSuspOverflow = false;
  }
}

void taskCAN(void)
{
  if (CAN_ReceiveData(&mcp2515, &canACK) == MCP2515::ERROR_OK)
  {
    if (canACK.id.tipo == EK304CAN_ID_TYPE_ACK)
    {
      if (canACK.id.endDestino == EK304CAN_ID_ADDRESS_THIS)
      {
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_GTW);
        tmrBlinkEnable = true;
      }
    }
  }

  if (tmrCANSendOverflow)
  {
    //Envios

    CAN_SendData(&mcp2515, &canOUTROS);

    tmrCANSendOverflow = false;
  }
}

void taskAcele(void) //Tarefa do acelerometro
{
  if (tmrAceleOverflow)
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
    GyX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    Acx1 = (AcX / 16384) * 100; //  Dividido por 16384 para converter os valores para 1G,
    Acy1 = (AcY / 16384) * 100; //  multiplicado por 100 para obter com precisao de duas
    Acz1 = (AcZ / 16384) * 100; //  casas decimais.
    Gyx1 = GyX / 131;           //  Dividido por 131 para converter os valores para graus/s
    Gyy1 = GyY / 131;
    Gyz1 = GyZ / 131;

    int iAcx1 = int(Acx1);    // Nova escala de 0 a 200.
    int iAcy1 = int(Acy1);    // Essa escala se refere a -1 a 1 G.
    int iAcz1 = int(Acz1);    // Aproximadamente 0 se refere a -1 G.
    int iiAcx1 = iAcx1 + 105; // Aproximadamente 100 se refere a 0 G.
    int iiAcy1 = iAcy1 + 105; // Aproximadamente 200 se refere a 1 G.
    int iiAcz1 = iAcz1 + 105;
    fAcx1 = map(iiAcx1, 0, 220, 0, 200);
    fAcy1 = map(iiAcy1, 0, 220, 0, 200);
    fAcz1 = map(iiAcz1, 0, 220, 0, 200);

    int iGyx1 = int(Gyx1);    // Nova escala de 0 a 250.
    int iGyy1 = int(Gyy1);    // Essa escala se refere a -250 a 250 graus/s.
    int iGyz1 = int(Gyz1);    // Aproximadamente 0 se refere a -250 graus/s.
    int iiGyx1 = iGyx1 + 250; // Aproximadamente 125 se refere a 0 graus/s.
    int iiGyy1 = iGyy1 + 250; // Aproximadamente 250 se refere a 250 graus/s.
    int iiGyz1 = iGyz1 + 250;
    int fGyx1 = map(iiGyx1, 0, 500, 0, 250);
    int fGyy1 = map(iiGyy1, 0, 500, 0, 250);
    int fGyz1 = map(iiGyz1, 0, 500, 0, 250);

    canACEL.msg.data[0] = fAcx1;
    canACEL.msg.data[1] = fAcy1;
    canACEL.msg.data[2] = fAcz1;
    canACEL.msg.data[3] = fGyx1;
    canACEL.msg.data[4] = fGyy1;
    canACEL.msg.data[5] = fGyz1;

    CAN_SendData(&mcp2515, &canACEL);

    tmrAceleOverflow = false;
  }
}

void taskScheduler(void) //Aqui comeca o escalonador
{
  if (tmrCANSendEnable)
  {
    tmrCANSendCount++;                             //Incrementa o valor dessa variável todo ciclo
    if (tmrCANSendCount >= TMR_CANSEND / TMR_BASE) //Checa se o valor estorou
    {
      tmrCANSendCount = 0;       //Volta o valor para 0
      tmrCANSendOverflow = true; //Ativa o overflow para a tarefa ser ativada
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

  if (tmrPressaoEnable)
  {
    tmrPressaoCount++;
    if (tmrPressaoCount >= TMR_PRESSAO / TMR_BASE)
    {
      tmrPressaoCount = 0;
      tmrPressaoOverflow = true;
    }
  }

  if (tmrAceleEnable)
  {
    tmrAceleCount++;
    if (tmrAceleCount >= TMR_ACELE / TMR_BASE)
    {
      tmrAceleCount = 0;
      tmrAceleOverflow = true;
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