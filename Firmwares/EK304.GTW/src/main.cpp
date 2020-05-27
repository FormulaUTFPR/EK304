////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Nome do projeto: EK304.GTW                                                                                                         //
// Nome do arquivo: main.cpp                                                                                                          //
// Desenvolvido por: Rafael Ramalho | @RamalhoFael                                                                                    //
// Data/versão: 03/11/2019 (v0.0.1)                                                                                                   //
// IDE utilizada: Visual Studio Code & PlatformIO                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**************************************************************************************************************************************/
/* CONFIGURAÇÕES DO PROJETO                                                                                                           */
/**************************************************************************************************************************************/
#define FIRMWARE_VERSION "v0.1.2"

#define TMR_BASE 100000
#define TMR_BLINK 1000000
#define TMR_STARTUP_LOGO 1000000
#define TMR_STARTUP_NAME 1000000
#define TMR_ERRORMSGS 2000000
#define TMR_CANTEST 1000000

#define TMR_CANCHECKERROR 1500 // millisecs()

#define TMR_DEBOUNCING_ENCODER 200000
#define TMR_GPS_TIMEOUT 2000000

#define GPS_DATA_NULL 0x00
#define GPS_DATA_GNZDA 0x01
#define GPS_DATA_GPGSA 0x02
#define GPS_DATA_GNGLL 0x03

#define GPS_COMMAND_DATETIME "$GPZDA" // identificador de dados, string de informacoes de data/hora
#define GPS_COMMAND_POS "$GPGGA"      // identificador de dados, string de informacoes de fix e *dop

#define GPS_DATA_DATE 0x01
#define GPS_DATA_COORD 0x02

// ids dos sensores
#define SENSOR_NULL_ID 0x00
#define SENSOR_SPEED_ID 0x01
#define SENSOR_ROTATION_ID 0x02
#define SENSOR_MOTOR_TEMPERATURE_ID 0x03
#define SENSOR_GEAR_ID 0x04
#define SENSOR_GPS_LATITUDE_ID 0x05
#define SENSOR_GPS_LONGITUDE_ID 0x06
#define SENSOR_GPS_ALTITUDE_ID 0x07
#define SENSOR_GPS_FIX_ID 0x08
#define SENSOR_GPS_HDOP_ID 0x09
#define SENSOR_VOLTAGE_BATTERY_ID 0x0A
#define SENSOR_MAP_ID 0x0B
#define SENSOR_AIR_INTAKE_TEMPERATURE_ID 0x0C
#define SENSOR_WATER_TEMPERATURE_ID 0x0D
#define SENSOR_TPS_ID 0x0E
#define SENSOR_LAMBDA_ID 0x0F
#define SENSOR_OIL_PRESSURE_ID 0x10
#define SENSOR_OIL_TEMPERATURE_ID 0x11
#define SENSOR_REAR_LEFT_SUSPENSION_ID 0x12
#define SENSOR_REAR_RIGHT_SUSPENSION_ID 0x13
#define SENSOR_FRONT_LEFT_SUSPENSION_ID 0x14
#define SENSOR_FRONT_RIGHT_SUSPENSION_ID 0x15

// parâmetros dos sensores
#define SENSOR_GEAR_MAX 6
#define SENSOR_GEAR_MIN 0
#define SENSOR_GPSHDOP_REF 2.0
#define SENSOR_VBAT_SAMPLES 10
#define SENSOR_VBAT_OVERVOLTAGE 14.5
#define SENSOR_VBAT_UNDERVOLTAGE 10.5
#define CONST_RPM 51             //255*51 =~ 13000 RPM
#define SENSOR_SUSPENSION_MAX 90 //Ângulo máximo da suspensão
#define SENSOR_SUSPENSION_MIN 0  //Ângulo mínimo da suspensão
#define SENSOR_QTTY_TOTAL 50     //Número de sensores

// strings
#define STRING_DISPLAY_TEAMNAME "Formula UTFPR"
#define STRING_DISPLAY_CARNAME "EK-304"
#define STRING_DISPLAY_DATETIME_MAX 33

#define STRING_FILE_NAME "ola2.csv"

// erros
#define ERROR_MSGSIZE 25

// habilitação das ecus

#define EK304CAN_ECU01_ENABLED true
#define EK304CAN_ECU02_ENABLED true
#define EK304CAN_ECU03_ENABLED true
#define EK304CAN_ECU04_ENABLED true
#define EK304CAN_ECU15_ENABLED false

/**************************************************************************************************************************************/
/* DEFINIÇÃO DE PINOS                                                                                                                 */
/**************************************************************************************************************************************/
// pinos utilizados pelo display de lcd
#define LCD_RS 37 // define LCD_RS no pino 38
#define LCD_EN 39 // define LCD_EN no pino 40
#define LCD_RW 38 // define LCD_RW no pino 39
#define LCD_RE 48 // define LCD_RE no pino 49
#define LCD_D7 47 // define LCD_D7 no pino 48
#define LCD_D6 46 // define LCD_D6 no pino 47
#define LCD_D5 45 // define LCD_D5 no pino 46
#define LCD_D4 44 // define LCD_D4 no pino 45
#define LCD_D3 43 // define LCD_D3 no pino 44
#define LCD_D2 42 // define LCD_D2 no pino 43
#define LCD_D1 41 // define LCD_D1 no pino 42
#define LCD_D0 40 // define LCD_D0 no pino 41

// pinos utilizados pelo encoder
#define ENC_CL 2 // define ENC_CL (pino clock do encoder) no pino 2
#define ENC_DT 4 // define ENC_DT (pino data do encoder) no pino 4
#define ENC_SW 3 // define ENC_SW (switch do encoder)no pino 3

// pinos utilizados pela SPI
#define SPI_SCK 52    // define SPI_SCK (clock do SPI bus) no pino 52
#define SPI_SI 51     // define SPI_SI  (slave input do SPI bus) no pino 51
#define SPI_SO 50     // define SPI_SO  (slave output do SPI bus) no pino 50
#define SPI_CS_CAN 53 // define SPI_CS_CAN  (slave select da CAN no SPI bus) no pino 53
#define SPI_CS_SD 49  // define SPI_CS_SD  (slave select do módulo SD no SPI bus) no pino 49

// pinos utilizados como entrada analógica
#define V_BAT A0 // define V_BAT no pino A0

// pinos para os LEDs

#define LED_RUN 22
#define LED_ERR 23

/**************************************************************************************************************************************/
/* IMPORTAÇÃO DE BIBLIOTECAS                                                                                                          */
/**************************************************************************************************************************************/
#include <U8glib.h>      // importa biblioteca de funções do display de LCD 128x64
#include <TimerOne.h>    // importa bibliotreca de funções do timer1
#include <SPI.h>         // importa biblioteca de funções do protocolo SPI
#include <mini-printf.h> // importa biblioteca de funções de string (versão leve)
#include <mcp2515.h>     // importa biblioteca de funções módulo CAN transcievers (baixo nível)
#include <LinkedList.h>
#include <SD.h>

#include <EK304ERRORS.h>
#include <EK304SETTINGS.h>

#include <EK304SD.h>
#include <EK304CAN.h> // importa biblioteca de funções para a CAN (alto nivel)

/**************************************************************************************************************************************/
/* DECLARAÇÃO DAS TAREFAS                                                                                                             */
/**************************************************************************************************************************************/
void taskDisplay();       // tarefa de atualização do display
void taskGPS();           // tarefa de interface com o módulo GPS
void taskCAN();           // tarefa de interface com om módulo CAN transciever
void taskBlink();         // tarefa de acionamento do led
void taskMain();          // tarefa principal (máquina de estados)
void taskScheduler();     // tarefa do controle dos tempos (escalonador)
void taskSerial();        // tarefa de controle da serial
void taskEncoder();       // tarefa de leitura do encoder
void taskErrorsMonitor(); // tarefa de monitoração de erros no sistema
void taskADC();           // tafera de leitura das entradas analógicas

/**************************************************************************************************************************************/
/* PROTÓTIPO DAS FUNÇÕES                                                                                                              */
/**************************************************************************************************************************************/
void u8gPrepare();             // comandos de preparaÃ§Ã£o para esvrever no display
void u8gDraw(int showDisplay); // comando para desenhar no display

void scrLogo();     // tela 1: logotipo da equipe
void scrWelcome();  // tela 2: nome da equipe/carro
void scrOverview(); // tela 3: visão geral (sensores selecionados)
void scrGPS();      // tela 4: mostra informacoes recebidas pelo modulo GPS

void setupDisplay();  // configura display
void setupSerial();   // configura a serial
void setupCAN();      // configura o módulo CAN transciever e a respectiva tarefa
void setupTimer();    // configura os timers do sistema (escalonador)
void setupEncoder();  // configura a tarefa de leitura do encoder
void setupKernel();   // configura e inicializa o Kernel
void setupBlink();    // configura a tarefa de sinalização do sistema
void setupErrors();   // configura a tarefa de
void setupInit();     // configura o início do sistema
void setupADC();      // configura os conversoes A/D
void setupSensors();  // configura os sensores
void setupSDModule(); // configura o módulo SDCard

void SDWrite(String fileName, String data);
void SDInit(int SC_PIN);

void isrEncoderSpin();  // trata interrupção do giro do encoder
void isrEncoderClick(); // trata interrupção do clique do encoder

void setAckTime(int origin);
void sendSettingsSetup();

/**************************************************************************************************************************************/
/* DEFINIÇÃO DE TIPOS NÃO PRIMITIVOS                                                                                                   */
/**************************************************************************************************************************************/
// armazena as inforações dos sensores
class Sensor
{
public:
  String nome;
  String unidade;
  long valor;
  unsigned long canID;
  unsigned long ultimoRecebimento;
  int origin;
  int frequency;

  Sensor()
  {
    nome = "";
    unidade = "";
    valor = 0.0;
    canID = 0;
    origin = 0;
    frequency = 0;
  }

  Sensor(String n, String u, unsigned long id, int o, int f)
  {
    nome = n;
    unidade = u;
    valor = 0.0;
    canID = id;
    origin = o;
    frequency = f;
  }

  String ToString()
  {
    String s = nome;
    s += ": ";
    s += valor;
    s += " ";
    s += unidade;

    return s;
  }
};

class Error
{
public:
  String name; // nome do erro
  int id;      // identificador do erro (índice no vetor)
  long code;   // código de erro (formato 0x0000)
  bool status; // status do erro (true = erro presente)
  String msg;  // descrição do erro

  Error()
  {
    name = "";
    id = 0;
    code = 0;
    status = false;
    msg = "";
  }

  Error(String n, String m, int id, long c)
  {
    name = n;
    msg = m;
    id = id;
    code = c;
    status = false;
  }

  String ToStringCenter(int max)
  {
    int l = msg.length();
    if (l >= max)
      return msg;

    String s;
    int spaces = (max - l) / 2;
    for (int i = 0; i < spaces; i++)
      s += " ";
    s += msg;
    return s;
  }
};

// armazena as informações do encoder
struct Encoder
{
  bool botao; // estado do botão (0 não pressionado, 1 pressionado)
  char giro;  // sentido de giro (-1 antihorário, 0 parado, 1 horário)
};

struct Acc
{
  int id;
  long accX;
  long accY;
  long accZ;
  long tiltX;
  long tiltY;
  long tiltZ;
};

// armazena as informações de data/hora
class DateTime
{
public:
  int day;     // dia
  int month;   // mes
  int year;    // ano
  int hours;   // horas
  int minutes; // minutos
  int seconds; // segundos

  DateTime()
  {
    day = 1;
    month = 1;
    year = 2000;

    hours = 0;
    minutes = 0;
    seconds = 0;
  }

  DateTime(int d, int mo, int y, int h, int m, int s)
  {
    day = d;
    month = mo;
    year = y;

    hours = h;
    minutes = m;
    seconds = s;
  }

  String ToString()
  {
    String s = "";
    s += hours / 10;
    s += hours % 10;
    s += ":";
    s += minutes / 10;
    s += minutes % 10;
    s += "  ";
    s += month / 10;
    s += month % 10;
    s += "/";
    s += day / 10;
    s += day % 10;
    s += "/";
    s += year / 1000;
    s += (year % 1000) / 100;
    s += ((year % 1000) % 100) / 10;
    s += ((year % 1000) % 100) % 10;
    return s;
  }

  String ToStringCenter(int max)
  {
    String temp = ToString();
    int l = temp.length();

    if (l >= max)
      return temp;

    String s = "";
    int spaces = (max - l) / 2;
    for (int i = 0; i < spaces; i++)
    {
      s += "  ";
    }
    s += temp;

    return s;
  }

  bool IsValid()
  {
    if (day > 1 && day <= 31 && month >= 1 && month <= 12 && year >= 2000 && year <= 2100 && hours >= 0 && hours <= 23 && minutes >= 0 && minutes < 59 && seconds >= 0 && seconds <= 59)
      return true;
    else
      return false;
  }
};

// armazena o estado atual do sistema
enum StateMachine
{
  STATE_STARTUP_LOGO, // mostra o logo da equipe
  STATE_STARTUP_NAME, // mostra o nome da equipe
  STATE_OVERVIEW,     // mostra a tela de visão geral
  STATE_MENU          // mostra menu de configurações
};

/**************************************************************************************************************************************/
/* DECLARAÇÃO DE VARIÁVEIS E CONSTANTES GLOBAIS                                                                                       */
/**************************************************************************************************************************************/
// imagens
const unsigned char logo_bits[] PROGMEM{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00,
    0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00,
    0x00, 0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1e, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x0e, 0x70, 0x00, 0x78, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff,
    0xff, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xfc, 0xff, 0xf1, 0x8f, 0xff, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0x20, 0x04, 0xff, 0x3f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x41,
    0x04, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7c, 0x00, 0x80, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xbc, 0x00, 0x00, 0x7f, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf6, 0x01,
    0x80, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0xf3, 0x03, 0x80, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf9, 0x07, 0xc0, 0xff, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xb8, 0x07,
    0xe0, 0xfd, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xf8, 0x7f, 0x1f, 0xf8, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x18, 0xf0, 0xff, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00,
    0xe0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7e, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x08, 0xe0, 0x7f, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xde, 0x05,
    0xe0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x9e, 0x05, 0xe0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8f, 0x0f, 0xf0, 0xff, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0f, 0x37,
    0xf8, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x80, 0x0f, 0x6f, 0xfc, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0xce, 0xe3, 0xff, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0xfe,
    0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xe0, 0x0f, 0xfe, 0xff, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x0f, 0xfe, 0xff, 0x7f, 0x07, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x1e, 0xfc,
    0xff, 0x7f, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xf0, 0x1c, 0x7c, 0xfe, 0x7f, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x1c, 0xf8, 0xff, 0x7f, 0x0e, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x1c, 0xf0,
    0xff, 0x3f, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x78, 0x38, 0xe0, 0xff, 0xbf, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x38, 0x00, 0xfc, 0x9f, 0x0f, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x30, 0x00,
    0xff, 0xdf, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xe0, 0x71, 0xe0, 0xff, 0xef, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x63, 0xf0, 0xff, 0xef, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x67, 0xf0,
    0xff, 0xf7, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xcf, 0xf0, 0xff, 0xfb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9e, 0xf0, 0xff, 0x7f, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0xf1,
    0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x78, 0xe0, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xe0, 0xff, 0x0f, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xe1,
    0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0xe3, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0xff, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xde,
    0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xb8, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00};

// flags
bool flagEncoderClickEvent = false;    // sinaliza quando houve um evento de clique do encoder
bool flagEncoderSpinEvent = false;     // sinaliza quando houve um evento de giro do encoder
bool flagEncoderClockwiseSpin = false; // sinaliza o sentido de giro o encoder (horário = true)

bool flagFreezeDisplay = false; // sinaliza quando o display deve estar congelado
bool flagUpdateDisplay = false; // sinaliza quando o display quando o display deve ser atualizado

bool flagErrorsFound = false;     // sinaliza quando houve(m) falha(s) no sistema
bool flagErrorSDCardInit = false; // sinaliza quando há erro na inicialização do cartão SD
bool flagErrorSDCardFull = false; // sinaliza quando o cartão SD está ficando cheio ou está cheio

bool flagErrorPacketID = false; // sinaliza quando houve(m) falha(s) no sistema

bool intEncoderClockEnabled = true;  // habilita a interrupção no clock do encoder
bool intEncoderButtonEnabled = true; // habilita a interrupção no botão do encoder

// cria uma máquina de estados
StateMachine stateMachine;

// Scheduler
bool tmrStartupLogoEnabled = false;  // habilita o timer para o tempo da tela "Startup Logo"
bool tmrStartupLogoOverflow = false; // sinaliza quando houve estouro do contator do tempo da tela ""
int tmrStartupLogoCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para o "Startup Logo"

bool tmrStartupNameEnabled = false;  // habilita o timer para o tempo da tela "Startup Name"
bool tmrStartupNameOverflow = false; // sinaliza quando houve estouro do contator do tempo da tela "Startup Name"
int tmrStartupNameCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para o "Startup Name"

bool tmrBlinkEnabled = false;  // habilita o timer para a tarefa de acionamento do led
bool tmrBlinkOverflow = false; // sinaliza quando houve estouro do contator do tempo usado apra acionamento do led
int tmrBlinkCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para a tarefa de acionamento do led

bool tmrErrorMsgsEnabled = false;  // habilita o timer para a transição de mensagens de erro na tela
bool tmrErrorMsgsOverflow = false; // sinaliza quando houve estouro do contator do tempo usado para transição das mensagens de erro na tela
int tmrErrorMsgsCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para a tarefa de

bool tmrSpinDebouncingEnabled = false;  // habilita o timer para o tempo de debouncing do giro do encoder
bool tmrSpinDebouncingOverflow = false; // sinaliza quando houve estouro do contator do tempo de debouncing do giro do encoder
int tmrSpinDebouncingCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para o debouncing do giro do encoder

bool tmrGPSTimeoutEnabled = false;  // habilita o timer para o tempo de recebimento do ultimo dado do gps
bool tmrGPSTimeoutOverflow = false; // sinaliza quando houve estouro do contator do tempo de recebimento do ultimo dado do gps
int tmrGPSTimeoutCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para o tempo de recebimento do ultimo dado do gps

bool tmrCanTestEnabled = false;  // habilita o timer para o tempo de teste de comunicação CAN com os módulos
bool tmrCanTestOverflow = false; // sinaliza quando houve estouro do contator dotempo de teste de comunicação CAN com os módulos
int tmrCanTestCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para o tempo de teste de comunicação CAN com os módulos

bool tmrCanCheckErrorEnabled = false;  // habilita o timer para o tempo de teste de comunicação CAN com os módulos
bool tmrCanCheckErrorOverflow = false; // sinaliza quando houve estouro do contator dotempo de teste de comunicação CAN com os módulos
int tmrCanCheckErrorCount = 0;         // conta quantos múltiplos de TMR_BASE se passaram para o tempo de teste de comunicação CAN com os módulos

// informações da tela
Sensor senOverview[4]; // armazena uma cópia dos valores dos sensores (buffer) a serem mostrados no display
DateTime horaAtual;    // armazena a hora a ser mostrada na tela
int telaAtual;

bool gpsFlagNewData = false; // indica quando um novo dado (válido ou não) chegou pelo GPS
bool flagCheckSum = false;   // indica quando está sendo verificado o checksum
int contCheckSum = 0;        // conta quando dados do checksum já foram recebidos
int gpsData = 0;             // identifica qual o dado está sendo recebido pelo GPS

// sensores
Sensor sensors[SENSOR_QTTY_TOTAL];
Acc acc[2];
//Acelerometros

// erros
Error sysErrors[ERROR_AMOUNT]; // vetor de erros do sistema
LinkedList<String> sMessages;
int selectedError = 0;
char cErrorMsg[ERROR_MSGSIZE];   // buffer para transmissão da mensagem de erro
int cntErrors = 0;               // conta quantos erros estão presentes no sistema (em tempo real)
long tmpRecebimentoAckECU01 = 0; // tempo do ultimo recebimento do ACK da ECU01
long tmpRecebimentoAckECU02 = 0; // tempo do ultimo recebimento do ACK da ECU02
long tmpRecebimentoAckECU03 = 0; // tempo do ultimo recebimento do ACK da ECU03
long tmpRecebimentoAckECU04 = 0; // tempo do ultimo recebimento do ACK da ECU04
long tmpRecebimentoAckECU15 = 0; // tempo do ultimo recebimento do ACK da ECU15

// encoder
int encContadorPosicao; // armazena o valor do contador de posição do encoder (variável auxilar)
int encUltimaPosicao;   // armazena a ultima posição do encoder (variável auxilar)
int encLeituraClock;    // armazena a leitura do pino CLK do encoder (variável auxilar)
bool encSentidoHorario; // armazena o sentido de rotação do encoder (variável auxilar)

Encoder encoder; // cria uma instância do encoder

// can
can_frame canMsg; // armazena as informações do frame da CAN
can_frame frame;

// serial
String bufSerial; // buffer para transmissão na serial

// adc
int amostrasVBat = 0;
float ValorVBat = 0.0;

/**************************************************************************************************************************************/
/* CONFIGURAÇÃO DOS PERIFÉRICOS                                                                                                       */
/**************************************************************************************************************************************/

// cria instância do display de LCD (configura pinos) https://github.com/olikraus/u8glib/wiki/device#st7920-128x64
// U8GLIB_ST7920_128X64_1X(sck, mosi, cs [, reset])

//U8GLIB_ST7920_128X64_1X u8g(LCD_D0, LCD_D1, LCD_D2, LCD_D3, LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_EN, LCD_RS, LCD_RW, LCD_RE);

U8GLIB_ST7920_128X64_1X u8g(LCD_EN, LCD_RW, LCD_RS); //SPI-

// cria uma instância do módulo transciever CAN (configura pino CS)
MCP2515 mcp2515(SPI_CS_CAN);

// cria uma instância do módulo SDCard
EK304SD sdCard;

// Rotina principal de configuração
void setup()
{
  pinMode(LED_ERR, OUTPUT);

  setupDisplay();  // configura display
  setupSerial();   // configura serial
  setupADC();      // confgura o conversor A/D
  setupCAN();      // configura CAN
  setupSDModule(); // configura o módulo SDCard

  setupEncoder(); // configura Encoder
  setupTimer();   // configura os timers e as respectivas interrupções
  setupBlink();   // configura a arefa de blink
  setupErrors();  // configura as mensagens de erro do sistema
  setupSensors(); // configura as informações dos sensores
  setupInit();    // configura e inicializa o Kernel
}

void setupErrors()
{
  sysErrors[ERROR_CAN_INIT_ID] = Error("CAN_INIT", "Master initialization failed", ERROR_CAN_INIT_ID, ERROR_CAN_INIT_CODE);
  sysErrors[ERROR_GPS_NOTCONNECTED_ID] = Error("GPS_NOTCONNECTED", "GPS device not found", ERROR_GPS_NOTCONNECTED_ID, ERROR_GPS_NOTCONNECTED_CODE);
  sysErrors[ERROR_GPS_LOWSIGNAL_ID] = Error("GPS_LOWSIGNAL", "Low GPS acurracy", ERROR_GPS_LOWSIGNAL_ID, ERROR_GPS_LOWSIGNAL_CODE);
  sysErrors[ERROR_GPS_NOSIGNAL_ID] = Error("GPS_NOSIGNAL", "No GPS signal", ERROR_GPS_NOSIGNAL_ID, ERROR_GPS_NOSIGNAL_CODE);
  sysErrors[ERROR_ELECTRICAL_OVERVOLTAGE_ID] = Error("ELECTRICAL_OVERVOLTAGE", "Overvoltage detected", ERROR_ELECTRICAL_OVERVOLTAGE_ID, ERROR_ELECTRICAL_OVERVOLTAGE_CODE);
  sysErrors[ERROR_ELECTRICAL_UNDERVOLTAGE_ID] = Error("ELECTRICAL_UNDERVOLTAGE", "Undervoltage detected", ERROR_ELECTRICAL_UNDERVOLTAGE_ID, ERROR_ELECTRICAL_UNDERVOLTAGE_CODE);
  sysErrors[ERROR_CAN_ECU01_ID] = Error("CAN_ECU01_FAILURE", "ECU01 failure", ERROR_CAN_ECU01_ID, ERROR_CAN_ECU01_CODE);
  sysErrors[ERROR_CAN_ECU02_ID] = Error("CAN_ECU02_FAILURE", "ECU02 failure", ERROR_CAN_ECU02_ID, ERROR_CAN_ECU02_CODE);
  sysErrors[ERROR_CAN_ECU03_ID] = Error("CAN_ECU03_FAILURE", "ECU03 failure", ERROR_CAN_ECU03_ID, ERROR_CAN_ECU03_CODE);
  sysErrors[ERROR_CAN_ECU04_ID] = Error("CAN_ECU04_FAILURE", "ECU04 failure", ERROR_CAN_ECU04_ID, ERROR_CAN_ECU04_CODE);
  sysErrors[ERROR_CAN_ECU15_ID] = Error("CAN_ECU15_FAILURE", "ECU15 failure", ERROR_CAN_ECU15_ID, ERROR_CAN_ECU15_CODE);
  sysErrors[ERROR_SD_CARD_INIT] = Error("SD_CARD_FAILURE", "SD Card Init. failure", ERROR_SD_CARD_INIT, ERROR_SD_CARD_INIT);
  sysErrors[ERROR_SD_CARD_FULL] = Error("SD_CARD_FULL", "SD Card write error", ERROR_SD_CARD_FULL, ERROR_SD_CARD_FULL);
  sysErrors[ERROR_ID_PACKET] = Error("ERROR_ID_PACKET", "CAN ID error", ERROR_ID_PACKET, ERROR_ID_PACKET);

  tmrGPSTimeoutEnabled = true;
  tmrErrorMsgsEnabled = true;
  tmrCanCheckErrorEnabled = true;
}

void setupADC()
{
  pinMode(V_BAT, INPUT);
}

void setupBlink()
{
  tmrBlinkEnabled = true;
}

void setupSensors()
{
  sensors[SENSOR_SPEED_ID] = Sensor("Speed", "Km/h", SENSOR_SPEED_ID, EK304CAN_ID_ADDRESS_ECU04, EK304CAN_SPEED_FREQUENCY);
  sensors[SENSOR_ROTATION_ID] = Sensor("Mot. Rotation", "rpm", SENSOR_ROTATION_ID, EK304CAN_ID_ADDRESS_ECU02, EK304CAN_RPM_FREQUENCY);
  sensors[SENSOR_MOTOR_TEMPERATURE_ID] = Sensor("Mot. Temperature", "oC", SENSOR_MOTOR_TEMPERATURE_ID, EK304CAN_ID_ADDRESS_ECU03, EK304CAN_TEMPERATURE_FREQUENCY);
  sensors[SENSOR_GEAR_ID] = Sensor("Gear", "", SENSOR_GEAR_ID, EK304CAN_ID_ADDRESS_ECU03, EK304CAN_GEAR_POSITION_FREQUENCY);
  sensors[SENSOR_GPS_LATITUDE_ID] = Sensor("Latitude", "o", SENSOR_GPS_LATITUDE_ID, EK304CAN_ID_ADDRESS_GTW, EK304CAN_GPS_FREQUENCY);
  sensors[SENSOR_GPS_LONGITUDE_ID] = Sensor("Longitude", "o", SENSOR_GPS_LONGITUDE_ID, EK304CAN_ID_ADDRESS_GTW, EK304CAN_GPS_FREQUENCY);
  sensors[SENSOR_GPS_ALTITUDE_ID] = Sensor("Altitude", "o", SENSOR_GPS_ALTITUDE_ID, EK304CAN_ID_ADDRESS_GTW, EK304CAN_GPS_FREQUENCY);
  sensors[SENSOR_GPS_FIX_ID] = Sensor("GPS Fix", "o", SENSOR_GPS_FIX_ID, EK304CAN_ID_ADDRESS_GTW, EK304CAN_GPS_FREQUENCY);
  sensors[SENSOR_GPS_HDOP_ID] = Sensor("GPS Fix", "", SENSOR_GPS_HDOP_ID, EK304CAN_ID_ADDRESS_GTW, EK304CAN_GPS_FREQUENCY);
  sensors[SENSOR_VOLTAGE_BATTERY_ID] = Sensor("Main Supply", "V", SENSOR_VOLTAGE_BATTERY_ID, EK304CAN_ID_ADDRESS_GTW, EK304CAN_VOLTAGE_FREQUENCY);
  sensors[SENSOR_MAP_ID] = Sensor("MAP", "kPa", SENSOR_MAP_ID, EK304CAN_ID_ADDRESS_ECU02, EK304CAN_MAP_FREQUENCY);
  sensors[SENSOR_AIR_INTAKE_TEMPERATURE_ID] = Sensor("Air Temp.", "°C", SENSOR_AIR_INTAKE_TEMPERATURE_ID, EK304CAN_ID_ADDRESS_ECU02, EK304CAN_AIR_TEMP_FREQUENCY);
  sensors[SENSOR_WATER_TEMPERATURE_ID] = Sensor("Water Temp.", "°C", SENSOR_WATER_TEMPERATURE_ID, EK304CAN_ID_ADDRESS_ECU02, EK304CAN_WATER_TEMP_FREQUENCY);
  sensors[SENSOR_TPS_ID] = Sensor("TPS", "%", SENSOR_TPS_ID, EK304CAN_ID_ADDRESS_ECU02, EK304CAN_TPS_FREQUENCY);
  sensors[SENSOR_LAMBDA_ID] = Sensor("Lambda", "%", SENSOR_LAMBDA_ID, EK304CAN_ID_ADDRESS_ECU02, EK304CAN_LAMBDA_FREQUENCY);
  sensors[SENSOR_OIL_PRESSURE_ID] = Sensor("Oil Pressure", "kPa", SENSOR_OIL_PRESSURE_ID, EK304CAN_ID_ADDRESS_ECU04, EK304CAN_OIL_PRESSURE_FREQUENCY);
  sensors[SENSOR_OIL_TEMPERATURE_ID] = Sensor("Oil Temp.", "°C", SENSOR_OIL_TEMPERATURE_ID, EK304CAN_ID_ADDRESS_ECU04, EK304CAN_OIL_TEMP_FREQUENCY);
  sensors[SENSOR_REAR_LEFT_SUSPENSION_ID] = Sensor("RL Susp.", "°", SENSOR_REAR_LEFT_SUSPENSION_ID, EK304CAN_ID_ADDRESS_ECU04, EK304CAN_SUSP_REAR_FREQUENCY);
  sensors[SENSOR_REAR_RIGHT_SUSPENSION_ID] = Sensor("RR Susp", "°", SENSOR_REAR_RIGHT_SUSPENSION_ID, EK304CAN_ID_ADDRESS_ECU04, EK304CAN_SUSP_REAR_FREQUENCY);
  sensors[SENSOR_FRONT_LEFT_SUSPENSION_ID] = Sensor("FL Susp", "°", SENSOR_FRONT_LEFT_SUSPENSION_ID, EK304CAN_ID_ADDRESS_ECU01, EK304CAN_SUSP_FRONT_FREQUENCY);
  sensors[SENSOR_FRONT_RIGHT_SUSPENSION_ID] = Sensor("FR Susp", "°", SENSOR_FRONT_RIGHT_SUSPENSION_ID, EK304CAN_ID_ADDRESS_ECU01, EK304CAN_SUSP_FRONT_FREQUENCY);

  //Acelerometros

  //acc[EK304CAN_ID_ACC_01];
}

/**************************************************************************************************************************************/
/* LOOP PRINCIPAL                                                                                                                     */
/**************************************************************************************************************************************/
void loop()
{
  taskDisplay();
  taskGPS();
  taskBlink();
  taskEncoder();
  taskErrorsMonitor();
  taskADC();
  taskCAN();

  taskMain();
  //taskSerial();
}

/**************************************************************************************************************************************/
/* DEFINIÇÃO DAS TAREFAS                                                                                                              */
/**************************************************************************************************************************************/
void taskMain()
{
  //Serial.println(sensors[SENSOR_GPS_HDOP_ID].valor);
  if (stateMachine == STATE_STARTUP_LOGO)
  {
    if (tmrStartupLogoOverflow)
    {
      stateMachine = STATE_STARTUP_NAME;
      tmrStartupLogoOverflow = false;
      tmrStartupLogoEnabled = false;
      tmrStartupNameEnabled = true;

      telaAtual = 2;
      flagUpdateDisplay = true;
    }
  }
  else if (stateMachine == STATE_STARTUP_NAME)
  {
    if (tmrStartupNameOverflow)
    {
      stateMachine = STATE_OVERVIEW;

      tmrStartupNameOverflow = false;
      tmrStartupNameEnabled = false;

      telaAtual = 3;
      flagUpdateDisplay = true;
    }
  }
  else if (stateMachine == STATE_OVERVIEW)
  {
    senOverview[0] = sensors[SENSOR_GPS_LATITUDE_ID];
    senOverview[1] = sensors[SENSOR_GPS_LONGITUDE_ID];

    if (encoder.botao == 1)
    {
      encoder.botao = 0;
      intEncoderButtonEnabled = true;
    }

    if (encoder.giro == 1)
    {
      sensors[SENSOR_GEAR_ID].valor++;
      //sensors[SENSOR_SPEED_ID].valor++;
      if (sensors[SENSOR_GEAR_ID].valor > 6)
        sensors[SENSOR_GEAR_ID].valor = 6;
      encoder.giro = 0;
      tmrSpinDebouncingEnabled = true;
    }

    if (encoder.giro == -1)
    {
      sensors[SENSOR_GEAR_ID].valor--;
      //sensors[SENSOR_SPEED_ID].valor--;
      if (sensors[SENSOR_GEAR_ID].valor < 0)
        sensors[SENSOR_GEAR_ID].valor = 0;
      encoder.giro = 0;
      tmrSpinDebouncingEnabled = true;
    }

    if (tmrSpinDebouncingOverflow)
    {

      tmrSpinDebouncingEnabled = false;
      tmrSpinDebouncingOverflow = false;
      intEncoderClockEnabled = true;
    }
  }
}

void taskScheduler()
{
  if (tmrBlinkEnabled)
  {
    tmrBlinkCount++;
    if (tmrBlinkCount >= TMR_BLINK / TMR_BASE)
    {
      tmrBlinkOverflow = true;
      tmrBlinkCount = 0;
    }
  }

  if (tmrStartupLogoEnabled)
  {
    tmrStartupLogoCount++;
    if (tmrStartupLogoCount >= TMR_STARTUP_LOGO / TMR_BASE)
    {
      tmrStartupLogoOverflow = true;
      tmrStartupLogoCount = 0;
    }
  }

  if (tmrStartupNameEnabled)
  {
    tmrStartupNameCount++;
    if (tmrStartupNameCount >= TMR_STARTUP_NAME / TMR_BASE)
    {
      tmrStartupNameOverflow = true;
      tmrStartupNameCount = 0;
    }
  }

  if (tmrSpinDebouncingEnabled)
  {
    tmrSpinDebouncingCount++;
    if (tmrSpinDebouncingCount >= TMR_DEBOUNCING_ENCODER / TMR_BASE)
    {
      tmrSpinDebouncingOverflow = true;
      tmrSpinDebouncingCount = 0;
    }
  }

  if (tmrGPSTimeoutEnabled)
  {
    tmrGPSTimeoutCount++;
    if (tmrGPSTimeoutCount >= TMR_GPS_TIMEOUT / TMR_BASE)
    {
      tmrGPSTimeoutOverflow = true;
      tmrGPSTimeoutCount = 0;
    }
  }

  if (tmrErrorMsgsEnabled)
  {
    tmrErrorMsgsCount++;
    if (tmrErrorMsgsCount >= TMR_ERRORMSGS / TMR_BASE)
    {
      tmrErrorMsgsOverflow = true;
      tmrErrorMsgsCount = 0;
    }
  }

  if (tmrCanTestEnabled)
  {
    tmrCanTestCount++;
    if (tmrCanTestCount >= TMR_CANTEST / TMR_BASE)
    {
      tmrCanTestOverflow = true;
      tmrCanTestCount = 0;
    }
  }
}

void taskADC()
{
  if (amostrasVBat < SENSOR_VBAT_SAMPLES)
  {
    ValorVBat += (float)(analogRead(V_BAT) * 15.0 / 1023.0);
    amostrasVBat++;
  }
  else
  {
    sensors[SENSOR_VOLTAGE_BATTERY_ID].valor = (float)(ValorVBat / SENSOR_VBAT_SAMPLES);
    ValorVBat = 0.0;
    amostrasVBat = 0;
  }
}

void taskErrorsMonitor()
{
  LinkedList<String> listErrors;

  // verifica se o gps esta conectado
  if (tmrGPSTimeoutOverflow)
  {
    sysErrors[ERROR_GPS_NOTCONNECTED_ID].status = true;
    listErrors.add(sysErrors[ERROR_GPS_NOTCONNECTED_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else // verifica demais erros do GPS
  {
    // verifica se há erro de falta de sinal do GPS
    if ((int)(sensors[SENSOR_GPS_FIX_ID].valor) == 0)
    {
      sysErrors[ERROR_GPS_NOSIGNAL_ID].status = true;
      listErrors.add(sysErrors[ERROR_GPS_NOSIGNAL_ID].ToStringCenter(ERROR_MSGSIZE));
    }
    else
    {
      sysErrors[ERROR_GPS_NOSIGNAL_ID].status = false;

      // verifica se há erro de baixa precisão de GPS
      if (sensors[SENSOR_GPS_HDOP_ID].valor > SENSOR_GPSHDOP_REF)
      {
        sysErrors[ERROR_GPS_LOWSIGNAL_ID].status = true;
        listErrors.add(sysErrors[ERROR_GPS_LOWSIGNAL_ID].ToStringCenter(ERROR_MSGSIZE));
      }
      else
      {
        sysErrors[ERROR_GPS_LOWSIGNAL_ID].status = false;
      }
    }
  }

  // verifica se há sobretensão
  if (sensors[SENSOR_VOLTAGE_BATTERY_ID].valor > SENSOR_VBAT_OVERVOLTAGE)
  {
    sysErrors[ERROR_ELECTRICAL_OVERVOLTAGE_ID].status = true;
    listErrors.add(sysErrors[ERROR_ELECTRICAL_OVERVOLTAGE_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_ELECTRICAL_OVERVOLTAGE_ID].status = false;

  // verifica se há subtensão
  if (sensors[SENSOR_VOLTAGE_BATTERY_ID].valor < SENSOR_VBAT_UNDERVOLTAGE)
  {
    sysErrors[ERROR_ELECTRICAL_UNDERVOLTAGE_ID].status = true;
    listErrors.add(sysErrors[ERROR_ELECTRICAL_UNDERVOLTAGE_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_ELECTRICAL_UNDERVOLTAGE_ID].status = false;

  // verifica se há erro na comunicação com as ECUs
  if (EK304CAN_ECU01_ENABLED && ((millis() - tmpRecebimentoAckECU01) >= TMR_CANCHECKERROR))
  {
    sysErrors[ERROR_CAN_ECU01_ID].status = true;
    listErrors.add(sysErrors[ERROR_CAN_ECU01_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_CAN_ECU01_ID].status = false;

  if (EK304CAN_ECU02_ENABLED && ((millis() - tmpRecebimentoAckECU02) >= TMR_CANCHECKERROR))
  {
    sysErrors[ERROR_CAN_ECU02_ID].status = true;
    listErrors.add(sysErrors[ERROR_CAN_ECU02_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_CAN_ECU02_ID].status = false;

  if (EK304CAN_ECU03_ENABLED && ((millis() - tmpRecebimentoAckECU03) >= TMR_CANCHECKERROR))
  {
    sysErrors[ERROR_CAN_ECU03_ID].status = true;
    listErrors.add(sysErrors[ERROR_CAN_ECU03_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_CAN_ECU03_ID].status = false;

  if (EK304CAN_ECU04_ENABLED && ((millis() - tmpRecebimentoAckECU04) >= TMR_CANCHECKERROR))
  {
    sysErrors[ERROR_CAN_ECU04_ID].status = true;
    listErrors.add(sysErrors[ERROR_CAN_ECU04_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_CAN_ECU04_ID].status = false;

  if (EK304CAN_ECU15_ENABLED && ((millis() - tmpRecebimentoAckECU15) >= TMR_CANCHECKERROR))
  {
    sysErrors[ERROR_CAN_ECU15_ID].status = true;
    listErrors.add(sysErrors[ERROR_CAN_ECU15_ID].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_CAN_ECU15_ID].status = false;

  //Verifica se há erro no cartão SD

  if (flagErrorSDCardInit)
  {
    sysErrors[ERROR_SD_CARD_INIT].status = true;
    listErrors.add(sysErrors[ERROR_SD_CARD_INIT].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_SD_CARD_INIT].status = false;

  if (flagErrorSDCardFull)
  {
    sysErrors[ERROR_SD_CARD_FULL].status = true;
    listErrors.add(sysErrors[ERROR_SD_CARD_FULL].ToStringCenter(ERROR_MSGSIZE));
  }
  else
    sysErrors[ERROR_SD_CARD_FULL].status = false;

  if (flagErrorPacketID)
  {
    sysErrors[ERROR_ID_PACKET].status = true;
    listErrors.add(sysErrors[ERROR_ID_PACKET].ToStringCenter(ERROR_MSGSIZE));
    flagErrorPacketID = false;
  }
  else
    sysErrors[ERROR_ID_PACKET].status = false;

  // algoritmo para mudar mensagem de erro na tela
  cntErrors = listErrors.size();
  sMessages = listErrors;
  if (cntErrors > 0)
  {
    flagErrorsFound = true;
    digitalWrite(LED_ERR, HIGH);
    if (tmrErrorMsgsOverflow)
    {
      sMessages[selectedError].toCharArray(cErrorMsg, ERROR_MSGSIZE);
      selectedError++;
      if (selectedError >= cntErrors)
        selectedError = 0;
      tmrErrorMsgsCount = 0;
      tmrErrorMsgsOverflow = false;
      flagUpdateDisplay = true;
    }
  }
  else
  {
    flagErrorsFound = false;
    digitalWrite(LED_ERR, LOW);
  }
}

void taskDisplay()
{
  if (flagUpdateDisplay)
  {
    if (!flagFreezeDisplay)
    {
      u8gDraw(telaAtual);
    }

    flagUpdateDisplay = false;
  }
}

void taskGPS()
{
  while (Serial3.available())
  {
    char c = Serial3.read();
    sysErrors[ERROR_GPS_NOTCONNECTED_ID].status = false;
    tmrGPSTimeoutOverflow = false;
    tmrGPSTimeoutCount = 0;

    if (!gpsFlagNewData)
    {
      if (c == '$')
      {
        bufSerial = "$";
        gpsFlagNewData = true;
        gpsData = 0;
      }
      else
      {
        bufSerial = "";
        gpsFlagNewData = false;
        flagCheckSum = false;
      }
    }
    else
    {
      bufSerial.concat(c);
      if (bufSerial.length() == 6)
      {
        if (bufSerial == GPS_COMMAND_DATETIME)
        {
          gpsData = GPS_DATA_DATE;
        }
        else if (bufSerial == GPS_COMMAND_POS)
        {
          gpsData = GPS_DATA_COORD;
        }
        else
        {
          bufSerial = "";
          gpsFlagNewData = false;
          flagCheckSum = false;
        }
      }
      else if (c == '*')
      {
        contCheckSum = 0;
        flagCheckSum = true;
      }
      else if (flagCheckSum)
      {
        contCheckSum++;
        if (contCheckSum == 2)
        {
          //Serial.println(bufSerial);
          if (gpsData == GPS_DATA_DATE)
          {
            int field = 0;
            int hh, mm, ss;
            int dd, mo, yy;

            for (unsigned int i = 0; i < bufSerial.length(); i++)
            {
              if (bufSerial[i] == ',')
              {
                field++;
                for (unsigned int j = i + 1, k = 0; bufSerial[j] != ',' && bufSerial[j] != '*'; j++, k++)
                {
                  unsigned int value = bufSerial[j] - 0x30;
                  if (field == 1)
                  {
                    if (k == 0)
                      hh = value * 10;
                    else if (k == 1)
                      hh += value;
                    else if (k == 2)
                      mm = value * 10;
                    else if (k == 3)
                      mm += value;
                    else if (k == 4)
                      ss = value * 10;
                    else if (k == 5)
                      ss += value;
                  }
                  else if (field == 2)
                  {
                    if (k == 0)
                      dd = value * 10;
                    else if (k == 1)
                      dd += value;
                  }
                  else if (field == 3)
                  {
                    if (k == 0)
                      mo = value * 10;
                    else if (k == 1)
                      mo += value;
                  }
                  else if (field == 4)
                  {
                    if (k == 0)
                      yy = value * 1000;
                    else if (k == 1)
                      yy += value * 100;
                    else if (k == 2)
                      yy += value * 10;
                    else if (k == 3)
                      yy += value;
                  }
                  i = j;
                }
              }
            }

            DateTime dt(dd, mo, yy, hh, mm, ss);
            if (dt.IsValid())
              horaAtual = dt;
          }
          else if (gpsData == GPS_DATA_COORD)
          {
            int field = 0, gradLat, gradLong, intHdop = 0, decHdop = 0;
            float minutesLat, minutesLong;
            bool flagDecimalPart = false;

            for (unsigned int i = 0; i < bufSerial.length(); i++)
            {
              if (bufSerial[i] == ',')
              {
                field++;
                for (unsigned int j = i + 1, k = 0; bufSerial[j] != ',' && bufSerial[j] != '*'; j++, k++)
                {
                  int value = bufSerial[j] - 0x30;
                  if (field == 2)
                  {
                    if (k == 0)
                      gradLat = value * 10;
                    else if (k == 1)
                      gradLat += value;
                    if (k == 2)
                      minutesLat = value * 10;
                    else if (k == 3)
                      minutesLat += value;
                    else if (k == 5)
                      minutesLat += value / 10.0;
                    else if (k == 6)
                      minutesLat += value / 100.0;
                    else if (k == 7)
                      minutesLat += value / 1000.0;
                    else if (k == 8)
                      minutesLat += value / 10000.0;
                  }
                  if (field == 3)
                  {
                    if (bufSerial[j] == 'S')
                    {
                      gradLat *= -1;
                      minutesLat *= -1;
                    }
                  }
                  if (field == 4)
                  {
                    if (k == 0)
                      gradLong = value * 100;
                    else if (k == 1)
                      gradLong += value * 10;
                    if (k == 2)
                      gradLong += value;
                    else if (k == 3)
                      minutesLong = value * 10;
                    else if (k == 4)
                      minutesLong += value;
                    else if (k == 6)
                      minutesLong += value / 10.0;
                    else if (k == 7)
                      minutesLong += value / 100.0;
                    else if (k == 8)
                      minutesLong += value / 1000.0;
                    else if (k == 9)
                      minutesLong += value / 10000.0;
                  }
                  if (field == 5)
                  {
                    if (bufSerial[j] == 'W')
                    {
                      gradLong *= -1;
                      minutesLong *= -1;
                    }
                  }
                  if (field == 6)
                  {
                    if (bufSerial[j] >= '0' && bufSerial[j] <= '9')
                    {
                      sensors[SENSOR_GPS_FIX_ID].valor = value;
                    }
                  }
                  if (field == 8)
                  {
                    if (!flagDecimalPart)
                    {
                      if (bufSerial[j] >= '0' && bufSerial[j] <= '9')
                        intHdop = intHdop * 10 + value;
                      else if (bufSerial[j] == '.')
                        flagDecimalPart = true;
                    }
                    else
                    {
                      if (bufSerial[j] >= '0' && bufSerial[j] <= '9')
                      {
                        decHdop = decHdop * 10 + value;
                      }
                    }
                  }
                  i = j;
                }
              }
            }

            if ((int)(sensors[SENSOR_GPS_FIX_ID].valor) > 0)
            {
              sensors[SENSOR_GPS_LATITUDE_ID].valor = (float)(gradLat + minutesLat / 60.0);
              sensors[SENSOR_GPS_LONGITUDE_ID].valor = (float)(gradLong + minutesLong / 60.0);
            }

            sensors[SENSOR_GPS_HDOP_ID].valor = (float)(intHdop + decHdop / 100.0);
          }

          gpsFlagNewData = false;
          flagCheckSum = false;
          flagUpdateDisplay = true;
        }
      }
    }
  }
}

void taskEncoder()
{
  if (flagEncoderClickEvent)
  {
    encoder.botao = true; // muda estado do botao
    while (!digitalRead(ENC_SW))
    {

    }                              // aguarda liberação da tecla
    flagEncoderClickEvent = false; // limpa flag
  }

  // verifica se houve um evento de giro
  else if (flagEncoderSpinEvent)
  {
    // identifica e armazena o sentido de giro
    if (flagEncoderClockwiseSpin)
      encoder.giro = 1;
    else
      encoder.giro = -1;

    // limpa flag
    flagEncoderSpinEvent = false;
  }
}

void taskBlink()
{
  if (tmrBlinkOverflow)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    tmrBlinkOverflow = false;
  }
}

void taskCAN()
{
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK)
  {
    mcp2515.readMessage(&frame);

    switch (frame.can_id)
    {
    case EK304CAN_ID_RPM:
      //RPM
      sensors[SENSOR_ROTATION_ID].valor = frame.data[0] * CONST_RPM;

      setAckTime(sensors[SENSOR_ROTATION_ID].origin);
      break;
    case EK304CAN_ID_ACC_01:
      //Acelerômetro 1
      break;
    case EK304CAN_ID_ACC_02:
      //Acelerômetro 2
      break;
    case EK304CAN_ID_ACC_03:
      //Acelerômetro 3
      break;
    case EK304CAN_ID_LAMBA:
      //Lambda
      sensors[SENSOR_LAMBDA_ID].valor = frame.data[0];

      setAckTime(sensors[SENSOR_LAMBDA_ID].origin);
      break;
    case EK304CAN_ID_GEAR_POSITION:
      //Gear Position
      sensors[SENSOR_GEAR_ID].valor = frame.data[4];

      setAckTime(sensors[SENSOR_GEAR_ID].origin);
      break;
    case EK304CAN_ID_TEMPERATURE:
      //Temperature
      sensors[SENSOR_MOTOR_TEMPERATURE_ID].valor = frame.data[1];

      setAckTime(sensors[SENSOR_MOTOR_TEMPERATURE_ID].origin);
      break;
    case EK304CAN_ID_PRESSURE:
      //Pressure
      break;
    case EK304CAN_ID_SUSP_REAR:
      //SuspRear
      sensors[SENSOR_FRONT_LEFT_SUSPENSION_ID].valor = frame.data[1] / (2.8 + 1 / 30);
      sensors[SENSOR_FRONT_LEFT_SUSPENSION_ID].valor = sensors[SENSOR_FRONT_LEFT_SUSPENSION_ID].valor << 8;
      sensors[SENSOR_FRONT_LEFT_SUSPENSION_ID].valor += frame.data[0] / (2.8 + 1 / 30);
      sensors[SENSOR_FRONT_RIGHT_SUSPENSION_ID].valor = frame.data[3] / (2.8 + 1 / 30);
      sensors[SENSOR_FRONT_RIGHT_SUSPENSION_ID].valor = sensors[SENSOR_FRONT_RIGHT_SUSPENSION_ID].valor << 8;
      sensors[SENSOR_FRONT_RIGHT_SUSPENSION_ID].valor += frame.data[2] / (2.8 + 1 / 30);

      setAckTime(sensors[SENSOR_FRONT_RIGHT_SUSPENSION_ID].origin);
      break;
    case EK304CAN_ID_OIL_PRESSURE:
      //OilPressure
      sensors[SENSOR_OIL_PRESSURE_ID].valor = frame.data[2];

      setAckTime(sensors[SENSOR_OIL_PRESSURE_ID].origin);
      break;
    case EK304CAN_ID_OIL_TEMPERATURE:
      //OilTemp
      sensors[SENSOR_OIL_TEMPERATURE_ID].valor = frame.data[2];

      setAckTime(sensors[SENSOR_OIL_TEMPERATURE_ID].origin);
      break;
    case EK304CAN_ID_SPEED:
      //Speed
      sensors[SENSOR_SPEED_ID].valor = frame.data[0];

      setAckTime(sensors[SENSOR_SPEED_ID].origin);
      break;
    case EK304CAN_ID_AIR_TEMP:
      //AirTemp
      sensors[SENSOR_AIR_INTAKE_TEMPERATURE_ID].valor = frame.data[4];

      setAckTime(sensors[SENSOR_AIR_INTAKE_TEMPERATURE_ID].origin);
      break;
    case EK304CAN_ID_WATER_TEMPERATURE:
      //WaterTemp
      sensors[SENSOR_WATER_TEMPERATURE_ID].valor = frame.data[3];

      setAckTime(sensors[SENSOR_WATER_TEMPERATURE_ID].origin);
      break;
    case EK304CAN_ID_TPS:
      //TPS
      sensors[SENSOR_TPS_ID].valor = frame.data[2];

      setAckTime(sensors[SENSOR_TPS_ID].origin);
      break;
    case EK304CAN_ID_SUSP_FRONT: //SuspFront
      sensors[SENSOR_REAR_LEFT_SUSPENSION_ID].valor = frame.data[1] / (2.8 + 1 / 30);
      sensors[SENSOR_REAR_LEFT_SUSPENSION_ID].valor = sensors[SENSOR_REAR_LEFT_SUSPENSION_ID].valor << 8;
      sensors[SENSOR_REAR_LEFT_SUSPENSION_ID].valor += frame.data[0] / (2.8 + 1 / 30);
      sensors[SENSOR_REAR_RIGHT_SUSPENSION_ID].valor = frame.data[3] / (2.8 + 1 / 30);
      sensors[SENSOR_REAR_RIGHT_SUSPENSION_ID].valor = sensors[SENSOR_REAR_RIGHT_SUSPENSION_ID].valor << 8;
      sensors[SENSOR_REAR_RIGHT_SUSPENSION_ID].valor += frame.data[2] / (2.8 + 1 / 30);

      setAckTime(sensors[SENSOR_REAR_RIGHT_SUSPENSION_ID].origin);

      break;
    case EK304CAN_ID_MAP:
      //MAP
      sensors[SENSOR_MAP_ID].valor = frame.data[0];

      setAckTime(sensors[SENSOR_MAP_ID].origin);
      break;

    default:
      flagErrorPacketID = true;
      break;
    }

    //Aqui começa a parte da gravação no cartão SD

    if (!sdCard.Write(STRING_FILE_NAME, ""))
    {
      flagErrorSDCardFull = true;
    }
    else
    {
      flagErrorSDCardInit = false;
    }

    sdCard.Write(STRING_FILE_NAME, ";");
    sdCard.Write(STRING_FILE_NAME, String(millis())); //Grava o tempo decorrido
    sdCard.Write(STRING_FILE_NAME, ";");

    sdCard.Write(STRING_FILE_NAME, String(frame.can_id)); //Grava os dados da CAN
    sdCard.Write(STRING_FILE_NAME, ";");
    sdCard.Write(STRING_FILE_NAME, String(frame.can_dlc));
    sdCard.Write(STRING_FILE_NAME, ";");
    for (int i = 0; i < frame.can_dlc; i++)
    {
      sdCard.Write(STRING_FILE_NAME, String(frame.data[i]));
      sdCard.Write(STRING_FILE_NAME, ";");
    }
    sdCard.Write(STRING_FILE_NAME, ";\r\n"); //Quebra a linha

    Serial.print(frame.can_id, HEX);
    Serial.print(frame.can_dlc, HEX);
    Serial.print(" ");
    for (int i = 0; i < frame.can_dlc; i++)
    {
      Serial.print(frame.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println(";");
    /*    // Envia os pacotes de ACK
    if (tmrCanTestOverflow)
    {
      if (EK304CAN_ECU01_ENABLED)
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_ECU01);
      if (EK304CAN_ECU02_ENABLED)
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_ECU02);
      if (EK304CAN_ECU03_ENABLED)
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_ECU03);
      if (EK304CAN_ECU04_ENABLED)
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_ECU04);
      if (EK304CAN_ECU15_ENABLED)
        CAN_SendACK(&mcp2515, EK304CAN_ID_ADDRESS_ECU15);

      tmrCanTestOverflow = false;
    }  
    */
  }
  else if (mcp2515.readMessage(&frame) == MCP2515::ERROR_FAIL)
  {
    Serial.println("CAN ERROR");
  }
}

void taskSerial()
{
  while (Serial3.available())
  {
    char c = Serial3.read();
    Serial.print(c);
  }

  while (Serial.available())
  {
    char c = Serial.read();
    Serial3.print(c);
  }
}

/**************************************************************************************************************************************/
/* FUNÇÕES DE CONFIGURAÇÕES                                                                                                           */
/**************************************************************************************************************************************/
// configura a serial
void setupSerial()
{
  Serial.begin(9600);  // configura baud rate da serial0 para 115200bps
  Serial3.begin(9600); // configura baud rate da serial3 para 115200bps
}

// configura o modo do display a ser utilizado
void setupDisplay()
{
  // configura de acordo com o modelo de display
  if (u8g.getMode() == U8G_MODE_R3G3B2)
    u8g.setColorIndex(255);
  else if (u8g.getMode() == U8G_MODE_GRAY2BIT)
    u8g.setColorIndex(1);
  else if (u8g.getMode() == U8G_MODE_BW)
    u8g.setColorIndex(1);

  // tela inicial: logo equipe
  u8gDraw(1);
}

// configura o modulo transciever CAN
void setupCAN()
{
  SPI.begin();
  CAN_Init(&mcp2515, CAN_1000KBPS);
  tmrCanTestEnabled = false;
}

// configura módulo encoder
void setupEncoder()
{
  // pinos confugrados com pullup interno
  pinMode(ENC_CL, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  // configura interrupções para os pinos de clock e data
  attachInterrupt(digitalPinToInterrupt(ENC_CL), isrEncoderSpin, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_SW), isrEncoderClick, FALLING);

  intEncoderClockEnabled = true;
}

void setupTimer()
{
  Timer1.initialize(TMR_BASE);
  Timer1.attachInterrupt(taskScheduler);
}

void setupInit()
{
  pinMode(LED_BUILTIN, OUTPUT);
  stateMachine = STATE_STARTUP_LOGO;

  tmrStartupLogoEnabled = true;
  telaAtual = 1;
  flagUpdateDisplay = true;
}

void setupSDModule()
{
  if (!sdCard.Init(SPI_CS_SD))
  {
    Serial.println("Erro ao iniciar");
    flagErrorSDCardInit = true;
  }
  if (!sdCard.Write("xx.txt", "asoksaosak;"))
  {
    Serial.println("Erro ao escrever");
    flagErrorSDCardInit = true;
  }
  if (!sdCard.Read("xx.txt"))
  {
    Serial.println("Erro ao ler");
    flagErrorSDCardInit = true;
  }
  if (!sdCard.Remove("xx.txt"))
  {
    Serial.println("Erro ao apagar");
    flagErrorSDCardInit = true;
  }
}

void sendSettingsSetup()
{
  //Manda as taxas de atualização via CAN

  can_frame setting_ecu01;
  can_frame setting_ecu02;
  can_frame setting_ecu03;
  can_frame setting_ecu04;
  can_frame setting_ecu15;

  //Configuração dos pacotes a serem enviados

  setting_ecu01.can_id = EK304CAN_ID_SETTING_ECU01;
  setting_ecu01.can_dlc = EK304CAN_SETTING_DLC_ECU01;

  setting_ecu02.can_id = EK304CAN_ID_SETTING_ECU02;
  setting_ecu02.can_dlc = EK304CAN_SETTING_DLC_ECU02;

  setting_ecu03.can_id = EK304CAN_ID_SETTING_ECU03;
  setting_ecu03.can_dlc = EK304CAN_SETTING_DLC_ECU03;

  setting_ecu04.can_id = EK304CAN_ID_SETTING_ECU04;
  setting_ecu04.can_dlc = EK304CAN_SETTING_DLC_ECU04;

  setting_ecu15.can_id = EK304CAN_ID_SETTING_ECU15;
  setting_ecu15.can_dlc = EK304CAN_SETTING_DLC_ECU15;

  //Configurar conteúdo dos pacotes
}

/**************************************************************************************************************************************/
/* FUNÇÕES DE DESENHO DAS TELAS                                                                                                       */
/**************************************************************************************************************************************/
void u8gPrepare()
{
  u8g.setFont(u8g_font_6x10);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
}

void u8gDraw(int screen)
{
  u8gPrepare();
  u8g.firstPage();
  do
  {
    switch (screen)
    {
    case 1:
      scrLogo();
      break;
    case 2:
      scrWelcome();
      break;
    case 3:
      scrOverview();
      break;
    }
  } while (u8g.nextPage());
}

void scrLogo()
{
  u8g.drawXBMP(0, 0, 128, 64, logo_bits);
  u8g.setFont(u8g_font_u8glib_4);
  u8g.drawStr(0, 63, FIRMWARE_VERSION);
}

void scrWelcome()
{
  u8g.setFont(u8g_font_unifont);
  u8g.drawStr(10, 35, STRING_DISPLAY_TEAMNAME);
  u8g.drawStr(10, 36, STRING_DISPLAY_TEAMNAME);
  u8g.setFont(u8g_font_6x10);
  u8g.drawStr(45, 45, STRING_DISPLAY_CARNAME);
  u8g.setFont(u8g_font_u8glib_4);
  u8g.drawStr(0, 63, FIRMWARE_VERSION);
}

void scrOverview()
{
  String sTemp = "";
  char cTemp[STRING_DISPLAY_DATETIME_MAX + 1];

  // mostra data e hora atuais
  u8g.setFont(u8g_font_u8glib_4);
  horaAtual.ToStringCenter(STRING_DISPLAY_DATETIME_MAX).toCharArray(cTemp, STRING_DISPLAY_DATETIME_MAX + 1);
  u8g.drawStr(0, 5, cTemp);

  // mostra as informações dos sensores selecionadas
  sTemp = "";
  sTemp.concat(senOverview[0].nome);
  sTemp.concat(": ");
  sTemp.concat(senOverview[0].valor);
  sTemp.concat(" ");
  sTemp.concat(senOverview[0].unidade);
  sTemp.toCharArray(cTemp, 20);
  u8g.drawStr(0, 55, cTemp);

  sTemp = "";
  sTemp.concat(senOverview[1].nome);
  sTemp.concat(": ");
  sTemp.concat(senOverview[1].valor);
  sTemp.concat(" ");
  sTemp.concat(senOverview[1].unidade);
  sTemp.toCharArray(cTemp, 20);
  u8g.drawStr(0, 63, cTemp);

  // mostra a velocidade
  u8g.setFont(u8g_font_fub20);

  sTemp = "";
  sTemp.concat((int)(sensors[SENSOR_SPEED_ID].valor));
  sTemp.toCharArray(cTemp, 4);
  if (sensors[SENSOR_SPEED_ID].valor >= 100)
    u8g.drawStr(30, 33, cTemp);
  else if (sensors[SENSOR_SPEED_ID].valor >= 10)
    u8g.drawStr(37, 33, cTemp);
  else
    u8g.drawStr(45, 33, cTemp);

  u8g.setFont(u8g_font_5x7);

  sTemp = "";
  sTemp.concat(sensors[SENSOR_SPEED_ID].unidade);
  sTemp.toCharArray(cTemp, 20);
  u8g.drawStr(80, 33, cTemp);

  // mostra a marcha
  u8g.setFont(u8g_font_5x7);

  if (sensors[SENSOR_GEAR_ID].valor >= SENSOR_GEAR_MIN && sensors[SENSOR_GEAR_ID].valor <= SENSOR_GEAR_MAX)
  {
    sTemp = sensors[SENSOR_GEAR_ID].nome;
    sTemp += ": ";
    if ((int)(sensors[SENSOR_GEAR_ID].valor) == 0)
      sTemp += "N";
    else if ((int)(sensors[SENSOR_GEAR_ID].valor) == 1)
      sTemp += "1st";
    else if ((int)(sensors[SENSOR_GEAR_ID].valor) == 2)
      sTemp += "2nd";
    else if ((int)(sensors[SENSOR_GEAR_ID].valor) == 3)
      sTemp += "3rd";
    else
    {
      sTemp += (int)(sensors[SENSOR_GEAR_ID].valor);
      sTemp += "th";
    }

    sTemp.toCharArray(cTemp, 20);
    u8g.drawStr(80, 23, cTemp);

    // mostra erros na tela
    if (flagErrorsFound)
    {

      u8g.drawBox(0, 37, 128, 9);
      u8g.setColorIndex(0);
      u8g.drawStr(2, 44, cErrorMsg);
      u8g.setColorIndex(1);
    }
  }
}

/**************************************************************************************************************************************/
/* FUNÇÕES DE TRATAMENTO DE INTERRUPÇÃO                                                                                               */
/**************************************************************************************************************************************/
void isrEncoderSpin()
{
  if (intEncoderClockEnabled)
  {
    intEncoderClockEnabled = false;
    if (digitalRead(ENC_CL))
      flagEncoderClockwiseSpin = !digitalRead(ENC_DT);
    else
      flagEncoderClockwiseSpin = digitalRead(ENC_DT);

    flagEncoderSpinEvent = true;
  }
}

void isrEncoderClick()
{
  if (intEncoderButtonEnabled)
  {
    intEncoderButtonEnabled = false;
    flagEncoderClickEvent = true;
  }
}

/**************************************************************************************************************************************/
/* FUNÇÕES GERAIS                                                                                                                     */
/**************************************************************************************************************************************/

void setAckTime(int origin)
{
  switch (origin)
  {
  case EK304CAN_ID_ADDRESS_ECU01:
    tmpRecebimentoAckECU01 = millis();
    break;
  case EK304CAN_ID_ADDRESS_ECU02:
    tmpRecebimentoAckECU02 = millis();
    break;
  case EK304CAN_ID_ADDRESS_ECU03:
    tmpRecebimentoAckECU03 = millis();
    break;
  case EK304CAN_ID_ADDRESS_ECU04:
    tmpRecebimentoAckECU04 = millis();
    break;
  case EK304CAN_ID_ADDRESS_ECU15:
    tmpRecebimentoAckECU15 = millis();
    break;

  default:
    break;
  }
}