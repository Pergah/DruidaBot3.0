
// Proyecto: Druida BOT de DataDruida
// Autor: Bryan Murphy
// Año: 2025
// Licencia: MIT
// VERSION ADAPTADA NUEVA WEBAPP

#include "esp_system.h"
#include <Wire.h>
#include <RTClib.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <UniversalTelegramBot.h>
#include <EEPROM.h>
#include <Time.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ESP32Servo.h>
#include "math.h"
#include <esp_wifi.h>
#include <HTTPUpdate.h>
#include <FFat.h>
#include "QRCodeGenerator.h"

#define I2C_SDA_MAIN 42
#define I2C_SCL_MAIN 41

#define TCA9554_ADDRESS 0x20
#define TCA9554_OUTPUT_REG 0x01
#define TCA9554_POL_REG    0x02
#define TCA9554_CONFIG_REG 0x03

#define RELAY1 1
#define RELAY2 2
#define RELAY3 3
#define RELAY4 4
#define RELAY5 5
#define RELAY6 6
#define RELAY7 7
#define RELAY8 8

#define RELAY_ON_LEVEL  1
#define RELAY_OFF_LEVEL 0

#define PCF85063_ADDRESS  0x51

#define RTC_CTRL_1_ADDR   0x00
#define RTC_CTRL_2_ADDR   0x04
#define RTC_SECOND_ADDR  0x04


// Bits CTRL1
#define RTC_CTRL_1_STOP    0x20
#define RTC_CTRL_1_SR      0x10
#define RTC_CTRL_1_CAP_SEL 0x01  // 0=7pF, 1=12.5pF


#define H 1
#define T 2
#define D 3
#define HT 4
#define HS 5
#define MANUAL 1
#define AUTO 2
#define CONFIG 3
#define SUPERCICLO 4
#define STATUS 5
#define TIMER 6
#define RIEGO 7
#define AUTORIEGO 8
#define AUTOINT 9
#define SUPERCICLO1313 13


// Aca se muestra como van conectados los componentes

#define SERVO 23

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define WDT_TIMEOUT 300 //(Diez minutos)

#define SDA_NANO 33
#define SCL_NANO 27

#define RELAY4_ACTIVE_LOW 1

#define ADDR_VEGE_START    384  // uint32_t
#define ADDR_FLORA_START   388  // uint32_t
#define ADDR_VEGE_DAYS     392  // int32_t
#define ADDR_FLORA_DAYS    396  // int32_t
#define ADDR_LAST_DATEKEY  400  // uint32_t (yyyymmdd local)
#define ADDR_VEGE_ACTIVE   404  // uint8_t
#define ADDR_FLORA_ACTIVE  405  // uint8_t
// ===== Direcciones EEPROM nuevas para riego R3 =====
#define ADDR_RIEGOS_HECHOS      464   // int32: cantidad de riegos ya hechos
#define ADDR_ULTIMO_DIA_RIEGO   468   // int32: día lógico 0–6, o -1 si ninguno
#define EEPROM_ADDR_HORA_ACTUAL   492  // uint8_t
#define EEPROM_ADDR_MINUTO_ACTUAL 496  // uint8_t
#define EEPROM_ADDR_MINUTES_UP     497  // uint16_t (2 bytes: 497-498)
#define EEPROM_ADDR_MAGIC         499
#define EEPROM_MAGIC_VALUE         0xA5

// ===== Direcciones EEPROM para R7 (iluminacion clon R4) =====
#define EEPROM_R7_MODO             552  // uint8_t
#define EEPROM_R7_TIME_ON          553  // uint16_t
#define EEPROM_R7_TIME_OFF         555  // uint16_t
#define EEPROM_R7_ESTADO           557  // uint8_t
#define EEPROM_R7_HORA_ON          558  // int32_t
#define EEPROM_R7_MIN_ON           562  // int32_t
#define EEPROM_R7_HORA_OFF         566  // int32_t
#define EEPROM_R7_MIN_OFF          570  // int32_t
#define EEPROM_R7_HORA_AMANECER    574  // int32_t
#define EEPROM_R7_HORA_ATARDECER   578  // int32_t
#define EEPROM_R7_HORAS_LUZ        582  // int32_t
#define EEPROM_R7_HORAS_OSCURIDAD  586  // int32_t
#define EEPROM_R7_SUPER_START      590  // int32_t
#define EEPROM_R7_NAME             594  // int32_t

// ===== Direcciones EEPROM para R8 (irrigacion clon R3) =====
#define EEPROM_R8_MODO             600  // uint8_t
#define EEPROM_R8_TIME_ON          601  // uint16_t
#define EEPROM_R8_TIME_OFF         603  // uint16_t
#define EEPROM_R8_DIAS             605  // uint8_t[7]
#define EEPROM_R8_ESTADO           612  // uint8_t
#define EEPROM_R8_HORA_ON          613  // int32_t
#define EEPROM_R8_MIN_ON           617  // int32_t
#define EEPROM_R8_HORA_OFF         621  // int32_t
#define EEPROM_R8_MIN_OFF          625  // int32_t
#define EEPROM_R8_MIN              629  // int32_t
#define EEPROM_R8_MAX              633  // int32_t
#define EEPROM_R8_TIEMPO_RIEGO     637  // int32_t
#define EEPROM_R8_TIEMPO_NO_RIEGO  641  // int32_t
#define EEPROM_R8_CANTIDAD         645  // int32_t
#define EEPROM_R8_UNIDAD_RIEGO     649  // int32_t
#define EEPROM_R8_UNIDAD_NO_RIEGO  653  // int32_t
#define EEPROM_R8_RIEGOS_HECHOS    657  // int32_t
#define EEPROM_R8_ULTIMO_DIA       661  // int32_t
#define EEPROM_R8_NAME             665  // int32_t

// ===== Direcciones EEPROM para asignacion de sensores a relays =====
#define EEPROM_SENSOR_R1            700  // int32_t: ambiente 1..4 combinable, ej 123
#define EEPROM_SENSOR_R2            704  // int32_t: ambiente 1..4 combinable
#define EEPROM_SENSOR_R3            708  // int32_t: suelo 5..6 combinable, ej 56
#define EEPROM_SENSOR_R5            712  // int32_t: ambiente 1..4 combinable
#define EEPROM_SENSOR_R6            716  // int32_t: ambiente 1..4 combinable
#define EEPROM_SENSOR_R8            720  // int32_t: suelo 5..6 combinable
#define EEPROM_SENSOR_AIR_ACTIVE    724  // uint8_t[4]
#define EEPROM_SENSOR_SOIL_ACTIVE   728  // uint8_t[2]

// ===== Versión de firmware =====
#define FW_VERSION "2.0"   // cambiala en cada release

// ===== URL del firmware OTA en GitHub =====
#define OTA_FIRMWARE_URL "https://raw.githubusercontent.com/Pergah/DruidaBot3.0/main/backend.ino.bin"
#define OTA_FFAT_URL "https://raw.githubusercontent.com/Pergah/DruidaBot3.0/main/frontend.bin"

#define ARR_LEN(a) (sizeof(a) / sizeof((a)[0]))

#define SOIL_TIMEOUT 5000
#define MODBUS_ID_SOIL 1   // Sensor suelo: temp, hum, EC

#define MODBUS_ID_AIR  2   // Sensor ambiente: temp, hum


#include <QRCodeGenerator.h>


TwoWire I2CNano = TwoWire(1);  // Usamos el bus I2C número 1


Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // -1 = sin pin RESET
Servo dimmerServo; // Objeto del servomotor


const String botToken = "8225459011:AAGrItaTDxdu6g8CSrF5u5Q1uVf-fq6af1c"; //DRUIDA 26
const char* ssid_AP = "DruidaBot"; // 
const char* password_AP = "12345678";          // Contraseña de la red AP
// ID: 1308350088 
String scriptId = "*******************";  //Druida 26


const unsigned long BOT_MTBS = 1000;
const int MAX_STRING_LENGTH = 32;
unsigned long bot_lasttime;
const unsigned long wifiCheckInterval = 120000;  //WiFi CheckStatus cada 2 minutos
unsigned long previousMillis = 0;

//bool irCaptureDone = false;


WiFiClientSecure secured_client;
UniversalTelegramBot bot(botToken, secured_client);


//Adafruit_AHTX0 aht;


RTC_DS3231 rtc;

String chat_id = "";
String ssid = "";
String password = "";


int conPW = 1;
int reset = 0;

byte modoR1 = 0;
float minR1 = 0;
float maxR1 = 0;
byte paramR1 = 0;
int timeOnR1 = 0;
int timeOffR1 = 0;
int horaOnR1 = 0;
int minOnR1 = 0;
int horaOffR1 = 0;
int minOffR1 = 0;

//byte modoR5 = 0;
float minR5 = 0;
float maxR5 = 0;
byte paramR5 = 0;
int timeOnR5 = 0;
int timeOffR5 = 0;
int horaOnR5 = 0;
int minOnR5 = 0;
int horaOffR5 = 0;
int minOffR5 = 0;

byte modoR2 = 0;
float minR2 = 0;
float maxR2 = 0;
float minTR2 = 0;
float maxTR2 = 0;
byte paramR2 = 0;


byte modoR3 = 0;
int minR3 = 0;
int maxR3 = 0;
byte paramR3 = HS;
byte diasRiego[7];
int timeOnR3 = 0;
int timeOffR3 = 0;

byte modoR4 = 0;
int timeOnR4 = 0;
int timeOffR4 = 0;

byte modoR7 = 0;
int timeOnR7 = 0;
int timeOffR7 = 0;

byte modoR8 = 0;
int minR8 = 0;
int maxR8 = 0;
byte paramR8 = HS;
byte diasRiegoR8[7];
int timeOnR8 = 0;
int timeOffR8 = 0;

byte modoMenu = -1;

int R1config = -1;
int R5config = -1;
int R2config = -1;
//int R2irconfig = -1;
int R3config = -1;
int R4config = -1;
int R7config = -1;
int R8config = -1;

byte estadoR1 = 0;
//byte estadoR5 = 0;
byte estadoR2 = 0;
//byte estadoR2ir = 0;
byte estadoR3 = 0;
byte estadoR4 = 0;
byte estadoR7 = 0;
byte estadoR8 = 0;

bool R1estado = HIGH;
bool R2estado = HIGH;
//bool R2irestado = HIGH;
bool R3estado = HIGH;
bool R4estado = HIGH;
bool R7estado = HIGH;
bool R8estado = HIGH;
//bool R5estado = HIGH;

float DPV = 0;

int diaNumero;
int diaHoy;
int cantCon = 0;

int horaOnR3, minOnR3, horaOffR3, minOffR3, horaOnR4, minOnR4, horaOffR4, minOffR4;
int horaOnR7, minOnR7, horaOffR7, minOffR7, horaOnR8, minOnR8, horaOffR8, minOffR8;
int horaWifi = 0;

float humedad;  // Variables globales
float temperature;

float maxHum = -999;
float minHum = 999;
float maxTemp = -999;
float minTemp = 999;
float maxVPD = -99;
float minVPD = 99;

int lastHourSent = -1;

byte estadoRTC = 0;

int tiempoR1 = 0;
int tiempoR5 = 0;
int tiempoR2 = 0;
//int tiempoR2ir = 0;
int tiempoR3 = 0;
int tiempoR4 = 0; 
int tiempoR7 = 0;
int tiempoR8 = 0;
bool esperandoTiempoR1 = false;
bool esperandoTiempoR5 = false;
bool esperandoTiempoR2 = false;
bool esperandoTiempoR2ir = false;
bool esperandoTiempoR3 = false;
bool esperandoTiempoR4 = false;
bool esperandoTiempoR7 = false;
bool esperandoTiempoR8 = false;


WebServer server(80);


int parametroActual = 0;  // Variable global para controlar qué parámetro mostrar
unsigned long lastUpdate = 0;  // Para manejar el tiempo entre actualizaciones
const unsigned long displayInterval = 2000;  // Intervalo de cambio (2 segundos)


// ===== RIEGO R3 (AUTO) - VARIABLES GLOBALES =====
unsigned long previousMillisRiego = 0;
bool enRiego = false;
bool ventanaActivaPrev = false;
int riegosHechos = 0;        // cuántos riegos ya se hicieron en el día lógico de riego

int tiempoRiego = -1;        // Tiempo de riego en segundos
int tiempoNoRiego = -1;      // Tiempo de pausa entre riegos en segundos
int cantidadRiegos = 1;      // Número de ciclos de riego

// Día lógico al que pertenece este ciclo de riego (0–6), -1 si ninguno
int ultimoDiaRiego = -1;


int horaAmanecer = -1; // Hora de amanecer en minutos (04:00 -> 240 minutos)
int horaAtardecer = -1;
int currentPosition = 0; // Posición inicial del servo

int previousSecondRiego = 0; // Inicialización con 0
int previousSeconds = 0; 
int horaAmanecerR7 = -1;
int horaAtardecerR7 = -1;

// ===== RIEGO R8 (AUTO) - VARIABLES GLOBALES =====
unsigned long previousMillisRiegoR8 = 0;
bool enRiegoR8 = false;
bool ventanaActivaPrevR8 = false;
int riegosHechosR8 = 0;
int tiempoRiegoR8 = -1;
int tiempoNoRiegoR8 = -1;
int cantidadRiegosR8 = 1;
int ultimoDiaRiegoR8 = -1;

String relayNames[] = {
  "Humidificacion",
  "Extraccion",
  "Irrigacion",
  "Iluminacion",
  "A/C",
  "Calefaccion",
  "Deshumidificacion",
  "Intraccion",
  "Ventilacion"
};

String relayAssignedNames[8] = {
  "R1",
  "R5",
  "R2",
  "R3",
  "R4",
  "R6",
  "R7",
  "R8"
}; // Hasta 8 relays

int R1name = 0;   // (Humidificacion)
int R5name = 0;   // (Humidificacion)
int R2name = 1;   // (Extraccion)
int R3name = 2;   // (Irrigacion)
int R4name = 3;   // (Iluminacion)
int R6name = 0;   // (Humidificacion)
int R7name = 3;   // (Iluminacion)
int R8name = 2;   // (Irrigacion)
//int R2irname = 4; // (Aire acondicionado)


int modoWiFi = 0;

//SUPERCICLO

unsigned int proximoCambioR4 = 60; // Hora del primer cambio, en minutos (ej. 01:00)
bool luzEncendida = false;

unsigned long previousMillisWD = 0;
const unsigned long interval = 20000; // 20 segundos

// Nuevas variables a añadir SUPERCICLO
int horasLuz = -1;             // Ahora en minutos
int horasOscuridad = -1;       // Ahora en minutos
unsigned long proximoEncendidoR4; // Próxima hora de encendido (minutos desde medianoche)
unsigned long proximoApagadoR4;   // Próxima hora de apagado (minutos desde medianoche)

int horasLuzR7 = -1;
int horasOscuridadR7 = -1;
unsigned long proximoEncendidoR7;
unsigned long proximoApagadoR7;

//int intervaloDatos = 60;  // Intervalo en minutos (por defecto 1 hora)
unsigned long previousMillisTelegram = 0;
unsigned long previousMillisGoogle = 0;

// Variables para el modo AUTORIEGO del relay 1
int tiempoEncendidoR1 = 5; // en minutos
int tiempoApagadoR1 = 10;  // en minutos
unsigned long previousMillisR1 = 0;
// Variables para el modo AUTORIEGO del relay 1
int tiempoEncendidoR5 = 5; // en minutos
int tiempoApagadoR5 = 10;  // en minutos
unsigned long previousMillisR5 = 0;
bool enHumidificacion = false;

int tiempoGoogle = 240;

int tiempoTelegram = 120; // En minutos, configurable desde la web

int unidadRiego = 60;     // 1 = seg, 60 = min, 3600 = h
int unidadNoRiego = 3600; // valores cargados desde EEPROM o lo que uses
int unidadRiegoR8 = 60;
int unidadNoRiegoR8 = 3600;

byte direccionR1 = 0;
byte direccionR5 = 0;

unsigned long tiempoInicioR2 = 0;
unsigned long tiempoEsperaR2 = 0;
bool enEsperaR2 = false;
float humedadReferenciaR2 = 0;
float temperaturaReferenciaR2 = 0;
float dpvReferenciaR2 = 0;

/***** Ajustes generales para el modo AUTO de R2 *****/
const unsigned long R2_WAIT_MS = 10UL * 60UL * 1000UL;   // 10 min
const float HUM_MIN_VALID      =   0.0;
const float HUM_MAX_VALID      =  99.9;
const float TMP_MIN_VALID      = -10.0;
const float TMP_MAX_VALID      =  50.0;

// ---------- Tiempos absolutos ----------
unsigned long tiempoProxEncendido = 0;  // En minutos desde epoch Unix
unsigned long tiempoProxApagado = 0;


static bool apMode = false;
static unsigned long lastRetryTime = 0;

int sensor1Value, sensor2Value, sensor3Value;
float sensorPH = 0.0;

bool sensorDataValid = false;

static bool superR4_Inicializado = false;

// ====== Polaridad de R5 ======
//const bool R5_ACTIVO_EN_HIGH = true;  // R5 cierra con HIGH

// ====== Estado lógico y físico de R5 ======
// Lógico (se persiste): 0 = OFF, 1 = ON
uint8_t estadoR5 = 0;
// Modo (se persiste): 0 = AUTO, 1 = MANUAL (ajustá si tu enum difiere)
uint8_t modoR5   = 1; // MANUAL por defecto si no lo tenías
// Físico (cache del último nivel escrito al pin)
uint8_t R5estado = LOW;


static const size_t SSID_CAP   = 50; // ocupa [37..86]
static const size_t PASS_CAP   = 50; // ocupa [87..136]
static const size_t CHATID_CAP = 25; // ocupa [215..239]

// Zona segura post-boot para persistencia (anti-brownout/escrituras con datos vacíos)
bool canPersist = false;
unsigned long bootMs = 0;

// Validador de horarios H:M
inline bool horarioOK(int h, int m) { return (h >= 0 && h < 24 && m >= 0 && m < 60); }

// ===== SUPERCICLO R4 =====
// Variables de scheduling SIEMPRE en minutos [0..1439]
extern int16_t nextOnR4Abs  = -1;
extern int16_t nextOffR4Abs = -1;
extern int16_t nextOnR7Abs  = -1;
extern int16_t nextOffR7Abs = -1;

// ===== Ciclos de cultivo (persistentes con Guardado_General) =====
uint32_t vegeStartEpoch  = 0;   // 0 = no iniciado
uint32_t floraStartEpoch = 0;   // 0 = no iniciado

bool vegeActive  = false;
bool floraActive = false;

int  vegeDays    = 0;           // 0 = sin iniciar / "--" en UI
int  floraDays   = 0;

uint32_t lastDateKey = 0;       // yyyymmdd local (-3h)

bool superEnabled = false;

int32_t superAnchorEpochR4 = 0; // ancla absoluta del ciclo (epoch local)

static int32_t nextOnEpoch_R4  = -1;
static int32_t nextOffEpoch_R4 = -1;

int32_t supercycleStartEpochR4 = 0;  // se guarda la fecha/hora de inicio en epoch
int32_t supercycleStartEpochR7 = 0;

const int SUPERCYCLE_13H = 13 * 60; // 780 min
#define SUPERCYCLE_13H_MIN      780UL    // 13 horas
#define SUPERCYCLE_1313_TOTAL   1560UL   // 26 horas

uint32_t super1313AnchorEpochR4 = 0;     // inicio del tramo ON del ciclo 13/13
bool     super1313InitDoneR4    = false; // evita reinicializaciones accidentales

static volatile bool g_needHardReconnect = false;

static unsigned long lastWiFiCheck = 0;

bool otaEjecutadaEsteBoot = false;

unsigned long bootMillis = 0;
uint8_t configTiempoGoogle = 0;
uint8_t configTiempoTelegram = 0;

int cicloLuz = 0;   // variable global si querés reusarla en otros lados

static uint8_t g_relayMask = 0x00; // bit0..bit7 = relés 1..8



typedef struct {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  dotw;   // 0=domingo
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
} datetime_t;

static uint8_t decToBcd(int val) { return (uint8_t)((val / 10 * 16) + (val % 10)); }
static int     bcdToDec(uint8_t val) { return (int)((val / 16 * 10) + (val % 16)); }


// ===== RS485 / Modbus RTU en Waveshare ESP32-S3-ETH-8DI-8RO =====
static const int RS485_TX = 17;
static const int RS485_RX = 18;

// TH-MB-02S: muy común que venga por defecto 4800 8N1
static const uint32_t RS485_BAUD = 4800;

static const uint8_t  MODBUS_ID = 1;
static const uint16_t REG_HUM   = 0;   // Holding Reg 0
static const uint16_t REG_TEMP  = 1;   // Holding Reg 1
// ===== IDs de sensores =====
static const uint8_t AIR_ID_1  = 1;
static const uint8_t AIR_ID_2  = 2;
static const uint8_t AIR_ID_3  = 3;
static const uint8_t AIR_ID_4  = 4;
static const uint8_t SOIL_ID_5 = 5;
static const uint8_t SOIL_ID_6 = 6;

extern HardwareSerial RS485;

// =========================================================
// SENSORES FISICOS
// =========================================================

// -------- Ambiente físicos: IDs 1..4 --------
float temperatura1 = NAN, humedad1 = NAN;
float temperatura2 = NAN, humedad2 = NAN;
float temperatura3 = NAN, humedad3 = NAN;
float temperatura4 = NAN, humedad4 = NAN;

bool sensorAmbOK1 = false;
bool sensorAmbOK2 = false;
bool sensorAmbOK3 = false;
bool sensorAmbOK4 = false;

uint32_t lastAirRead1 = 0;
uint32_t lastAirRead2 = 0;
uint32_t lastAirRead3 = 0;
uint32_t lastAirRead4 = 0;

// -------- Suelo físicos: IDs 5..6 --------
float temperaturaSuelo5 = NAN, humedadSuelo5 = NAN, ECSuelo5 = NAN;
float temperaturaSuelo6 = NAN, humedadSuelo6 = NAN, ECSuelo6 = NAN;

bool sensorSueloOK5 = false;
bool sensorSueloOK6 = false;

uint32_t lastSoilRead5 = 0;
uint32_t lastSoilRead6 = 0;

// =========================================================
// SENSORES VIRTUALES
// =========================================================

struct SensorVirtualTH {
  bool activo;
  uint8_t cantidad;     // cantidad de sensores físicos usados
  uint8_t ids[4];       // ids físicos válidos: 1..4
};

struct SensorVirtualSoil {
  bool activo;
  uint8_t cantidad;     // cantidad de sensores físicos usados
  uint8_t ids[2];       // ids físicos válidos: 5..6
};

// Índice 0 no se usa, para que coincida directo con "sensor virtual N"
SensorVirtualTH sensorVirtualTH[7];
SensorVirtualSoil sensorVirtualSoil[7];

// Valores calculados de sensores virtuales
float virtualTemp[7];
float virtualHum[7];
float virtualSoilTemp[7];
float virtualSoilHum[7];
float virtualSoilEC[7];

bool virtualTH_OK[7];
bool virtualSoil_OK[7];

// =========================================================
// ASIGNACION DE SENSOR VIRTUAL A CADA RELAY
// =========================================================
// Más adelante cada relay apuntará a uno de estos sensores virtuales

uint8_t sensorVirtualAsignadoR1 = 1;
uint8_t sensorVirtualAsignadoR2 = 1;
uint8_t sensorVirtualAsignadoR3 = 1;
uint8_t sensorVirtualAsignadoR4 = 1;
uint8_t sensorVirtualAsignadoR5 = 1;
uint8_t sensorVirtualAsignadoR6 = 1;
uint8_t sensorVirtualAsignadoR7 = 1;
uint8_t sensorVirtualAsignadoR8 = 1;

int sensorR1 = 1;
int sensorR2 = 1;
int sensorR3 = 5;
int sensorR5 = 1;
int sensorR6 = 1;
int sensorR7 = 5;
int sensorR8 = 5;

// ===== Debug / Estado (estos SÍ pueden ser volatile) =====
volatile bool     g_rs485_okH    = false;
volatile bool     g_rs485_okT    = false;
volatile uint16_t g_rawH         = 0;
volatile uint16_t g_rawT         = 0;
volatile uint8_t  g_lastRxLen    = 0;
volatile uint8_t  g_lastErr      = 0;   // 0=OK,1=timeout,2=badCRC,3=exception,4=format,5=wrongID
volatile uint32_t g_rs485_lastMs = 0;   // millis() del último intento/recepción

// ===== Debug RX bytes (buffers NO volatile) =====
uint8_t  g_lastRx[16] = {0};      // bytes RX para mostrar en OLED (HEX)
volatile uint8_t g_lastRxN = 0;   // cantidad válida en g_lastRx

// ===== Último request (para OLED) =====
volatile uint8_t  g_lastId  = MODBUS_ID;
volatile uint8_t  g_lastFn  = 0x03;     // Read Holding Registers
volatile uint16_t g_lastReg = REG_HUM;
volatile uint16_t g_lastQty = 1;

volatile uint8_t  g_scanStep = 0;     // 0..?
volatile uint8_t  g_scanFound = 0;    // 0=no, 1=si
volatile uint32_t g_scanBaud = 0;
volatile uint8_t  g_scanParity = 0;   // 0=N, 1=E
volatile uint8_t  g_scanId = 1;
volatile uint16_t g_scanLastOut = 0;
volatile uint8_t  g_scanLastOk  = 0;
volatile uint32_t g_scanTries   = 0;
// ============================
// RTC / NTP - Globals
// ============================
bool rtcTienePila = false;     // inferido: mantuvo hora (VL=0) => probablemente SI
bool rtcHoraValida = false;    // hora del RTC es confiable (VL=0 y año razonable)
bool ntpSincronizado = false;  // última sync NTP exitosa en esta sesión

int horaActual = 0;            // variables pedidas: hora y minuto (derivadas del RTC)
int minutoActual = 0;

static bool wifiEstabaConectado = false; // para detectar "transición" de reconexión

// Ajustá a tu gusto:
static const uint32_t NTP_TIMEOUT_MS = 6000UL;

// Lista de NTPs (la misma que ya usás)
static const char* NTP_SERVERS[] = {
  "time.google.com",
  "time.cloudflare.com",
  "ar.pool.ntp.org",
  "pool.ntp.org",
  nullptr
};

static const long GMT_OFFSET_SEC = 0;
static const int  DST_OFFSET_SEC = 0;

float minR6 = 0;
float maxR6 = 0;
byte paramR6 = 0;
int timeOnR6 = 0;
int timeOffR6 = 0;
int horaOnR6 = 0;
int minOnR6 = 0;
int horaOffR6 = 0;
int minOffR6 = 0;

int R6config = -1;

int tiempoR6 = 0;
bool esperandoTiempoR6 = false;

int tiempoEncendidoR6 = 5; // en minutos
int tiempoApagadoR6 = 10;  // en minutos
unsigned long previousMillisR6 = 0;

byte direccionR6 = 0;



// ====== Estado lógico y físico de R6 ======
uint8_t estadoR6 = 0;
uint8_t modoR6   = 1; // mismo criterio que R5
uint8_t R6estado = LOW;

// Valores crudos (debug)
uint16_t rawSoilHum  = 0;
uint16_t rawSoilTemp = 0;
uint16_t rawSoilEC   = 0;

// Valores procesados
float soilHum  = NAN;
float soilTemp = NAN;
float soilEC   = NAN;

// Estado
bool soilSensorOK = false;
unsigned long lastSoilRead = 0;

String cachedHTML;
String cachedCSS;
String cachedJS;

bool sensorAir1Activo  = true;
bool sensorAir2Activo  = false;
bool sensorAir3Activo  = false;
bool sensorAir4Activo  = false;

bool sensorSoil5Activo = false;
bool sensorSoil6Activo = false;

// --- Suspensión automática por fallos consecutivos ---
// índice 0 no se usa, índices 1-4 = aire, 5-6 = suelo
static uint8_t  rs485FailCount[7]      = {0};
static uint32_t rs485SuspendedUntil[7] = {0};
static const uint8_t  RS485_MAX_FAILS       = 3;
static const uint32_t RS485_SUSPEND_MS      = 60000UL; // 1 minuto

// ===============================
// QR automático en modo AP
// ===============================
const uint32_t QR_AP_DURATION_MS = 30000UL;

bool qrAPActivo = false;
uint32_t qrAPStartMs = 0;
uint8_t lastAPClients = 0;  

