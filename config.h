
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
#include <ETH.h>
#include <SPI.h>
#include <NTPClient.h>
#include <UniversalTelegramBot.h>
#include <EEPROM.h>
#include <Time.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <ESP32Servo.h>
#include "math.h"
#include <esp_wifi.h>
#include <HTTPUpdate.h>
#include <PubSubClient.h>
#include "mbedtls/base64.h"
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"
#include <FFat.h>
#include <Preferences.h>
#include "QRCodeGenerator.h"

// WPA2 Enterprise (redes con usuario+contraseña tipo universidad/empresa)
// WPA2_ENT_ENABLE() deshabilita la validación de tiempo del certificado del
// servidor (disable_time_check) ANTES de activar enterprise. Sin esto, si el
// RTC/NTP todavía no sincronizó al boot, el ESP32 rechaza el cert del AP
// silenciosamente y la autenticación falla aunque las credenciales sean correctas.
#if __has_include("esp_eap_client.h")
  #include "esp_eap_client.h"
  #define WPA2_ENT_SET_IDENTITY esp_eap_client_set_identity
  #define WPA2_ENT_SET_USERNAME esp_eap_client_set_username
  #define WPA2_ENT_SET_PASSWORD esp_eap_client_set_password
  // DISABLE: limpia la config enterprise anterior antes de reconfigurar.
  // Sin esto el primer intento falla (reason 210) porque el stack reutiliza
  // credenciales obsoletas del ciclo previo de inicialización.
  #define WPA2_ENT_DISABLE()    esp_wifi_sta_enterprise_disable()
  #define WPA2_ENT_ENABLE()     do { esp_eap_client_set_disable_time_check(true); \
                                     esp_wifi_sta_enterprise_enable(); } while(0)
#else
  #include "esp_wpa2.h"
  #define WPA2_ENT_SET_IDENTITY esp_wifi_sta_wpa2_ent_set_identity
  #define WPA2_ENT_SET_USERNAME esp_wifi_sta_wpa2_ent_set_username
  #define WPA2_ENT_SET_PASSWORD esp_wifi_sta_wpa2_ent_set_password
  #define WPA2_ENT_DISABLE()    esp_wifi_sta_wpa2_ent_disable()
  #define WPA2_ENT_ENABLE()     do { esp_wifi_sta_wpa2_ent_set_disable_time_check(true); \
                                     esp_wifi_sta_wpa2_ent_enable(); } while(0)
#endif


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
#define IR_MODE 10
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
#define EEPROM_SENSOR_TEMP          730  // int32_t: codigo decimal ambiente para temp (R1,R2)
#define EEPROM_SENSOR_HUM           734  // int32_t: codigo decimal ambiente para hum  (R5,R6)
#define EEPROM_SENSOR_SOIL          738  // int32_t: codigo decimal suelo para irrigac (R3,R8)

// Nombres personalizados de relay (char[20] cada uno, gap libre 742-799)
#define EEPROM_R7_CUSTOM_NAME       742  // char[20]: nombre libre para R7
#define EEPROM_R8_CUSTOM_NAME       762  // char[20]: nombre libre para R8
#define EEPROM_PPFD_ACTIVE          782  // uint8_t: ppfdActivo (0=off, 1=on)

// ===== Direcciones EEPROM para config IR A/C =====
// Struct IRCfg (146 bytes) almacenada a partir de EEPROM_IR_CFG_ADDR.
// Ocupa 840..985. Última dirección EEPROM usada: 1023.
#define EEPROM_IR_CFG_ADDR  840   // IRCfg (146 bytes)
#define EEPROM_IR_MAGIC     0xAC  // byte de validación en IRCfg.flag
#define EEPROM_IR_LINK_ADDR 986   // IRLinkCfg (5 bytes)
#define EEPROM_IR_LINK_MAGIC 0x1A

#define IR_RELAY_FOLLOW_LOGIC 0  // relay fisico sigue la demanda del A/C
#define IR_RELAY_TIMER_AUX    1  // relay fisico usa sus horarios TIMER

#define IR_AC_OFF  0
#define IR_AC_COOL 1
#define IR_AC_HEAT 2
#define IR_AC_DRY  3

// ===== VPD auxiliar — umbrales y flags por relé =====
#define EEPROM_VPD_MIN_R1   800  // float
#define EEPROM_VPD_MAX_R1   804  // float
#define EEPROM_VPD_MIN_R2   808  // float
#define EEPROM_VPD_MAX_R2   812  // float
#define EEPROM_VPD_MIN_R5   816  // float
#define EEPROM_VPD_MAX_R5   820  // float
#define EEPROM_VPD_MIN_R6   824  // float
#define EEPROM_VPD_MAX_R6   828  // float
#define EEPROM_VPD_AUX_R1   832  // uint8_t  (bool)
#define EEPROM_VPD_AUX_R2   833  // uint8_t  (bool)
#define EEPROM_VPD_AUX_R5   834  // uint8_t  (bool)
#define EEPROM_VPD_AUX_R6   835  // uint8_t  (bool)
#define EEPROM_VPD_AUX_DELAY_MIN  836  // uint16_t (minutos, 30..120)

// ===== EEPROM: usuario WiFi Enterprise (ocupa [174..213]) =====
#define EEPROM_WIFI_USER  174

// ===== Versión de firmware =====
#define FW_VERSION "3.0"   // cambiala en cada release

// ===== URL del firmware OTA en GitHub =====
#define OTA_FIRMWARE_URL "https://raw.githubusercontent.com/Pergah/DruidaBot3.0/main/backend.ino.bin"
#define OTA_FFAT_URL "https://raw.githubusercontent.com/Pergah/DruidaBot3.0/main/frontend.bin"

// ===== Dominio web (Next.js) =====
#define CLOUD_REGISTER_URL "https://app.datadruida.com.ar/api/device/register"

// ===== MQTT — HiveMQ Cloud =====
// Puerto 8883 = TLS/SSL (requerido por HiveMQ Cloud)
#define MQTT_HOST     "9aa497f67a4b45be8642b3fe37a753c4.s1.eu.hivemq.cloud"
#define MQTT_PORT     8883
#define MQTT_USER     "xMatys"
#define MQTT_PASS     "Druida2026"

// Intervalo de publicación MQTT del estado (milisegundos)
#define MQTT_PUBLISH_INTERVAL_MS  5000UL

// Intervalo de heartbeat al servidor cloud para actualizar last_seen
#define CLOUD_HEARTBEAT_INTERVAL_MS  30000UL

#define ARR_LEN(a) (sizeof(a) / sizeof((a)[0]))

#define SOIL_TIMEOUT 5000
#define MODBUS_ID_SOIL 1   // Sensor suelo: temp, hum, EC

#define MODBUS_ID_AIR  2   // Sensor ambiente: temp, hum

#define PPFD_ID        7   // ZTS-300AL-GH-N01 PAR sensor (Modbus addr cambiada de fábrica 1→7)


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
String wifiUser = "";  // usuario para redes WPA2 Enterprise (vacío = red normal)


int conPW = 1;
int reset = 0;

byte modoR1 = 0;
float minR1 = 0;
float maxR1 = 0;
byte paramR1 = T;  // temperatura - fijo (calefaccion)
int timeOnR1 = 0;
int timeOffR1 = 0;
int horaOnR1 = 0;
int minOnR1 = 0;
int horaOffR1 = 0;
int minOffR1 = 0;

//byte modoR5 = 0;
float minR5 = 0;
float maxR5 = 0;
byte paramR5 = H;  // humedad - fijo (humidificacion)
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
byte paramR2 = T;  // temperatura - fijo (aire acondicionado)


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

int R1name = 5;   // (Calefaccion) - fijo
int R5name = 0;   // (Humidificacion) - fijo
int R2name = 4;   // (A/C) - fijo
int R3name = 2;   // (Irrigacion)
int R4name = 3;   // (Iluminacion)
int R6name = 6;   // (Deshumidificacion) - fijo
int R7name = 3;   // (Iluminacion)
int R8name = 2;   // (Irrigacion)
//int R2irname = 4; // (Aire acondicionado)

// ===== VPD auxiliar =====
// Cada relé regulado (R1, R2, R5, R6) puede activarse como respaldo VPD
// cuando un primario (T o H) no logra su setpoint dentro del timer.
// Los pares vpdMinRx / vpdMaxRx son umbrales VPD propios del relé
// (no pisan minRx/maxRx, que regulan T o H).
byte  vpdAuxR1 = 0, vpdAuxR2 = 0, vpdAuxR5 = 0, vpdAuxR6 = 0;
float vpdMinR1 = 0.8, vpdMaxR1 = 1.4;
float vpdMinR2 = 0.8, vpdMaxR2 = 1.4;
float vpdMinR5 = 0.8, vpdMaxR5 = 1.4;
float vpdMinR6 = 0.8, vpdMaxR6 = 1.4;
uint16_t vpdAuxDelayMin = 60;  // 30..120 minutos

// Estado runtime (no se persiste): cuándo encendió el primario
// y si ya superó el timer sin lograr llegar al setpoint OFF.
unsigned long primaryOnSinceR1 = 0, primaryOnSinceR2 = 0;
unsigned long primaryOnSinceR5 = 0, primaryOnSinceR6 = 0;
bool primaryFailedR1 = false, primaryFailedR2 = false;
bool primaryFailedR5 = false, primaryFailedR6 = false;
bool vpdAuxActiveR1 = false, vpdAuxActiveR2 = false;
bool vpdAuxActiveR5 = false, vpdAuxActiveR6 = false;


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

// Nombres personalizados R7 y R8 (vacío = usar relayNames[RXname])
char customNameR7[20] = "";
char customNameR8[20] = "";

const int SUPERCYCLE_13H = 13 * 60; // 780 min
#define SUPERCYCLE_13H_MIN      780UL    // 13 horas
#define SUPERCYCLE_1313_TOTAL   1560UL   // 26 horas

uint32_t super1313AnchorEpochR4 = 0;     // inicio del tramo ON del ciclo 13/13
bool     super1313InitDoneR4    = false; // evita reinicializaciones accidentales

static volatile bool    g_needHardReconnect   = false;
static volatile uint8_t g_wpaDisconnectReason = 0;   // último reason del evento DISCONNECTED

// Estado global de conectividad a internet (DNS check). Default true para
// no caer al modo "webApp local" en el boot antes del primer chequeo.
// Se actualiza en wifiHealthCheck() cada INTERNET_CHECK_PERIOD.
static volatile bool g_internetReachable = true;

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


// ===== Ethernet W5500 (SPI) — Waveshare ESP32-S3-ETH-8DI-8RO =====
// El chip W5500 está conectado al bus SPI del ESP32-S3 con los siguientes pines.
// Fuente: demo oficial Waveshare (WS_ETH.h).
#define ETH_PHY_TYPE  ETH_PHY_W5500
#define ETH_PHY_ADDR  1
#define ETH_PHY_CS    16   // SPI Chip Select
#define ETH_PHY_IRQ   12   // Interrupción (activo bajo)
#define ETH_PHY_RST   39   // Reset (activo bajo)
#define ETH_SPI_SCK   15
#define ETH_SPI_MISO  14
#define ETH_SPI_MOSI  13

// ===== RS485 / Modbus RTU en Waveshare ESP32-S3-ETH-8DI-8RO =====
static const int RS485_TX = 17;
static const int RS485_RX = 18;

// TH-MB-02S: muy común que venga por defecto 4800 8N1
static const uint32_t RS485_BAUD = 4800;

static const uint8_t  MODBUS_ID = 1;
static const uint16_t REG_HUM   = 0;   // Holding Reg 0
static const uint16_t REG_TEMP  = 1;   // Holding Reg 1
static const uint16_t REG_MODBUS_ID_ADDR = 0x07D0; // Reg 2000: dirección Modbus del sensor
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

int tempSensor = 1;  // codigo decimal de sensores de ambiente para temperatura (R1, R2)
int humSensor  = 1;  // codigo decimal de sensores de ambiente para humedad (R5, R6)
int soilSensor = 5;  // codigo decimal de sensores de suelo para irrigacion (R3, R8)

// Promedio de ambiente publicado en el estado MQTT (actualizado en controlarRelaysYMedirSensores)
float g_displayTemp = NAN;
float g_displayHum  = NAN;

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

static const long GMT_OFFSET_SEC = -3 * 3600; // NTP -> hora local Argentina
static const int  DST_OFFSET_SEC = 0;

float minR6 = 0;
float maxR6 = 0;
byte paramR6 = H;  // humedad - fijo (deshumidificacion)
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

byte direccionR6 = 1;  // 1=bajar - fijo (deshumidificacion)



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

// ====== PPFD sensor (ZTS-300AL-GH-N01, Modbus ID=PPFD_ID) ======
float    g_ppfd       = NAN;
bool     ppfdSensorOK = false;
uint32_t lastPpfdRead = 0;
bool     ppfdActivo   = false; // habilitado desde el panel de admin

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
// índice 0 no se usa, índices 1-4 = aire, 5-6 = suelo, 7 = PPFD
static uint8_t  rs485FailCount[8]      = {0};
static uint32_t rs485SuspendedUntil[8] = {0};
static const uint8_t  RS485_MAX_FAILS       = 3;
static const uint32_t RS485_SUSPEND_MS      = 60000UL; // 1 minuto

// ===============================
// QR automático en modo AP
// ===============================
const uint32_t QR_AP_DURATION_MS = 30000UL;

bool qrAPActivo = false;
uint32_t qrAPStartMs = 0;
uint8_t lastAPClients = 0;

// ===== Ethernet — estado de conexión =====
// g_ethConnected se setea en onEthEvent() (DruidaBot3.0.ino).
// networkConnected() es el reemplazo de WiFi.status()==WL_CONNECTED:
// retorna true si hay red disponible por cualquier interfaz.
static bool      g_ethConnected = false;
static IPAddress g_ethIP;

static inline bool networkConnected() {
  return g_ethConnected || (WiFi.status() == WL_CONNECTED);
}

// ===== Objetos MQTT globales =====
WiFiClientSecure g_mqttTlsClient;
PubSubClient     g_mqttClient(g_mqttTlsClient);
String           g_deviceId      = "";
String           g_deviceCode    = "";   // generado de MAC en computeDeviceCode()
uint32_t         g_lastMqttPub   = 0;
uint32_t         g_lastCloudBeat = 0;
bool             g_publishStateRequested = false;

// ===== Tuya token cache (modo IR) =====
String   g_tuyaToken   = "";
uint32_t g_tuyaTokenMs = 0;  // millis() del último fetch

// ===== IR A/C config (persistida en EEPROM_IR_CFG_ADDR) =====
struct IRCfg {
  char    key[25];    // Tuya Client ID    (max 24 chars)
  char    secret[33]; // Tuya Client Secret (max 32 chars)
  char    region[6];  // Región Tuya: "us", "eu", etc.
  char    home[13];   // Home ID (string decimal)
  char    frio[17];   // Scene ID para FRÍO
  char    calor[17];  // Scene ID para CALOR
  char    dehum[17];  // Scene ID para DESHUMIDIFICACIÓN
  char    off[17];    // Scene ID para APAGAR
  uint8_t flag;       // EEPROM_IR_MAGIC cuando la config es válida
};
IRCfg g_irCfg;
bool  g_irConfigured = false;

struct IRLinkCfg {
  uint8_t magic;
  uint8_t r1Linked;
  uint8_t r1RelayMode;
  uint8_t r6Linked;
  uint8_t r6RelayMode;
};

bool    g_irR1Linked    = false;
bool    g_irR6Linked    = false;
uint8_t g_irR1RelayMode = IR_RELAY_FOLLOW_LOGIC;
uint8_t g_irR6RelayMode = IR_RELAY_FOLLOW_LOGIC;
uint8_t g_irAcState     = IR_AC_OFF;

// ===== Cannalytics Pairing & Ingest =====
#define CANNALYTICS_BASE_URL        "https://mhsspfgirypazdfyhlmy.supabase.co/functions/v1"
#define CANNALYTICS_MODEL           "druida-bot-v1"
#define CANNALYTICS_DISCOVER_MS     10000UL
#define CANNALYTICS_INGEST_DEFAULT_MS 15000UL
#define CANNALYTICS_CODE_ROTATE_MS  (15UL * 60UL * 1000UL)
#define CANNALYTICS_PAIRING_BTN_PIN 0
#define CANNALYTICS_BTN_HOLD_MS     5000UL

Preferences g_cannPrefs;
String   g_cannSerial          = "";
String   g_cannToken           = "";
String   g_cannDeviceId        = "";
String   g_cannCode            = "";
uint32_t g_cannCodeGenMs       = 0;
bool     g_cannHasToken        = false;
bool     g_cannPairing         = false; // true solo cuando server responde invite:true
int      g_cannIngestIntervalS = 15;
uint32_t g_cannLastInviteMs    = 0;    // timer poll /sensor-pair-invite
uint32_t g_cannLastDiscoverMs  = 0;   // timer poll /sensor-discover (solo en modo pairing)
uint32_t g_cannLastIngestMs    = 0;
uint32_t g_cannRetryUntilMs    = 0;
uint32_t g_cannBtnPressMs      = 0;
bool     g_cannBtnWasLow       = false;
