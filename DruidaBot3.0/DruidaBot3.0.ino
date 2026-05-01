
// Proyecto: Druida BOT de DataDruida
// Autor: Bryan Murphy
// Año: 2025
// Licencia: MIT

#include "config.h"

// ================== PROTOTIPOS (coinciden con el uso) ==================
int realDaysSince(uint32_t startEpoch, int tzOffsetSec = -3 * 3600);
int virtualDaysSince(uint32_t startEpoch, int horasLuz, int horasOscuridad, int tzOffsetSec = -3 * 3600);
HardwareSerial RS485(1);
void GuardadoSensoresConfig();
int normalizarCodigoSensoresAmbiente(int codigo);
int normalizarCodigoSensoresSuelo(int codigo);
int parseIntFlexible(String value);
bool riegoPermitidoPorSuelo(bool sensorOK, float humedadSuelo, int minH, int maxH, bool regandoAhora);
String sensorCodeLabel(int code);
static String jsonBool(bool v);
static String jsonFlt(float v, int dec);

static time_t timegm_compat(struct tm *tm) {
  // Forzar mktime a trabajar en UTC
  char *oldTZ = getenv("TZ");
  setenv("TZ", "UTC0", 1);
  tzset();
  time_t t = mktime(tm);
  // Restaurar TZ (opcional)
  if (oldTZ) setenv("TZ", oldTZ, 1);
  else unsetenv("TZ");
  tzset();
  return t;
}


static inline void tcaWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static bool pcfWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCF85063_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool pcfRead(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(PCF85063_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  if (Wire.requestFrom(PCF85063_ADDRESS, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

// Inicializa el RTC PCF85063 y deja el oscilador corriendo.
// Devuelve true si respondió OK.
bool PCF85063_Init() {
  // Config: capacitor 12.5pF + (NO STOP)
  uint8_t ctrl1 = RTC_CTRL_1_CAP_SEL;   // STOP=0
  if (!pcfWrite(RTC_CTRL_1_ADDR, ctrl1)) return false;

  uint8_t readback = 0xFF;
  if (!pcfRead(RTC_CTRL_1_ADDR, readback)) return false;

  // Si STOP quedó en 1, algo no anduvo (RTC detenido)
  if (readback & RTC_CTRL_1_STOP) return false;

  return true;
}

inline void setRelayActiveLow(uint8_t pin, bool on) {
  // En Waveshare (TCA9554): ON = true, OFF = false
  setRelayCH(pin, on);
}



// Helper para descomponer minutos absolutos a HH:MM
static inline void splitHM(int absMin, int &h, int &m) {
  absMin = (absMin % 1440 + 1440) % 1440;
  h = absMin / 60;
  m = absMin % 60;
}


constexpr uint32_t SEC_PER_HOUR = 3600UL;
constexpr uint32_t SEC_PER_DAY  = 86400UL;

// ===== Día real (24 h), contado desde 1
int realDaysSince(uint32_t startEpoch, int tzOffsetSec) {
  if (startEpoch == 0) return 0;

  // trabajar en 64 bits para evitar desbordes y permitir tz negativos
  int64_t nowLocal   = nowUtcSec64()       + (int64_t)tzOffsetSec;
  int64_t startLocal = (int64_t)startEpoch + (int64_t)tzOffsetSec;

  if (nowLocal < startLocal) return 0;

  uint64_t elapsed = (uint64_t)(nowLocal - startLocal);
  uint32_t daysCompleted = (uint32_t)(elapsed / SEC_PER_DAY);
  return (int)(daysCompleted + 1);
}

// ===== Día virtual (horasLuz + horasOscuridad), contado desde 1
int virtualDaysSince(uint32_t startEpoch,
                     int horasLuz,
                     int horasOscuridad,
                     int tzOffsetSec) {
  if (startEpoch == 0) return 0;
  if (horasLuz < 0 || horasOscuridad < 0) return 0;

  // duración del ciclo en segundos (64 bits)
  uint64_t cycleSec = (uint64_t)((uint32_t)horasLuz + (uint32_t)horasOscuridad) * (uint64_t)SEC_PER_HOUR;
  if (cycleSec == 0) return 0;

  int64_t nowLocal   = nowUtcSec64()       + (int64_t)tzOffsetSec;
  int64_t startLocal = (int64_t)startEpoch + (int64_t)tzOffsetSec;

  if (nowLocal < startLocal) return 0;

  uint64_t elapsed = (uint64_t)(nowLocal - startLocal);
  uint64_t cyclesCompleted = elapsed / cycleSec;   // floor
  return (int)(cyclesCompleted + 1);
}


void mostrarEnPantallaOLED(float temperature, float humedad, float DPV,
                           float soilTemp, float soilHum, float soilEC,
                           String hora);


void setup() {

  bootMs = millis();
  bootMillis = millis();

  // --- Serial ---
  Serial.begin(115200);
  delay(200);

  // --- EEPROM ---
  EEPROM.begin(1024);

  // --- FFat ---
  if (!FFat.begin(false)) {
    Serial.println("❌ Error montando FFat");
  } else {
    Serial.println("✅ FFat montado correctamente");
    Serial.printf("Total FFat: %u bytes\n", FFat.totalBytes());
    Serial.printf("Usado FFat: %u bytes\n", FFat.usedBytes());

    preloadFilesToRAM();
  }

  // --- I2C principal (Waveshare) ---
  Wire.begin(I2C_SDA_MAIN, I2C_SCL_MAIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);

  // =========================
  //  Watchdog
  // =========================
  //esp_task_wdt_init(WDT_TIMEOUT, true);
  //esp_task_wdt_add(NULL);

  // ----- Motivo de reinicio -----
  esp_reset_reason_t resetReason = esp_reset_reason();
  (void)resetReason;

  // =========================
  //  Cargar configuración
  // =========================
  Carga_General();
  debugPrintConfig();

  // =========================
  //  Relés (Waveshare TCA9554)
  // =========================
  relayDriverInit();

  // Estados seguros para R1–R6 (OFF)
  setRelayCH(RELAY1, false);
  setRelayCH(RELAY2, false);
  setRelayCH(RELAY3, false);
  setRelayCH(RELAY4, false);
  setRelayCH(RELAY5, false);
  setRelayCH(RELAY6, false);
  setRelayCH(RELAY7, false);
  setRelayCH(RELAY8, false);

  R1estado = HIGH;
  R2estado = HIGH;
  R3estado = HIGH;
  R4estado = HIGH;
  R5estado = HIGH;
  R6estado = HIGH;
  R7estado = HIGH;
  R8estado = HIGH;

  // =========================
  //  Sensores / RTC / Display
  // =========================
  if (!PCF85063_Init()) {
    printf("❌ PCF85063 no detectado / no inicializa");
  } else {
    printf("✅ PCF85063 OK");
  }

  // =========================
  //  RS485
  // =========================
  rs485Init();
  initSensoresVirtuales();

  delay(50);

  // ----- Inicializar pantalla OLED con reintentos -----
  {
    int retriesDisplayInit = 0;
    while (!display.begin(0x3C, true)) {
      Serial.println(F("Error al inicializar la OLED (SH1106), reintentando..."));
      retriesDisplayInit++;
      delay(500);
      if (retriesDisplayInit > 5) {
        Serial.println(F("No se pudo inicializar la OLED, iniciando de todas formas..."));
        break;
      }
      yield();
    }
    display.clearDisplay();
    display.display();
  }

  // ----- UI local -----
  mostrarMensajeBienvenida();

  // =====================================================
  auto initClockOffline = [&]() {
    RTC_UpdateStatusAndHM();

    if (rtcHoraValida) {
      Serial.printf("🕒 RTC válido: %02d:%02d\n", horaActual, minutoActual);
      return;
    }

    uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
    if (magic != EEPROM_MAGIC_VALUE) {
      Serial.println("🕒 EEPROM sin datos válidos (primer arranque o corrupción).");
      return;
    }

    uint8_t h = EEPROM.read(EEPROM_ADDR_HORA_ACTUAL);
    uint8_t m = EEPROM.read(EEPROM_ADDR_MINUTO_ACTUAL);

    if (h > 23 || m > 59) {
      Serial.println("🕒 EEPROM: datos fuera de rango, ignorando.");
      EEPROM.write(EEPROM_ADDR_MAGIC, 0xFF);
      EEPROM.commit();
      return;
    }

    uint16_t totalMin = (uint16_t)h * 60 + m + 1;
    totalMin %= 1440;

    horaActual   = totalMin / 60;
    minutoActual = totalMin % 60;

    Serial.printf("🕒 EEPROM: recuperado %02d:%02d → ajustado %02d:%02d\n",
                  h, m, (int)(totalMin / 60), (int)(totalMin % 60));

    datetime_t dt = {};
    dt.year   = 2026;
    dt.month  = 1;
    dt.day    = 1;
    dt.dotw   = 0;
    dt.hour   = (uint8_t)horaActual;
    dt.minute = (uint8_t)minutoActual;
    dt.second = 0;
    PCF85063_Set_All(dt);

    RTC_UpdateStatusAndHM();
  };

  // ✅ Importante: inicializar clock offline SIEMPRE al arranque
  initClockOffline();

  // =========================
  //  WiFi config
  // =========================
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);

  registerWiFiEvents();

  // --- Pequeña espera post-boot para que el router levante tras un corte ---
  const uint32_t BOOT_GRACE_MS = 10000UL;
  uint32_t tStart = millis();
  while (millis() - tStart < BOOT_GRACE_MS) {
    delay(50);
    yield();
  }

  // ===========================
  //   Conectividad de Red
  // ===========================
  if (modoWiFi == 1) {
    // Modo cliente (STA)
    connectToWiFi(ssid.c_str(), password.c_str());

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Conectado a Wi-Fi exitosamente.");

      secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

      startWebServer();

      String motivoReinicio = obtenerMotivoReinicio();
      bot.sendMessage(chat_id, "Druida Bot is ON (" + motivoReinicio + ")");

      //String keyboardJson = "[[\"STATUS\"], [\"CONTROL\"], [\"CONFIG\"]]";
      //bot.sendMessageWithReplyKeyboard(chat_id, "MENU PRINCIPAL:", "", keyboardJson, true);

      // ----------- Sincronizar NTP -----------
      static const char* NTP_SERVERS[] = {
        "time.google.com",
        "time.cloudflare.com",
        "ar.pool.ntp.org",
        "pool.ntp.org",
        nullptr
      };
      const long GMT_OFFSET_SEC = 0;
      const int  DST_OFFSET_SEC = 0;

      bool horaSincronizada = false;
      for (uint8_t i = 0; NTP_SERVERS[i] != nullptr && !horaSincronizada; ++i) {
        Serial.printf("\n⏱️  Probando NTP: %s\n", NTP_SERVERS[i]);
        configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVERS[i]);

        time_t now = 0;
        uint32_t t0 = millis();
        while (now < 24 * 3600 && (millis() - t0) < 6000UL) {
          now = time(nullptr);
          delay(200);
          yield();
        }
        horaSincronizada = (now >= 24 * 3600);
      }

      if (horaSincronizada) {
        Serial.println("✅ Hora NTP sincronizada");
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {

          datetime_t dt;
          dt.year   = (uint16_t)(timeinfo.tm_year + 1900);
          dt.month  = (uint8_t)(timeinfo.tm_mon + 1);
          dt.day    = (uint8_t)timeinfo.tm_mday;
          dt.dotw   = (uint8_t)timeinfo.tm_wday;
          dt.hour   = (uint8_t)timeinfo.tm_hour;
          dt.minute = (uint8_t)timeinfo.tm_min;
          dt.second = (uint8_t)timeinfo.tm_sec;
          PCF85063_Set_All(dt);

          RTC_UpdateStatusAndHM();

          const char* days[] = { "Domingo","Lunes","Martes","Miércoles","Jueves","Viernes","Sábado" };
          diaNumero = timeinfo.tm_wday;
          Serial.print("📆 Día de hoy: ");
          Serial.println(days[diaNumero]);
        }
      } else {
        Serial.println("❌ NTP no sincronizado. Mantengo clock offline (RTC/EEPROM).");
        initClockOffline();
      }

    } else {
      Serial.println("❌ No se pudo conectar a Wi-Fi. Cambio a Modo AP.");

      initClockOffline();
      startAccessPoint();
      startWebServer();
    }

  } else {
    Serial.println("\nModo AP activado para configuración.");

    initClockOffline();

    startAccessPoint();
    startWebServer();
  }

  // =========================
  //  Info
  // =========================
  Serial.print("chat_id: ");
  Serial.println(chat_id);
  Serial.println("Menu Serial:");
  Serial.println("1. Modificar Red WiFi");
  Serial.println("2. Modificar Chat ID");
  Serial.println("3. Modificar Señal IR");
  Serial.println("4. Mostrar sensores");
}

// Verifica rango y NaN en una sola línea
inline bool sensorOK(float v, float lo, float hi) {
  return !isnan(v) && v >= lo && v <= hi;
}

// Arranca (o reinicia) la pausa inteligente
inline void startPause() {
  enEsperaR2     = true;
  tiempoInicioR2 = millis();
  tiempoEsperaR2 = R2_WAIT_MS;

  // Copia lecturas para debug
  humedadReferenciaR2     = humedad;
  temperaturaReferenciaR2 = temperature;
  dpvReferenciaR2         = DPV;
}


static inline bool timeReached(uint32_t now, uint32_t &last, uint32_t period) {
  if ((uint32_t)(now - last) >= period) {
    last = now;
    return true;
  }
  return false;
}

void atenderWebDurante(uint32_t ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) {
    server.handleClient();
    yield();
    delay(1);
  }
}

void mostrarQR_URL(const String &url, const String &texto);
void mostrarQR_AP();
void mostrarQR_WIFI_LOCAL();
void manejarQRModoAP(uint32_t nowMs);

void mostrarQR_URL(const String &url, const String &texto = "") {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(3)];

  qrcode_initText(&qrcode, qrcodeData, 3, 0, url.c_str());

  int qrSize = qrcode.size;
  int scale = 2;

  int qrPixelSize = qrSize * scale;
  int xOffset = (SCREEN_WIDTH - qrPixelSize) / 2;
  int yOffset = 0;

  for (int y = 0; y < qrSize; y++) {
    for (int x = 0; x < qrSize; x++) {
      if (qrcode_getModule(&qrcode, x, y)) {
        display.fillRect(xOffset + x * scale, yOffset + y * scale, scale, scale, SH110X_WHITE);
      }
    }
  }

  // Solo dibuja texto si NO está vacío
  if (texto.length() > 0) {
    display.setCursor((SCREEN_WIDTH - texto.length() * 6) / 2, 56);
    display.print(texto);
  }

  display.display();
}

void mostrarQR_AP() {
  mostrarQR_URL("http://192.168.4.1/");
}

void mostrarQR_WIFI_LOCAL() {
  String ip = WiFi.localIP().toString();
  mostrarQR_URL("http://" + ip + "/");
}

void manejarQRModoAP(uint32_t nowMs) {
  bool estaEnModoAP = (modoWiFi == 0);

  if (!estaEnModoAP) {
    qrAPActivo = false;
    lastAPClients = 0;
    return;
  }

  uint8_t clientesActuales = WiFi.softAPgetStationNum();

  if (clientesActuales > lastAPClients) {
    qrAPActivo = true;
    qrAPStartMs = nowMs;
    mostrarQR_AP();
  }

  lastAPClients = clientesActuales;

  if (qrAPActivo && (uint32_t)(nowMs - qrAPStartMs) >= QR_AP_DURATION_MS) {
    qrAPActivo = false;
  }
}

void loop() {
  const uint32_t nowMs = millis();

  // Habilitación diferida de persistencia
  if (!canPersist && (uint32_t)(nowMs - bootMs) > 15000UL) {
    canPersist = true;
  }

  // Web server
  server.handleClient();
  yield();
  manejarQRModoAP(nowMs);

  // Intervalos calculados una sola vez
  const uint32_t intervaloGoogle   = (uint32_t)tiempoGoogle   * 60UL * 1000UL;
  const uint32_t intervaloTelegram = (uint32_t)tiempoTelegram * 60UL * 1000UL;

  // -------------------------------------------------
  // Tareas periódicas rápidas / no bloqueantes
  // -------------------------------------------------

  // Auto-scan RS485: mantenerlo escalonado
  //rs485AutoScanStep();

  // Guardado mínimo de hora/minuto en EEPROM cada 60 s
  static uint32_t lastGeneralSaveMs = 0;
  if (timeReached(nowMs, lastGeneralSaveMs, 60000UL)) {
    GuardarHoraMinuto_EEPROM();
  }

  // Heartbeat opcional
  static uint32_t lastBeat = 0;
  if (timeReached(nowMs, lastBeat, 1000UL)) {
    // debug opcional
  }

  // Check de WiFi seguro y espaciado
  if (modoWiFi == 1 && (uint32_t)(nowMs - lastWiFiCheck) >= (uint32_t)wifiCheckInterval) {
    lastWiFiCheck = nowMs;
    checkWiFiConnection();
  }

  // Ping watchdog / debug
  if ((uint32_t)(nowMs - previousMillisWD) >= interval) {
    previousMillisWD = nowMs;
    Serial.println("PING");
  }



  // -------------------------------------------------
  // RTC
  // -------------------------------------------------
  datetime_t now;
  PCF85063_Read_Time(&now);

  int hour   = now.hour;
  int minute = now.minute;
  int second = now.second;
  int day    = now.dotw;

  int localHour = hour - 3;
  if (localHour < 0) localHour += 24;

  // -------------------------------------------------
  // Tick diario
  // -------------------------------------------------
  static uint32_t lastTickDailyMs = 0;
  if (timeReached(nowMs, lastTickDailyMs, 10000UL)) {
    tickDaily();
  }

  // -------------------------------------------------
  // Serial
  // -------------------------------------------------
  int serial = Serial.read();
  (void)serial;

  // -------------------------------------------------
  // Medición + lógica
  // -------------------------------------------------
  float temperature = NAN;
  float humedad     = NAN;

  // =================================================
// SENSOR AMBIENTE PRINCIPAL (ID 1) + lógica existente
// =================================================
controlarRelaysYMedirSensores(localHour, minute, day, temperature, humedad);

// Reflejar en variables nuevas
temperatura1 = temperature;
humedad1     = humedad;
sensorAmbOK1 = isfinite(temperatura1) && isfinite(humedad1);
if (sensorAmbOK1) lastAirRead1 = nowMs;


// =================================================
// SENSORES AMBIENTE ADICIONALES + SUELO
// Lectura espaciada para no trabar la webApp
// =================================================
static uint32_t lastExtraSensorsMs = 0;

if (timeReached(nowMs, lastExtraSensorsMs, 5000UL)) {

  // -----------------------------
  // AMBIENTE ID 2
  // -----------------------------
  float t = NAN, h = NAN;

  if (rs485ReadTH(AIR_ID_2, t, h)) {
    temperatura2 = t;
    humedad2     = h;
    sensorAmbOK2 = true;
    lastAirRead2 = nowMs;
  } else {
    sensorAmbOK2 = false;
  }

  atenderWebDurante(20);

  // -----------------------------
  // AMBIENTE ID 3
  // -----------------------------
  t = NAN;
  h = NAN;

  if (rs485ReadTH(AIR_ID_3, t, h)) {
    temperatura3 = t;
    humedad3     = h;
    sensorAmbOK3 = true;
    lastAirRead3 = nowMs;
  } else {
    sensorAmbOK3 = false;
  }

  atenderWebDurante(20);

  // -----------------------------
  // AMBIENTE ID 4
  // -----------------------------
  t = NAN;
  h = NAN;

  if (rs485ReadTH(AIR_ID_4, t, h)) {
    temperatura4 = t;
    humedad4     = h;
    sensorAmbOK4 = true;
    lastAirRead4 = nowMs;
  } else {
    sensorAmbOK4 = false;
  }

  atenderWebDurante(20);

  // -----------------------------
  // SUELO ID 5
  // -----------------------------
  float st = NAN, sh = NAN, sec = NAN;

  if (rs485ReadSoil(SOIL_ID_5, st, sh, sec)) {
    temperaturaSuelo5 = st;
    humedadSuelo5     = sh;
    ECSuelo5          = sec;
    sensorSueloOK5    = true;
    lastSoilRead5     = nowMs;
  } else {
    sensorSueloOK5 = false;
  }

  atenderWebDurante(20);

  // -----------------------------
  // SUELO ID 6
  // -----------------------------
  st = NAN;
  sh = NAN;
  sec = NAN;

  if (rs485ReadSoil(SOIL_ID_6, st, sh, sec)) {
    temperaturaSuelo6 = st;
    humedadSuelo6     = sh;
    ECSuelo6          = sec;
    sensorSueloOK6    = true;
    lastSoilRead6     = nowMs;
  } else {
    sensorSueloOK6 = false;
  }

  // Compatibilidad temporal con tu código viejo
  soilTemp     = temperaturaSuelo5;
  soilHum      = humedadSuelo5;
  soilEC       = ECSuelo5;
  soilSensorOK = sensorSueloOK5;
  lastSoilRead = lastSoilRead5;
}

  // -------------------------------------------------
  // GOOGLE
  // -------------------------------------------------
  /*if (intervaloGoogle > 0 && (uint32_t)(nowMs - previousMillisGoogle) >= intervaloGoogle) {
    previousMillisGoogle = nowMs;

    if (WiFi.status() == WL_CONNECTED) {
      //sendDataToGoogleSheets();

      maxHum  = -999;
      minHum  = 999;
      maxTemp = -999;
      minTemp = 999;
    }
  }*/

  // -------------------------------------------------
  // TELEGRAM ENVÍO
  // -------------------------------------------------
  if (intervaloTelegram > 0 && (uint32_t)(nowMs - previousMillisTelegram) >= intervaloTelegram) {
    previousMillisTelegram = nowMs;

    if (WiFi.status() == WL_CONNECTED) {
      float tempBot = temperature;
      float humBot  = humedad;

      if (isnan(tempBot) || isnan(humBot)) {
        bool okAir = rs485ReadTH(AIR_ID_1, tempBot, humBot);
        if (!okAir) {
          tempBot = NAN;
          humBot  = NAN;
        }
      }

      float soilTempBot = soilTemp;
      float soilHumBot  = soilHum;
      float soilECBot   = soilEC;

      if (!soilSensorOK || !isSoilSensorAlive() ||
          !isfinite(soilTempBot) || !isfinite(soilHumBot) || !isfinite(soilECBot)) {

        bool okSoilBot = rs485ReadSoil(SOIL_ID_5, soilTempBot, soilHumBot, soilECBot);

        if (!okSoilBot) {
          soilTempBot = NAN;
          soilHumBot  = NAN;
          soilECBot   = NAN;
        }
      }

      String dateTime = "📅 Fecha y Hora: ";
      dateTime += String(now.day) + "/";
      dateTime += String(now.month) + "/";
      dateTime += String(now.year) + " ";
      dateTime += (localHour < 10 ? "0" : "") + String(localHour) + ":";
      dateTime += (now.minute < 10 ? "0" : "") + String(now.minute) + ":";
      dateTime += (second < 10 ? "0" : "") + String(second) + "\n";

      String statusMessage = "🌡️ Temp Aire: " + String(tempBot, 1) + " °C\n";
      statusMessage += "💧 Hum Aire: " + String(humBot, 1) + " %\n";
      statusMessage += "🌬️ VPD: " + String(DPV, 1) + " hPa\n";
      statusMessage += "🌱 Temp Suelo: " + String(soilTempBot, 1) + " °C\n";
      statusMessage += "🪴 Hum Suelo: " + String(soilHumBot, 1) + " %\n";
      statusMessage += "⚡ EC Suelo: " + String(soilECBot, 0) + "\n";
      statusMessage += dateTime;

      bot.sendMessage(chat_id, statusMessage, "");
    }
  }

  // -------------------------------------------------
  // ALERTA TEMP
  // -------------------------------------------------
  static bool highTempAlertSent = false;

  if (!isnan(temperature) && temperature > 40.0f) {
    temperature = 40.0f;

    if (!highTempAlertSent && WiFi.status() == WL_CONNECTED) {
      bot.sendMessage(chat_id, "Alerta, temperatura demasiado alta");
      highTempAlertSent = true;
    }
  } else {
    highTempAlertSent = false;
  }

  // ===== OLED PRINCIPAL =====
  String horaStr = (localHour < 10 ? "0" : "") + String(localHour) + ":" +
                   (minute < 10 ? "0" : "") + String(minute);

  if (qrAPActivo) {
  mostrarQR_AP();
} else {
  mostrarEnPantallaOLED(temperature, humedad, DPV, soilTemp, soilHum, soilEC, horaStr);
}
  //mostrarEnPantallaOLEDDebug(temperature, humedad, DPV, horaStr);

  manejarResetConOLED();

  yield();
}

void manejarResetConOLED() {
  if (reset == 1) {
    display.clearDisplay();
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);

    String msg = "REINICIANDO...";

    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

    int x = (display.width()  - w) / 2;
    int y = (display.height() - h) / 2;

    display.setCursor(x, y);
    display.print(msg);
    display.display();

    delay(1500);
    ESP.restart();
  }
}

void controlarRelaysYMedirSensores(int hour, int minute, int day, float &temperature, float &humedad) {
  // ===== LECTURA RS485 =====
  temperature = NAN;
  humedad     = NAN;
  float temperatureR1 = NAN, humedadR1 = NAN, DPVR1 = NAN;
  float temperatureR2 = NAN, humedadR2 = NAN, DPVR2 = NAN;
  float temperatureR5 = NAN, humedadR5 = NAN, DPVR5 = NAN;
  float temperatureR6 = NAN, humedadR6 = NAN, DPVR6 = NAN;
  float soilTempR3 = NAN, soilHumR3 = NAN, soilECR3 = NAN;
  float soilTempR8 = NAN, soilHumR8 = NAN, soilECR8 = NAN;

bool ok = rs485ReadTH(AIR_ID_1, temperature, humedad);
if (!ok) {
  temperature = NAN;
  humedad     = NAN;
}

  // Compatibilidad temporal:
  // seguimos dejando temperature/humedad como sensor base (ID 1 o el principal actual)
  DPV = calcularDPV(temperature, humedad);

  // Resolver sensores virtuales por relay de ambiente
  if (!resolverSensorVirtualAmbiente(sensorR1, temperatureR1, humedadR1)) {
    temperatureR1 = temperature;
    humedadR1     = humedad;
  }
  DPVR1 = calcularDPV(temperatureR1, humedadR1);

  if (!resolverSensorVirtualAmbiente(sensorR2, temperatureR2, humedadR2)) {
    temperatureR2 = temperature;
    humedadR2     = humedad;
  }
  DPVR2 = calcularDPV(temperatureR2, humedadR2);

  if (!resolverSensorVirtualAmbiente(sensorR5, temperatureR5, humedadR5)) {
    temperatureR5 = temperature;
    humedadR5     = humedad;
  }
  DPVR5 = calcularDPV(temperatureR5, humedadR5);

  if (!resolverSensorVirtualAmbiente(sensorR6, temperatureR6, humedadR6)) {
    temperatureR6 = temperature;
    humedadR6     = humedad;
  }
  DPVR6 = calcularDPV(temperatureR6, humedadR6);

  bool soilR3OK = resolverSensorVirtualSuelo(sensorR3, soilTempR3, soilHumR3, soilECR3);
  bool soilR8OK = resolverSensorVirtualSuelo(sensorR8, soilTempR8, soilHumR8, soilECR8);

  // Mantener min/max globales con la lectura base actual
  if (isfinite(temperature) && temperature > maxTemp) {
    maxTemp = temperature;
  }

  if (isfinite(temperature) && temperature < minTemp) {
    minTemp = temperature;
  }

  if (isfinite(humedad) && humedad > maxHum) {
    maxHum = humedad;
  }

  if (isfinite(humedad) && humedad < minHum) {
    minHum = humedad;
  }

  if (isfinite(DPV) && DPV > maxVPD) {
    maxVPD = DPV;
  }

  if (isfinite(DPV) && DPV < minVPD) {
    minVPD = DPV;
  }

  //MOSTRAR VALORES POR PANTALLA OLED:

  //manejarReles();

  //MODO MANUAL R1
  if (modoR1 == MANUAL) {
    if (estadoR1 == 1 && R1estado == HIGH) {
      setRelayCH(RELAY1, true);
      R1estado = LOW;
    }

    if (estadoR1 == 0 && R1estado == LOW) {
      setRelayCH(RELAY1, false);
      R1estado = HIGH;
    }
  }

  //MODO MANUAL R5
  if (modoR5 == MANUAL) {
    if (estadoR5 == 1 && R5estado == HIGH) {
      setRelayCH(RELAY5, true);
      R5estado = LOW;
    }

    if (estadoR5 == 0 && R5estado == LOW) {
      setRelayCH(RELAY5, false);
      R5estado = HIGH;
    }
  }

  //MODO MANUAL R6
if (modoR6 == MANUAL) {
  if (estadoR6 == 1 && R6estado == HIGH) {
    setRelayCH(RELAY6, true);
    R6estado = LOW;
  }

  if (estadoR6 == 0 && R6estado == LOW) {
    setRelayCH(RELAY6, false);
    R6estado = HIGH;
  }
}

  // MODO MANUAL R2
  if (modoR2 == MANUAL) {
    if (estadoR2 == 1 && R2estado == HIGH) {
      setRelayCH(RELAY2, true);
      R2estado = LOW;
    }
    if (estadoR2 == 0 && R2estado == LOW) {
      setRelayCH(RELAY2, false);
      R2estado = HIGH;
    }
  }

  // MODO MANUAL R3
  if (modoR3 == MANUAL) {
    if (estadoR3 == 1 && R3estado == HIGH) {
      setRelayCH(RELAY3, true);
      R3estado = LOW;
    }
    if (estadoR3 == 0 && R3estado == LOW) {
      setRelayCH(RELAY3, false);
      R3estado = HIGH;
    }
  }

  // MODO MANUAL R4
  if (modoR4 == MANUAL) {
    if (estadoR4 == 1 && R4estado == HIGH) {
      setRelayCH(RELAY4, true);
      R4estado = LOW;
    }
    if (estadoR4 == 0 && R4estado == LOW) {
      setRelayCH(RELAY4, false);
      R4estado = HIGH;
    }
  }

  // MODO MANUAL R7
  if (modoR7 == MANUAL) {
    if (estadoR7 == 1 && R7estado == HIGH) {
      setRelayCH(RELAY7, true);
      R7estado = LOW;
    }
    if (estadoR7 == 0 && R7estado == LOW) {
      setRelayCH(RELAY7, false);
      R7estado = HIGH;
    }
  }

  // MODO MANUAL R8
  if (modoR8 == MANUAL) {
    if (estadoR8 == 1 && R8estado == HIGH) {
      setRelayCH(RELAY8, true);
      R8estado = LOW;
    }
    if (estadoR8 == 0 && R8estado == LOW) {
      setRelayCH(RELAY8, false);
      R8estado = HIGH;
    }
  }

  //NUEVO MODO MIXTO PARA SUBIR Y BAJAR
    if (modoR1 == AUTO) {

    // ---------- HUMEDAD ----------
    if (paramR1 == H) {
      if (direccionR1 == 0) {
        // SUBIR humedad (humidificador)
        if (humedadR1 <= minR1 && R1estado != LOW) {
          setRelayCH(RELAY1, true);   // ENCENDER
          R1estado = LOW;
        }
        else if (humedadR1 >= maxR1 && R1estado != HIGH) {
          setRelayCH(RELAY1, false);  // APAGAR
          R1estado = HIGH;
        }
      } else {
        // BAJAR humedad (deshumidificador)
        if (humedadR1 >= maxR1 && R1estado != LOW) {
          setRelayCH(RELAY1, true);   // ENCENDER
          R1estado = LOW;
        }
        else if (humedadR1 <= minR1 && R1estado != HIGH) {
          setRelayCH(RELAY1, false);  // APAGAR
          R1estado = HIGH;
        }
      }
    }

    // ---------- TEMPERATURA ----------
    if (paramR1 == T) {
      if (direccionR1 == 0) {
        // SUBIR temperatura (calefactor)
        if (temperatureR1 <= minR1 && R1estado != LOW) {
          setRelayCH(RELAY1, true);
          R1estado = LOW;
        }
        else if (temperatureR1 >= maxR1 && R1estado != HIGH) {
          setRelayCH(RELAY1, false);
          R1estado = HIGH;
        }
      } else {
        // BAJAR temperatura (refrigeración)
        if (temperatureR1 >= maxR1 && R1estado != LOW) {
          setRelayCH(RELAY1, true);
          R1estado = LOW;
        }
        else if (temperatureR1 <= minR1 && R1estado != HIGH) {
          setRelayCH(RELAY1, false);
          R1estado = HIGH;
        }
      }
    }

    // ---------- DPV ----------
    if (paramR1 == D) {
      if (direccionR1 == 0) {
        // SUBIR DPV
        if (DPVR1 <= minR1 && R1estado != LOW) {
          setRelayCH(RELAY1, true);
          R1estado = LOW;
        }
        else if (DPVR1 >= maxR1 && R1estado != HIGH) {
          setRelayCH(RELAY1, false);
          R1estado = HIGH;
        }
      } else {
        // BAJAR DPV
        if (DPVR1 >= maxR1 && R1estado != LOW) {
          setRelayCH(RELAY1, true);
          R1estado = LOW;
        }
        else if (DPVR1 <= minR1 && R1estado != HIGH) {
          setRelayCH(RELAY1, false);
          R1estado = HIGH;
        }
      }
    }
  }

    if (modoR5 == AUTO) {

    // ---------- HUMEDAD ----------
    if (paramR5 == H) {
      if (direccionR5 == 0) {
        // SUBIR humedad
        if (humedadR5 <= minR5 && R5estado != LOW) {
          setRelayCH(RELAY5, true);   // ENCENDER
          R5estado = LOW;
        }
        else if (humedadR5 >= maxR5 && R5estado != HIGH) {
          setRelayCH(RELAY5, false);  // APAGAR
          R5estado = HIGH;
        }
      } else {
        // BAJAR humedad
        if (humedadR5 >= maxR5 && R5estado != LOW) {
          setRelayCH(RELAY5, true);   // ENCENDER
          R5estado = LOW;
        }
        else if (humedadR5 <= minR5 && R5estado != HIGH) {
          setRelayCH(RELAY5, false);  // APAGAR
          R5estado = HIGH;
        }
      }
    }

    // ---------- TEMPERATURA ----------
    if (paramR5 == T) {
      if (direccionR5 == 0) {
        // SUBIR temperatura
        if (temperatureR5 <= minR5 && R5estado != LOW) {
          setRelayCH(RELAY5, true);
          R5estado = LOW;
        }
        else if (temperatureR5 >= maxR5 && R5estado != HIGH) {
          setRelayCH(RELAY5, false);
          R5estado = HIGH;
        }
      } else {
        // BAJAR temperatura
        if (temperatureR5 >= maxR5 && R5estado != LOW) {
          setRelayCH(RELAY5, true);
          R5estado = LOW;
        }
        else if (temperatureR5 <= minR5 && R5estado != HIGH) {
          setRelayCH(RELAY5, false);
          R5estado = HIGH;
        }
      }
    }

    // ---------- DPV ----------
    if (paramR5 == D) {
      if (direccionR5 == 0) {
        // SUBIR DPV
        if (DPVR5 <= minR5 && R5estado != LOW) {
          setRelayCH(RELAY5, true);
          R5estado = LOW;
        }
        else if (DPVR5 >= maxR5 && R5estado != HIGH) {
          setRelayCH(RELAY5, false);
          R5estado = HIGH;
        }
      } else {
        // BAJAR DPV
        if (DPVR5 >= maxR5 && R5estado != LOW) {
          setRelayCH(RELAY5, true);
          R5estado = LOW;
        }
        else if (DPVR5 <= minR5 && R5estado != HIGH) {
          setRelayCH(RELAY5, false);
          R5estado = HIGH;
        }
      }
    }
  }




    if (modoR6 == AUTO) {

    // ---------- HUMEDAD ----------
    if (paramR6 == H) {
      if (direccionR6 == 0) {
        // SUBIR humedad
        if (humedadR6 <= minR6 && R6estado != LOW) {
          setRelayCH(RELAY6, true);   // ENCENDER
          R6estado = LOW;
        }
        else if (humedadR6 >= maxR6 && R6estado != HIGH) {
          setRelayCH(RELAY6, false);  // APAGAR
          R6estado = HIGH;
        }
      } else {
        // BAJAR humedad
        if (humedadR6 >= maxR6 && R6estado != LOW) {
          setRelayCH(RELAY6, true);   // ENCENDER
          R6estado = LOW;
        }
        else if (humedadR6 <= minR6 && R6estado != HIGH) {
          setRelayCH(RELAY6, false);  // APAGAR
          R6estado = HIGH;
        }
      }
    }

    // ---------- TEMPERATURA ----------
    if (paramR6 == T) {
      if (direccionR6 == 0) {
        // SUBIR temperatura
        if (temperatureR6 <= minR6 && R6estado != LOW) {
          setRelayCH(RELAY6, true);
          R6estado = LOW;
        }
        else if (temperatureR6 >= maxR6 && R6estado != HIGH) {
          setRelayCH(RELAY6, false);
          R6estado = HIGH;
        }
      } else {
        // BAJAR temperatura
        if (temperatureR6 >= maxR6 && R6estado != LOW) {
          setRelayCH(RELAY6, true);
          R6estado = LOW;
        }
        else if (temperatureR6 <= minR6 && R6estado != HIGH) {
          setRelayCH(RELAY6, false);
          R6estado = HIGH;
        }
      }
    }

    // ---------- DPV ----------
    if (paramR6 == D) {
      if (direccionR6 == 0) {
        // SUBIR DPV
        if (DPVR6 <= minR6 && R6estado != LOW) {
          setRelayCH(RELAY6, true);
          R6estado = LOW;
        }
        else if (DPVR6 >= maxR6 && R6estado != HIGH) {
          setRelayCH(RELAY6, false);
          R6estado = HIGH;
        }
      } else {
        // BAJAR DPV
        if (DPVR6 >= maxR6 && R6estado != LOW) {
          setRelayCH(RELAY6, true);
          R6estado = LOW;
        }
        else if (DPVR6 <= minR6 && R6estado != HIGH) {
          setRelayCH(RELAY6, false);
          R6estado = HIGH;
        }
      }
    }
  }

  // DATA TIMERS
timeOnR3 = horaOnR3 * 60 + minOnR3;
timeOffR3 = horaOffR3 * 60 + minOffR3;
timeOnR4 = horaOnR4 * 60 + minOnR4;
timeOffR4 = horaOffR4 * 60 + minOffR4;
timeOnR7 = horaOnR7 * 60 + minOnR7;
timeOffR7 = horaOffR7 * 60 + minOffR7;
timeOnR8 = horaOnR8 * 60 + minOnR8;
timeOffR8 = horaOffR8 * 60 + minOffR8;
timeOnR1 = horaOnR1 * 60 + minOnR1;
timeOffR1 = horaOffR1 * 60 + minOffR1;

// >>> NUEVO R5
timeOnR5 = horaOnR5 * 60 + minOnR5;
timeOffR5 = horaOffR5 * 60 + minOffR5;

// >>> NUEVO R6
timeOnR6 = horaOnR6 * 60 + minOnR6;
timeOffR6 = horaOffR6 * 60 + minOffR6;

// Convierte todo a minutos para facilitar la comparación
int currentTime = hour * 60 + minute;
int startR3 = timeOnR3;
int offR3   = timeOffR3;
int startR4 = timeOnR4;
int offR4   = timeOffR4;
int startR7 = timeOnR7;
int offR7   = timeOffR7;
int startR8 = timeOnR8;
int offR8   = timeOffR8;
int startR1 = timeOnR1;
int offR1   = timeOffR1;
int startR5 = timeOnR5;
int offR5   = timeOffR5;
int startR6 = timeOnR6;
int offR6   = timeOffR6;
int c;

  // MODO TIMER R1 (Waveshare: ON=true / OFF=false)
  if (modoR1 == TIMER) {
    if (startR1 < offR1) {
      if (currentTime >= startR1 && currentTime < offR1) {
        if (R1estado == HIGH) {
          setRelayCH(RELAY1, true);
          R1estado = LOW;
        }
      } else {
        if (R1estado == LOW) {
          setRelayCH(RELAY1, false);
          R1estado = HIGH;
        }
      }
    } else {
      if (currentTime >= startR1 || currentTime < offR1) {
        if (R1estado == HIGH) {
          setRelayCH(RELAY1, true);
          R1estado = LOW;
        }
      } else {
        if (R1estado == LOW) {
          setRelayCH(RELAY1, false);
          R1estado = HIGH;
        }
      }
    }
  }

  // MODO TIMER R5
  if (modoR5 == TIMER) {
    if (startR5 < offR5) {
      if (currentTime >= startR5 && currentTime < offR5) {
        if (R5estado == HIGH) {
          setRelayCH(RELAY5, true);
          R5estado = LOW;
        }
      } else {
        if (R5estado == LOW) {
          setRelayCH(RELAY5, false);
          R5estado = HIGH;
        }
      }
    } else {
      if (currentTime >= startR5 || currentTime < offR5) {
        if (R5estado == HIGH) {
          setRelayCH(RELAY5, true);
          R5estado = LOW;
        }
      } else {
        if (R5estado == LOW) {
          setRelayCH(RELAY5, false);
          R5estado = HIGH;
        }
      }
    }
  }

  if (modoR6 == TIMER) {
  if (startR6 < offR6) {
    if (currentTime >= startR6 && currentTime < offR6) {
      if (R6estado == HIGH) {
        setRelayCH(RELAY6, true);
        R6estado = LOW;
      }
    } else {
      if (R6estado == LOW) {
        setRelayCH(RELAY6, false);
        R6estado = HIGH;
      }
    }
  } else {
    if (currentTime >= startR6 || currentTime < offR6) {
      if (R6estado == HIGH) {
        setRelayCH(RELAY6, true);
        R6estado = LOW;
      }
    } else {
      if (R6estado == LOW) {
        setRelayCH(RELAY6, false);
        R6estado = HIGH;
      }
    }
  }
}

    if (modoR2 == AUTO) {

    if (paramR2 == H) {
      if (isnan(humedadR2) || humedadR2 < 0 || humedadR2 > 99.9) {
        humedadR2 = 0;
      } else {
        if (humedadR2 > maxR2 && R2estado == HIGH) {
          setRelayCH(RELAY2, true);
          R2estado = LOW;
          delay(200);
        }
        if (humedadR2 < minR2 && R2estado == LOW) {
          setRelayCH(RELAY2, false);
          R2estado = HIGH;
          delay(200);
        }
      }
    }

    if (paramR2 == T) {
      if (isnan(temperatureR2) || temperatureR2 < -10 || temperatureR2 > 50) {
      } else {
        if (temperatureR2 > maxR2 && R2estado == HIGH) {
          setRelayCH(RELAY2, true);
          R2estado = LOW;
          delay(200);
        }
        if (temperatureR2 < minR2 && R2estado == LOW) {
          setRelayCH(RELAY2, false);
          R2estado = HIGH;
          delay(200);
        }
      }
    }

    if (paramR2 == D) {
      if (DPVR2 > maxR2 && R2estado == HIGH) {
        setRelayCH(RELAY2, true);
        R2estado = LOW;
        delay(200);
      }
      if (DPVR2 < minR2 && R2estado == LOW) {
        setRelayCH(RELAY2, false);
        R2estado = HIGH;
        delay(200);
      }
    }

    if (paramR2 == HT) {
      static byte activador = 0;

      bool humedadValida = !(isnan(humedadR2) || humedadR2 < 0 || humedadR2 > 99.9);
      bool temperaturaValida = !(isnan(temperatureR2) || temperatureR2 < -10 || temperatureR2 > 50);

      if (humedadValida && temperaturaValida) {
        if (R2estado == HIGH) {
          if (humedadR2 > maxR2) {
            setRelayCH(RELAY2, true);
            R2estado = LOW;
            activador = 1;
            delay(200);
          } else if (temperatureR2 > maxTR2) {
            setRelayCH(RELAY2, true);
            R2estado = LOW;
            activador = 2;
            delay(200);
          }
        }
        else if (R2estado == LOW) {
          if ((activador == 1 && humedadR2 < minR2) ||
              (activador == 2 && temperatureR2 < minTR2)) {
            setRelayCH(RELAY2, false);
            R2estado = HIGH;
            activador = 0;
            delay(200);
          }
        }
      }
    }
  }

  // ===== RIEGO R3 (AUTO) =====
  if (modoR3 == AUTO) {

    bool ventanaActiva = false;
    int diaActual = day;
    int diaAyer   = (day + 6) % 7;

    if (startR3 < offR3) {
      if (diasRiego[diaActual] == 1 &&
          currentTime >= startR3 &&
          currentTime < offR3) {
        ventanaActiva = true;
      }
    } else {
      if (diasRiego[diaActual] == 1 &&
          currentTime >= startR3) {
        ventanaActiva = true;
      }
      else if (diasRiego[diaAyer] == 1 &&
               currentTime < offR3) {
        ventanaActiva = true;
      }
    }

    bool riegoPermitidoSensor = riegoPermitidoPorSuelo(soilR3OK, soilHumR3, minR3, maxR3, enRiego);

    if (ventanaActiva && riegoPermitidoSensor) {

      int diaLogicoRiego = diaActual;
      if (startR3 >= offR3 && currentTime < offR3) {
        diaLogicoRiego = diaAyer;
      }

      if (!ventanaActivaPrev) {

        setRelayCH(RELAY3, false);
        R3estado = HIGH;

        enRiego = false;
        previousMillisRiego = millis() - (unsigned long)tiempoNoRiego * 1000UL;

        if (diaLogicoRiego != ultimoDiaRiego) {
          riegosHechos   = 0;
          ultimoDiaRiego = diaLogicoRiego;
          GuardadoR3();
        }
      }

      riegoIntermitente();

    } else {
      setRelayCH(RELAY3, false);
      R3estado = HIGH;
      enRiego  = false;

      if (ventanaActivaPrev) {
        previousMillisRiego = 0;
      }
    }

    ventanaActivaPrev = ventanaActiva;
  }

  // ===== RIEGO R8 (AUTO) =====
  if (modoR8 == AUTO) {

    bool ventanaActiva = false;
    int diaActual = day;
    int diaAyer   = (day + 6) % 7;

    if (startR8 < offR8) {
      if (diasRiegoR8[diaActual] == 1 &&
          currentTime >= startR8 &&
          currentTime < offR8) {
        ventanaActiva = true;
      }
    } else {
      if (diasRiegoR8[diaActual] == 1 &&
          currentTime >= startR8) {
        ventanaActiva = true;
      }
      else if (diasRiegoR8[diaAyer] == 1 &&
               currentTime < offR8) {
        ventanaActiva = true;
      }
    }

    bool riegoPermitidoSensorR8 = riegoPermitidoPorSuelo(soilR8OK, soilHumR8, minR8, maxR8, enRiegoR8);

    if (ventanaActiva && riegoPermitidoSensorR8) {

      int diaLogicoRiego = diaActual;
      if (startR8 >= offR8 && currentTime < offR8) {
        diaLogicoRiego = diaAyer;
      }

      if (!ventanaActivaPrevR8) {

        setRelayCH(RELAY8, false);
        R8estado = HIGH;

        enRiegoR8 = false;
        previousMillisRiegoR8 = millis() - (unsigned long)tiempoNoRiegoR8 * 1000UL;

        if (diaLogicoRiego != ultimoDiaRiegoR8) {
          riegosHechosR8   = 0;
          ultimoDiaRiegoR8 = diaLogicoRiego;
          GuardadoR8();
        }
      }

      riegoIntermitenteR8();

    } else {
      setRelayCH(RELAY8, false);
      R8estado = HIGH;
      enRiegoR8  = false;

      if (ventanaActivaPrevR8) {
        previousMillisRiegoR8 = 0;
      }
    }

    ventanaActivaPrevR8 = ventanaActiva;
  }

  // MODO AUTO R4 (Luz)
  if (modoR4 == AUTO) {
    if (startR4 < offR4) {
      if (currentTime >= startR4 && currentTime < offR4) {
        if (R4estado == HIGH) {
          setRelayCH(RELAY4, true);
          R4estado = LOW;
        }
      } else {
        if (R4estado == LOW) {
          setRelayCH(RELAY4, false);
          R4estado = HIGH;
        }
      }
    } else {
      if (currentTime >= startR4 || currentTime < offR4) {
        if (R4estado == HIGH) {
          setRelayCH(RELAY4, true);
          R4estado = LOW;
        }
      } else {
        if (R4estado == LOW) {
          setRelayCH(RELAY4, false);
          R4estado = HIGH;
        }
      }
    }
  }

  // MODO AUTO R7 (Luz)
  if (modoR7 == AUTO) {
    if (startR7 < offR7) {
      if (currentTime >= startR7 && currentTime < offR7) {
        if (R7estado == HIGH) {
          setRelayCH(RELAY7, true);
          R7estado = LOW;
        }
      } else {
        if (R7estado == LOW) {
          setRelayCH(RELAY7, false);
          R7estado = HIGH;
        }
      }
    } else {
      if (currentTime >= startR7 || currentTime < offR7) {
        if (R7estado == HIGH) {
          setRelayCH(RELAY7, true);
          R7estado = LOW;
        }
      } else {
        if (R7estado == LOW) {
          setRelayCH(RELAY7, false);
          R7estado = HIGH;
        }
      }
    }
  }

  if (modoR4 == SUPERCICLO1313) {

  int duracion = 13 * 60; // 13 horas en minutos

  // =========================
  // MISMA LOGICA QUE AUTO
  // =========================
  if (startR4 < offR4) {

    if (currentTime >= startR4 && currentTime < offR4) {

      // ====== DEBE ESTAR ENCENDIDO ======
      if (R4estado == HIGH) {
        setRelayCH(RELAY4, true);
        R4estado = LOW;

        // recalcular próximo OFF
        offR4 = (startR4 + duracion) % 1440;
        horaOffR4 = offR4 / 60;
        minOffR4  = offR4 % 60;

        GuardadoR4();
      }

    } else {

      // ====== DEBE ESTAR APAGADO ======
      if (R4estado == LOW) {
        setRelayCH(RELAY4, false);
        R4estado = HIGH;

        // recalcular próximo ON
        startR4 = (offR4 + duracion) % 1440;
        horaOnR4 = startR4 / 60;
        minOnR4  = startR4 % 60;

        GuardadoR4();
      }
    }

  } else {

    // ===== CRUCE DE MEDIANOCHE =====
    if (currentTime >= startR4 || currentTime < offR4) {

      // ====== DEBE ESTAR ENCENDIDO ======
      if (R4estado == HIGH) {
        setRelayCH(RELAY4, true);
        R4estado = LOW;

        offR4 = (startR4 + duracion) % 1440;
        horaOffR4 = offR4 / 60;
        minOffR4  = offR4 % 60;

        GuardadoR4();
      }

    } else {

      // ====== DEBE ESTAR APAGADO ======
      if (R4estado == LOW) {
        setRelayCH(RELAY4, false);
        R4estado = HIGH;

        startR4 = (offR4 + duracion) % 1440;
        horaOnR4 = startR4 / 60;
        minOnR4  = startR4 % 60;

        GuardadoR4();
      }
    }
  }
}

  if (modoR7 == SUPERCICLO1313) {
    int duracion = 13 * 60;

    if (startR7 < offR7) {
      if (currentTime >= startR7 && currentTime < offR7) {
        if (R7estado == HIGH) {
          setRelayCH(RELAY7, true);
          R7estado = LOW;

          offR7 = (startR7 + duracion) % 1440;
          horaOffR7 = offR7 / 60;
          minOffR7  = offR7 % 60;

          GuardadoR7();
        }
      } else {
        if (R7estado == LOW) {
          setRelayCH(RELAY7, false);
          R7estado = HIGH;

          startR7 = (offR7 + duracion) % 1440;
          horaOnR7 = startR7 / 60;
          minOnR7  = startR7 % 60;

          GuardadoR7();
        }
      }
    } else {
      if (currentTime >= startR7 || currentTime < offR7) {
        if (R7estado == HIGH) {
          setRelayCH(RELAY7, true);
          R7estado = LOW;

          offR7 = (startR7 + duracion) % 1440;
          horaOffR7 = offR7 / 60;
          minOffR7  = offR7 % 60;

          GuardadoR7();
        }
      } else {
        if (R7estado == LOW) {
          setRelayCH(RELAY7, false);
          R7estado = HIGH;

          startR7 = (offR7 + duracion) % 1440;
          horaOnR7 = startR7 / 60;
          minOnR7  = startR7 % 60;

          GuardadoR7();
        }
      }
    }
  }
}





float calcularDPV(float temperature, float humedad) {
  const float VPS_values[] = {
    6.57, 7.06, 7.58, 8.13, 8.72, 9.36, 10.02, 10.73, 11.48, 12.28,
    13.12, 14.02, 14.97, 15.98, 17.05, 18.18, 19.37, 20.54, 21.97, 23.38,
    24.86, 26.43, 28.09, 29.83, 31.67, 33.61, 35.65, 37.79, 40.05, 42.42,
    44.92, 47.54, 50.29, 53.18, 56.21, 59.40, 62.73, 66.23, 69.90, 73.74,
    77.76
  };

  if (!isfinite(temperature) || !isfinite(humedad) || temperature > 41 || temperature < 1) {
  return 0.0f;
}
  float temp = temperature - 2;

  float VPS = VPS_values[static_cast<int>(temp) - 1];
  float DPV = 100 - humedad;
  float DPV1 = DPV / 100;
  float DPV2 = DPV1 * VPS;

  return DPV2;
}

//Carga descarga
void Carga_General() {
  Serial.println("Inicializando carga");

  // === Helpers temporales por tamaño ===
  uint8_t  b8;
  uint16_t w16;
  int32_t  i32;
  int64_t  i64;

  // 4 bytes
  EEPROM.get(0,  minR1);
  EEPROM.get(4,  maxR1);
  EEPROM.get(8,  minR2);
  EEPROM.get(12, maxR2);

  // 1 byte
  EEPROM.get(16, b8); paramR1 = b8;
  EEPROM.get(17, b8); paramR2 = b8;

  // 1 byte
  EEPROM.get(18, b8); modoR1 = b8;
  EEPROM.get(19, b8); modoR2 = b8;

  // 1 byte
  EEPROM.get(20, b8); modoR3 = b8;

  // 2 bytes (minutos)
  EEPROM.get(21, w16); timeOnR3  = w16;
  EEPROM.get(23, w16); timeOffR3 = w16;

  // 1 byte
  EEPROM.get(25, b8);  modoR4 = b8;

  // 2 bytes
  EEPROM.get(26, w16); timeOnR4  = w16;
  EEPROM.get(28, w16); timeOffR4 = w16;

  // 1 byte x 7
  for (int u = 0; u < 7; u++) {
    int h = 30 + u;
    EEPROM.get(h, b8);
    diasRiego[u] = b8;
  }

  // Strings
  ssid     = readStringFromEEPROM(37);
  password = readStringFromEEPROM(87);

  // 1 byte (estados)
  EEPROM.get(137, b8); estadoR1 = b8;
  EEPROM.get(138, b8); estadoR2 = b8;
  EEPROM.get(139, b8); estadoR3 = b8;
  EEPROM.get(140, b8); estadoR4 = b8;

  // 4 bytes
  EEPROM.get(141, i32); horaOnR3  = i32;
  EEPROM.get(145, i32); minOnR3   = i32;
  EEPROM.get(149, i32); horaOffR3 = i32;
  EEPROM.get(153, i32); minOffR3  = i32;

  EEPROM.get(158, i32); horaOnR4  = i32;
  EEPROM.get(162, i32); minOnR4   = i32;
  EEPROM.get(166, i32); horaOffR4 = i32;
  EEPROM.get(170, i32); minOffR4  = i32;

  // chat_id (64 bits)
  chat_id = readStringFromEEPROM(215);

  // 4 bytes
  EEPROM.get(240, minR3);
  EEPROM.get(245, maxR3);

  // ====== R5 ======
  // 4 bytes
  EEPROM.get(250, minR5);
  EEPROM.get(255, maxR5);

  // 1 byte
  EEPROM.get(260, b8); modoR5 = b8;

  // 1 byte
  EEPROM.get(265, b8); paramR5 = b8;

  // 1 byte  (¡crítico para no pisar/leer de más!)
  EEPROM.get(270, b8); estadoR5 = b8;

  // 1 byte
  EEPROM.get(272, b8); direccionR5 = b8;

  // ====== R1 (sin cambios) ======
  EEPROM.get(276, i32); horaOnR1  = i32;
  EEPROM.get(280, i32); horaOffR1 = i32;
  EEPROM.get(284, i32); minOnR1   = i32;
  EEPROM.get(288, i32); minOffR1  = i32;

  EEPROM.get(292, i32); tiempoRiego   = i32;
  EEPROM.get(296, i32); tiempoNoRiego = i32;

  EEPROM.get(300, b8);  direccionR1   = b8;

  EEPROM.get(304, i32); currentPosition = i32;
  EEPROM.get(308, i32); horaAmanecer    = i32;
  EEPROM.get(312, i32); horaAtardecer   = i32;

  EEPROM.get(316, b8);  modoWiFi = b8;

  EEPROM.get(320, i32); R1name = i32;

  // ====== R5 tiempos ======
  EEPROM.get(324, i32); minOnR5  = i32;
  // (ANTES: 328) -> AHORA lee minOffR5 en 460
  EEPROM.get(460, i32); minOffR5 = i32;   // <-- NUEVO: minOffR5 @ 460

  // =====================================================
  // NUEVO: cargar horaActual y minutoActual (1 byte c/u)
  // Guardados en 464..465 (pegados a minOffR5 460..463)
  // =====================================================
  uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC);
  uint8_t _hh   = EEPROM.read(EEPROM_ADDR_HORA_ACTUAL);
  uint8_t _mm   = EEPROM.read(EEPROM_ADDR_MINUTO_ACTUAL);

if (magic == EEPROM_MAGIC_VALUE && _hh <= 23 && _mm <= 59) {
  horaActual   = (int)_hh;
  minutoActual = (int)_mm;
  Serial.printf("🕒 Carga_General: hora EEPROM %02d:%02d (magic OK)\n", _hh, _mm);
} else {
  Serial.println("🕒 Carga_General: sin hora válida en EEPROM, se usará RTC/NTP.");
}

  // 4 bytes
  EEPROM.get(330, i32); minTR2 = i32;
  EEPROM.get(334, i32); maxTR2 = i32;

  // ====== R5 horas ======
  EEPROM.get(338, i32); horaOnR5  = i32;
  // (ANTES: 342) -> AHORA lee horaOffR5 en 450
  EEPROM.get(450, i32); horaOffR5 = i32;  // <-- NUEVO: horaOffR5 @ 450
  EEPROM.get(455, i32); supercycleStartEpochR4 = i32;

  // 4 bytes varios
  EEPROM.get(346, i32); horasLuz        = i32;
  EEPROM.get(350, i32); horasOscuridad  = i32;
  EEPROM.get(354, i32); tiempoGoogle    = i32;
  EEPROM.get(358, i32); tiempoTelegram  = i32;
  EEPROM.get(362, i32); cantidadRiegos  = i32;
  EEPROM.get(366, i32); unidadRiego     = i32;
  EEPROM.get(370, i32); unidadNoRiego   = i32;

  // ====== R5 name ======
  EEPROM.get(380, i32); R5name = i32;

  // ====== NUEVO: riego R3 persistente ======
  EEPROM.get(ADDR_RIEGOS_HECHOS, i32);        // <<< NUEVO R3 >>>
  if (i32 < 0) i32 = 0;
  riegosHechos = i32;

  EEPROM.get(ADDR_ULTIMO_DIA_RIEGO, i32);     // <<< NUEVO R3 >>>
  if (i32 < -1 || i32 > 6) i32 = -1;
  ultimoDiaRiego = i32;

  // Clamps defensivos relacionados con riego
  if (cantidadRiegos < 1) cantidadRiegos = 1;                // mínimo 1
  if (riegosHechos > cantidadRiegos) riegosHechos = cantidadRiegos;

  // ====== NUEVO: Ciclos VEGE/FLORA ======
  uint32_t _vegeStart = 0, _floraStart = 0, _dateKey = 0;
  int32_t  _vegeDays  = 0, _floraDays  = 0;
  uint8_t  _vegeAct   = 0, _floraAct   = 0;

  EEPROM.get(ADDR_VEGE_START,   _vegeStart);
  EEPROM.get(ADDR_FLORA_START,  _floraStart);
  EEPROM.get(ADDR_VEGE_DAYS,    _vegeDays);
  EEPROM.get(ADDR_FLORA_DAYS,   _floraDays);
  EEPROM.get(ADDR_LAST_DATEKEY, _dateKey);
  EEPROM.get(ADDR_VEGE_ACTIVE,  _vegeAct);
  EEPROM.get(ADDR_FLORA_ACTIVE, _floraAct);

    // ====== R6 ======
  EEPROM.get(500, minR6);
  EEPROM.get(506, maxR6);

  EEPROM.get(512, b8); modoR6 = b8;
  EEPROM.get(515, b8); paramR6 = b8;
  EEPROM.get(518, b8); estadoR6 = b8;
  EEPROM.get(521, b8); direccionR6 = b8;

  EEPROM.get(524, i32); minOnR6  = i32;
  EEPROM.get(530, i32); minOffR6 = i32;
  EEPROM.get(536, i32); horaOnR6 = i32;
  EEPROM.get(542, i32); horaOffR6 = i32;
  EEPROM.get(548, i32); R6name = i32;

  // ====== R7 (clon R4 / iluminacion) ======
  EEPROM.get(EEPROM_R7_MODO, b8); modoR7 = b8;
  EEPROM.get(EEPROM_R7_TIME_ON, w16); timeOnR7 = w16;
  EEPROM.get(EEPROM_R7_TIME_OFF, w16); timeOffR7 = w16;
  EEPROM.get(EEPROM_R7_ESTADO, b8); estadoR7 = b8;
  EEPROM.get(EEPROM_R7_HORA_ON, i32); horaOnR7 = i32;
  EEPROM.get(EEPROM_R7_MIN_ON, i32); minOnR7 = i32;
  EEPROM.get(EEPROM_R7_HORA_OFF, i32); horaOffR7 = i32;
  EEPROM.get(EEPROM_R7_MIN_OFF, i32); minOffR7 = i32;
  EEPROM.get(EEPROM_R7_HORA_AMANECER, i32); horaAmanecerR7 = i32;
  EEPROM.get(EEPROM_R7_HORA_ATARDECER, i32); horaAtardecerR7 = i32;
  EEPROM.get(EEPROM_R7_HORAS_LUZ, i32); horasLuzR7 = i32;
  EEPROM.get(EEPROM_R7_HORAS_OSCURIDAD, i32); horasOscuridadR7 = i32;
  EEPROM.get(EEPROM_R7_SUPER_START, i32); supercycleStartEpochR7 = i32;
  EEPROM.get(EEPROM_R7_NAME, i32); R7name = i32;

  // ====== R8 (clon R3 / irrigacion) ======
  EEPROM.get(EEPROM_R8_MODO, b8); modoR8 = b8;
  EEPROM.get(EEPROM_R8_TIME_ON, w16); timeOnR8 = w16;
  EEPROM.get(EEPROM_R8_TIME_OFF, w16); timeOffR8 = w16;
  for (int u = 0; u < 7; u++) {
    EEPROM.get(EEPROM_R8_DIAS + u, b8);
    diasRiegoR8[u] = b8;
  }
  EEPROM.get(EEPROM_R8_ESTADO, b8); estadoR8 = b8;
  EEPROM.get(EEPROM_R8_HORA_ON, i32); horaOnR8 = i32;
  EEPROM.get(EEPROM_R8_MIN_ON, i32); minOnR8 = i32;
  EEPROM.get(EEPROM_R8_HORA_OFF, i32); horaOffR8 = i32;
  EEPROM.get(EEPROM_R8_MIN_OFF, i32); minOffR8 = i32;
  EEPROM.get(EEPROM_R8_MIN, minR8);
  EEPROM.get(EEPROM_R8_MAX, maxR8);
  EEPROM.get(EEPROM_R8_TIEMPO_RIEGO, i32); tiempoRiegoR8 = i32;
  EEPROM.get(EEPROM_R8_TIEMPO_NO_RIEGO, i32); tiempoNoRiegoR8 = i32;
  EEPROM.get(EEPROM_R8_CANTIDAD, i32); cantidadRiegosR8 = i32;
  EEPROM.get(EEPROM_R8_UNIDAD_RIEGO, i32); unidadRiegoR8 = i32;
  EEPROM.get(EEPROM_R8_UNIDAD_NO_RIEGO, i32); unidadNoRiegoR8 = i32;
  EEPROM.get(EEPROM_R8_RIEGOS_HECHOS, i32); riegosHechosR8 = i32;
  EEPROM.get(EEPROM_R8_ULTIMO_DIA, i32); ultimoDiaRiegoR8 = i32;
  EEPROM.get(EEPROM_R8_NAME, i32); R8name = i32;

  EEPROM.get(EEPROM_SENSOR_R1, i32); sensorR1 = normalizarCodigoSensoresAmbiente(i32);
  EEPROM.get(EEPROM_SENSOR_R2, i32); sensorR2 = normalizarCodigoSensoresAmbiente(i32);
  EEPROM.get(EEPROM_SENSOR_R3, i32); sensorR3 = normalizarCodigoSensoresSuelo(i32);
  EEPROM.get(EEPROM_SENSOR_R5, i32); sensorR5 = normalizarCodigoSensoresAmbiente(i32);
  EEPROM.get(EEPROM_SENSOR_R6, i32); sensorR6 = normalizarCodigoSensoresAmbiente(i32);
  EEPROM.get(EEPROM_SENSOR_R8, i32); sensorR8 = normalizarCodigoSensoresSuelo(i32);

  EEPROM.get(EEPROM_SENSOR_AIR_ACTIVE + 0, b8); sensorAir1Activo = (b8 == 0xFF) ? true : (b8 != 0);
  EEPROM.get(EEPROM_SENSOR_AIR_ACTIVE + 1, b8); sensorAir2Activo = (b8 == 0xFF) ? false : (b8 != 0);
  EEPROM.get(EEPROM_SENSOR_AIR_ACTIVE + 2, b8); sensorAir3Activo = (b8 == 0xFF) ? false : (b8 != 0);
  EEPROM.get(EEPROM_SENSOR_AIR_ACTIVE + 3, b8); sensorAir4Activo = (b8 == 0xFF) ? false : (b8 != 0);
  EEPROM.get(EEPROM_SENSOR_SOIL_ACTIVE + 0, b8); sensorSoil5Activo = (b8 == 0xFF) ? false : (b8 != 0);
  EEPROM.get(EEPROM_SENSOR_SOIL_ACTIVE + 1, b8); sensorSoil6Activo = (b8 == 0xFF) ? false : (b8 != 0);

  // Asignación con saneo básico
  vegeStartEpoch  = _vegeStart;                 // 0 = no iniciado
  floraStartEpoch = _vegeStart;
  vegeDays        = (_vegeDays  < 0) ? 0 : _vegeDays;
  floraDays       = (_floraDays < 0) ? 0 : _floraDays;
  lastDateKey     = _dateKey;                   // 0 = no inicializado
  vegeActive      = (_vegeAct  != 0);
  floraActive     = (_floraAct != 0);

  // (Opcional) Normalización de coherencia:
  if (!vegeActive)  vegeDays  = (vegeDays  > 0 ? vegeDays  : 0);
  if (!floraActive) floraDays = (floraDays > 0 ? floraDays : 0);

  // ---- Normalizaciones defensivas (evita valores basura en arranque) ----
  // modos 0/1
  if (modoR1 > 9)  modoR1  = 1;
  if (modoR2 > 9)  modoR2  = 1;
  if (modoR3 > 9)  modoR3  = 1;
  if (modoR4 > 13) modoR4  = 1;
  if (modoR5 > 9)  modoR5  = 1;
    if (modoR6 > 9)  modoR6  = 1;
  if (modoR7 > 13) modoR7  = 1;
  if (modoR8 > 9)  modoR8  = 1;

  if (estadoR6 > 1) estadoR6 = 0;
  if (estadoR7 > 1) estadoR7 = 0;
  if (estadoR8 > 1) estadoR8 = 0;
  if (cantidadRiegosR8 < 1) cantidadRiegosR8 = 1;
  if (riegosHechosR8 < 0) riegosHechosR8 = 0;
  if (riegosHechosR8 > cantidadRiegosR8) riegosHechosR8 = cantidadRiegosR8;
  if (ultimoDiaRiegoR8 < -1 || ultimoDiaRiegoR8 > 6) ultimoDiaRiegoR8 = -1;
  if (minR3 < 0 || minR3 > 100) minR3 = 0;
  if (maxR3 < 0 || maxR3 > 100) maxR3 = 0;
  if (minR8 < 0 || minR8 > 100) minR8 = 0;
  if (maxR8 < 0 || maxR8 > 100) maxR8 = 0;

  if (paramR6 > 255)   paramR6 = 0;
  if (direccionR6 > 1) direccionR6 = 0;

  // estados 0/1
  if (estadoR1 > 1) estadoR1 = 0;
  if (estadoR2 > 1) estadoR2 = 0;
  if (estadoR3 > 1) estadoR3 = 0;
  if (estadoR4 > 1) estadoR4 = 0;
  if (estadoR5 > 1) estadoR5 = 0;

  // params/direcciones (si esperás 0..N chicos)
  if (paramR5 > 255)   paramR5   = 0;  // por si es int en tu sketch
  if (direccionR5 > 1) direccionR5 = 0;  // si usás solo 0/1 (invertir o no)

  Serial.println("Carga completa");
}






void Guardado_General() {
  Serial.println("Guardando en memoria..");

  // 4 bytes (int/float)
  EEPROM.put(0,   minR1);
  EEPROM.put(4,   maxR1);
  EEPROM.put(8,   minR2);
  EEPROM.put(12,  maxR2);

  // 1 byte (si tus params son enumeraciones 0..N)
  EEPROM.put(16,  (uint8_t)paramR1);
  EEPROM.put(17,  (uint8_t)paramR2);

  // 1 byte (modo 0/1)
  EEPROM.put(18,  (uint8_t)modoR1);
  EEPROM.put(19,  (uint8_t)modoR2);

  // 1 byte
  EEPROM.put(20,  (uint8_t)modoR3);

  // 2 bytes (si tus timeOn/OffR3 son minutos en uint16)
  EEPROM.put(21,  (uint16_t)timeOnR3);
  EEPROM.put(23,  (uint16_t)timeOffR3);

  // 1 byte
  EEPROM.put(25,  (uint8_t)modoR4);

  // 2 bytes (uint16)
  EEPROM.put(26,  (uint16_t)timeOnR4);
  EEPROM.put(28,  (uint16_t)timeOffR4);

  // 1 byte x 7 (bool/byte de días de riego)
  for (int y = 0; y < 7; y++) {
    int p = 30 + y;
    EEPROM.put(p, (uint8_t)diasRiego[y]);
  }

  // Cadenas (ya usás helper)
  writeStringToEEPROM(37,  ssid);
  writeStringToEEPROM(87,  password);

  // 1 byte cada uno (estados ON/OFF)
  EEPROM.put(137, (uint8_t)estadoR1);
  EEPROM.put(138, (uint8_t)estadoR2);
  EEPROM.put(139, (uint8_t)estadoR3);
  EEPROM.put(140, (uint8_t)estadoR4);

  // 4 bytes c/u
  EEPROM.put(141, (int32_t)horaOnR3);
  EEPROM.put(145, (int32_t)minOnR3);
  EEPROM.put(149, (int32_t)horaOffR3);
  EEPROM.put(153, (int32_t)minOffR3);

  EEPROM.put(158, (int32_t)horaOnR4);
  EEPROM.put(162, (int32_t)minOnR4);
  EEPROM.put(166, (int32_t)horaOffR4);
  EEPROM.put(170, (int32_t)minOffR4);

  // chat_id
  writeStringToEEPROM(215, chat_id);

  // 4 bytes
  EEPROM.put(240, minR3);
  EEPROM.put(245, maxR3);

  // === R5: min/max/modo ===
  EEPROM.put(250, minR5);
  EEPROM.put(255, maxR5);
  EEPROM.put(260, (uint8_t)modoR5);

  // === R5: param/estado/direccion ===
  EEPROM.put(265, (uint8_t)paramR5);
  EEPROM.put(270, (uint8_t)estadoR5);
  EEPROM.put(272, (uint8_t)direccionR5);

  // ==== R1 (sin cambios) ====
  EEPROM.put(276, (int32_t)horaOnR1);
  EEPROM.put(280, (int32_t)horaOffR1);
  EEPROM.put(284, (int32_t)minOnR1);
  EEPROM.put(288, (int32_t)minOffR1);
  EEPROM.put(292, (int32_t)tiempoRiego);
  EEPROM.put(296, (int32_t)tiempoNoRiego);

  EEPROM.put(300, (uint8_t)direccionR1);

  EEPROM.put(304, (int32_t)currentPosition);
  EEPROM.put(308, (int32_t)horaAmanecer);
  EEPROM.put(312, (int32_t)horaAtardecer);

  EEPROM.put(316, (uint8_t)modoWiFi);

  EEPROM.put(320, (int32_t)R1name);

  // === R5: minOn/minOff (4 bytes c/u) ===
  EEPROM.put(324, (int32_t)minOnR5);
  // EEPROM.put(328, (int32_t)minOffR5);           // DEPRECATED
  EEPROM.put(460, (int32_t)minOffR5);             // <-- NUEVO lugar (460..463)

  // 4 bytes c/u
  EEPROM.put(330, (int32_t)minTR2);
  EEPROM.put(334, (int32_t)maxTR2);

  // === R5: horarios (4 bytes c/u) ===
  EEPROM.put(338, (int32_t)horaOnR5);
  // EEPROM.put(342, (int32_t)horaOffR5);          // DEPRECATED
  EEPROM.put(450, (int32_t)horaOffR5);            // <-- NUEVO lugar
  EEPROM.put(455, (int32_t)supercycleStartEpochR4);

  // 4 bytes varios
  EEPROM.put(346, (int32_t)horasLuz);
  EEPROM.put(350, (int32_t)horasOscuridad);
  EEPROM.put(354, (int32_t)tiempoGoogle);
  EEPROM.put(358, (int32_t)tiempoTelegram);
  EEPROM.put(362, (int32_t)cantidadRiegos);
  EEPROM.put(366, (int32_t)unidadRiego);
  EEPROM.put(370, (int32_t)unidadNoRiego);

  // === R5: nombre (numérico) ===
  EEPROM.put(380, (int32_t)R5name);

  // ====== Ciclos VEGE/FLORA ======
  EEPROM.put(ADDR_VEGE_START,   (uint32_t)vegeStartEpoch);
  EEPROM.put(ADDR_FLORA_START,  (uint32_t)floraStartEpoch);
  EEPROM.put(ADDR_VEGE_DAYS,    (int32_t)vegeDays);
  EEPROM.put(ADDR_FLORA_DAYS,   (int32_t)floraDays);
  EEPROM.put(ADDR_LAST_DATEKEY, (uint32_t)lastDateKey);
  EEPROM.put(ADDR_VEGE_ACTIVE,  (uint8_t)(vegeActive ? 1 : 0));
  EEPROM.put(ADDR_FLORA_ACTIVE, (uint8_t)(floraActive ? 1 : 0));

  // ====== NUEVO: riego R3 persistente ======
  int32_t i32;
  i32 = riegosHechos;
  EEPROM.put(ADDR_RIEGOS_HECHOS, i32);

  i32 = ultimoDiaRiego;
  EEPROM.put(ADDR_ULTIMO_DIA_RIEGO, i32);

  // ====== R7 (clon R4 / iluminacion) ======
  EEPROM.put(EEPROM_R7_MODO, (uint8_t)modoR7);
  EEPROM.put(EEPROM_R7_TIME_ON, (uint16_t)timeOnR7);
  EEPROM.put(EEPROM_R7_TIME_OFF, (uint16_t)timeOffR7);
  EEPROM.put(EEPROM_R7_ESTADO, (uint8_t)estadoR7);
  EEPROM.put(EEPROM_R7_HORA_ON, (int32_t)horaOnR7);
  EEPROM.put(EEPROM_R7_MIN_ON, (int32_t)minOnR7);
  EEPROM.put(EEPROM_R7_HORA_OFF, (int32_t)horaOffR7);
  EEPROM.put(EEPROM_R7_MIN_OFF, (int32_t)minOffR7);
  EEPROM.put(EEPROM_R7_HORA_AMANECER, (int32_t)horaAmanecerR7);
  EEPROM.put(EEPROM_R7_HORA_ATARDECER, (int32_t)horaAtardecerR7);
  EEPROM.put(EEPROM_R7_HORAS_LUZ, (int32_t)horasLuzR7);
  EEPROM.put(EEPROM_R7_HORAS_OSCURIDAD, (int32_t)horasOscuridadR7);
  EEPROM.put(EEPROM_R7_SUPER_START, (int32_t)supercycleStartEpochR7);
  EEPROM.put(EEPROM_R7_NAME, (int32_t)R7name);

  // ====== R8 (clon R3 / irrigacion) ======
  EEPROM.put(EEPROM_R8_MODO, (uint8_t)modoR8);
  EEPROM.put(EEPROM_R8_TIME_ON, (uint16_t)timeOnR8);
  EEPROM.put(EEPROM_R8_TIME_OFF, (uint16_t)timeOffR8);
  for (int y = 0; y < 7; y++) {
    EEPROM.put(EEPROM_R8_DIAS + y, (uint8_t)diasRiegoR8[y]);
  }
  EEPROM.put(EEPROM_R8_ESTADO, (uint8_t)estadoR8);
  EEPROM.put(EEPROM_R8_HORA_ON, (int32_t)horaOnR8);
  EEPROM.put(EEPROM_R8_MIN_ON, (int32_t)minOnR8);
  EEPROM.put(EEPROM_R8_HORA_OFF, (int32_t)horaOffR8);
  EEPROM.put(EEPROM_R8_MIN_OFF, (int32_t)minOffR8);
  EEPROM.put(EEPROM_R8_MIN, minR8);
  EEPROM.put(EEPROM_R8_MAX, maxR8);
  EEPROM.put(EEPROM_R8_TIEMPO_RIEGO, (int32_t)tiempoRiegoR8);
  EEPROM.put(EEPROM_R8_TIEMPO_NO_RIEGO, (int32_t)tiempoNoRiegoR8);
  EEPROM.put(EEPROM_R8_CANTIDAD, (int32_t)cantidadRiegosR8);
  EEPROM.put(EEPROM_R8_UNIDAD_RIEGO, (int32_t)unidadRiegoR8);
  EEPROM.put(EEPROM_R8_UNIDAD_NO_RIEGO, (int32_t)unidadNoRiegoR8);
  EEPROM.put(EEPROM_R8_RIEGOS_HECHOS, (int32_t)riegosHechosR8);
  EEPROM.put(EEPROM_R8_ULTIMO_DIA, (int32_t)ultimoDiaRiegoR8);
  EEPROM.put(EEPROM_R8_NAME, (int32_t)R8name);

  EEPROM.put(EEPROM_SENSOR_R1, (int32_t)sensorR1);
  EEPROM.put(EEPROM_SENSOR_R2, (int32_t)sensorR2);
  EEPROM.put(EEPROM_SENSOR_R3, (int32_t)sensorR3);
  EEPROM.put(EEPROM_SENSOR_R5, (int32_t)sensorR5);
  EEPROM.put(EEPROM_SENSOR_R6, (int32_t)sensorR6);
  EEPROM.put(EEPROM_SENSOR_R8, (int32_t)sensorR8);

  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 0, (uint8_t)(sensorAir1Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 1, (uint8_t)(sensorAir2Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 2, (uint8_t)(sensorAir3Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 3, (uint8_t)(sensorAir4Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_SOIL_ACTIVE + 0, (uint8_t)(sensorSoil5Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_SOIL_ACTIVE + 1, (uint8_t)(sensorSoil6Activo ? 1 : 0));

  GuardarHoraMinuto_EEPROM();  // valida RTC antes de escribir


  EEPROM.commit();
  Serial.println("Guardado realizado con exito.");
}




//ACA SE CONFIGURAN TODOS LOS COMANDOS DEL BOT DE TELEGRAM, HABRIA QUE PASARLO A OTRA PESTA

String readSerialInput() {
  String input = "";
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (input.length() > 0) {
          break;  // Romper el bucle si ya se ha leído algo
        }
        continue;  // Ignorar '\n' y '\r' si aún no se ha leído nada
      }
      input += c;
      // Romper si la entrada es mayor que un tamaño razonable
      if (input.length() > 50) {
        break;
      }
    }
  }
  return input;
}


void connectToWiFi(const char* ssid, const char* password) {
  Serial.println();
  Serial.print(F("Conectando a WiFi: "));
  Serial.println(ssid && ssid[0] ? ssid : "(SSID vacío)");

  // Saneamiento previo: rápido y consistente
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_STA);

  // Hostname (si aplica, siempre antes de begin)
  #ifdef ARDUINO_ESP32_RELEASE
  WiFi.setHostname("DruidaBot");
  #endif

  // Reset de estado + begin real
  WiFi.disconnect(true, true);   // corta y limpia
  delay(80);
  if (conPW == 1 && password && password[0]) WiFi.begin(ssid, password);
  else                                       WiFi.begin(ssid);

  // Espera breve; el resto lo maneja checkWiFiConnection()
  const uint32_t TIMEOUT_INITIAL_MS = 15000UL;  // spin corto para no bloquear
  uint32_t t0 = millis();
  while ((millis() - t0) < TIMEOUT_INITIAL_MS) {
    if (WiFi.status() == WL_CONNECTED) {
      String ip = WiFi.localIP().toString();
      Serial.printf("\n✅ WiFi conectado. IP: %s  RSSI: %ld dBm\n",
                    ip.c_str(), WiFi.RSSI());

      // ===== Mostrar en OLED =====
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SH110X_WHITE);

      // Mensaje principal
      display.setCursor((SCREEN_WIDTH - (strlen("WiFi Conectado") * 6)) / 2, 20);
      display.println(F("WiFi Conectado"));

      // IP debajo
      display.setCursor((SCREEN_WIDTH - (ip.length() * 6)) / 2, 40);
      display.println(ip);

      display.display();

      return;
    }
    delay(120);
    Serial.print('.');
    yield();
  }

  Serial.println(F("\n❌ No se conectó en el intento breve de connectToWiFi()."));
}








void writeStringToEEPROM(int addrOffset, const String& strToWrite) {
  byte len = strToWrite.length();
  if (len > 32) len = 32;         // Limitar longitud a 32 caracteres
  EEPROM.write(addrOffset, len);  // Guardar longitud de la cadena
  for (int i = 0; i < len; i++) {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
  EEPROM.write(addrOffset + 1 + len, '\0');  // Añadir terminador nulo
}

String readStringFromEEPROM(int addrOffset) {
  int newStrLen = EEPROM.read(addrOffset);
  if (newStrLen > 32) newStrLen = 32;  // Limitar longitud a 32 caracteres
  char data[newStrLen + 1];
  for (int i = 0; i < newStrLen; i++) {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';  // Asegurar terminación nula
  return String(data);
}

void modificarChatID() {
  while (Serial.available() > 0) {
    Serial.read();
  }
  Serial.println("Por favor ingrese el nuevo Chat ID:");
  while (!Serial.available()) {}
  if (Serial.available() > 0) {
    chat_id = Serial.readStringUntil('\n');
    Serial.println("Chat ID Modificado: ");
    Serial.print("Nuevo Chat ID: ");
    Serial.println(chat_id);
    GuardadoConfig();
  }
}



/*void sendDataToGoogleSheets() {
  HTTPClient http;

  // Estado real de la luz leyendo el pin
  bool luzR4On = isLightOnR4();

  // Guardamos internamente 0 / 1
  cicloLuz = luzR4On ? 1 : 0;

  // Texto para el Excel
  String cicloLuzStr = luzR4On ? "LUZ" : "OSCURIDAD";

  // Construir la URL con los parámetros que deseas enviar
  String url = "https://script.google.com/macros/s/" + String(scriptId) + "/exec?"
               + "maxTemperature=" + String(maxTemp, 2)
               + "&minTemperature=" + String(minTemp, 2)
               + "&maxHumidity=" + String(maxHum, 2)
               + "&minHumidity=" + String(minHum, 2)
               + "&maxVPD=" + String(maxVPD, 2)
               + "&minVPD=" + String(minVPD, 2)
               + "&cicloLuz=" + cicloLuzStr;   // 👈 se envía "LUZ" o "OSCURIDAD"

  Serial.print("Enviando datos a Google Sheets ");
  Serial.println(url);

  http.begin(url);
  int httpResponseCode = http.GET();

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.print("Payload recibido");
    Serial.println(payload);
  } else {
    Serial.print("Error en la solicitud HTTP: ");
    Serial.println(httpResponseCode);
  }

  http.end();
}*/



// Globals requeridas (asegúrate de declararlas en tu sketch principal):
// bool apMode = false;
// unsigned long lastRetryTime = 0;

void checkWiFiConnection() {
  // --- Estrategia y tiempos (tuyos) ---
  const uint32_t CONNECT_SPIN_MS        = 3000UL;     // espera corta al intentar conectar
  const uint32_t BACKOFF_BASE_MS        = 60000UL;    // 1 min
  const uint32_t BACKOFF_MAX_MS         = 5UL * 60UL * 1000UL; // 5 min tope
  const uint32_t AP_RETRY_MIN_MS        = 60000UL;    // 1 min min en AP
  const uint32_t INTERNET_CHECK_PERIOD  = 15000UL;    // check DNS cada 15 s
  const uint32_t BOOT_GRACE_MS          = 20000UL;    // esperar al modem tras corte 

  // Estado interno local persistente (sin nuevas globales)
  static uint32_t nextRetryDelayMs = BACKOFF_BASE_MS;
  static uint32_t lastInternetCheck = 0;
  static bool     webServerUp = false;
  static uint32_t firstCallMillis = 0;
  if (firstCallMillis == 0) firstCallMillis = millis();

  auto ensureWebServer = [&]() {
    if (!webServerUp) { startWebServer(); webServerUp = true; }
  };
  auto stopWebServerFlag = [&]() {
    // si tenés stopWebServer(), llamalo aquí
    webServerUp = false;
  };

  // Prueba de internet real por DNS
  auto internetOK = []() -> bool {
    IPAddress ip;
    bool ok = (WiFi.hostByName("time.google.com", ip) == 1);
    return ok && (ip != IPAddress((uint32_t)0));
  };

  // Si el WiFi está deshabilitado por UI/config, no hacer nada
  if (modoWiFi != 1) return;

  // Grace post-boot: no spamear intentos mientras el módem aún levanta
  if (!apMode && (millis() - firstCallMillis < BOOT_GRACE_MS)) {
    return;
  }

  // ===========================
  //        CONECTADO
  // ===========================
  if (WiFi.status() == WL_CONNECTED) {
    if (apMode) {
      Serial.println(F("Conexión STA restablecida. Cerrando AP..."));
      WiFi.softAPdisconnect(true);
      WiFi.mode(WIFI_STA);
      delay(100);
      apMode = false;
      stopWebServerFlag();
      SyncNTP_to_RTC();
      RTC_UpdateStatusAndHM();
    }

    ensureWebServer();

    // Salud de Internet y/o reconexión fuerte si un evento previo lo pidió
    if (millis() - lastInternetCheck > INTERNET_CHECK_PERIOD) {
      lastInternetCheck = millis();

      if (g_needHardReconnect || !internetOK()) {
        Serial.println(F("⚠️  Conectado pero sin salida o evento de desconexión previo. Reintento fuerte..."));
        // Reconexión fuerte: disconnect + begin corto
        connectToWiFi(ssid.c_str(), password.c_str());
        if (WiFi.status() == WL_CONNECTED) {
          nextRetryDelayMs = BACKOFF_BASE_MS;
          g_needHardReconnect = false;
        }
      } else {
        nextRetryDelayMs = BACKOFF_BASE_MS; // reset backoff
      }
    }
    return;
  }

  // ===========================
  //  DESCONECTADO (STA activo)
  // ===========================
  if (!apMode) {
    Serial.println(F("WiFi desconectado. Reintento fuerte (disconnect+begin)..."));

    connectToWiFi(ssid.c_str(), password.c_str());  // ya hace disconnect(true,true) + begin + spin corto

    // Ventana breve extra para confirmar
    uint32_t t0 = millis();
    while ((millis() - t0) < CONNECT_SPIN_MS) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("¡Conectado!"));
        ensureWebServer();
        nextRetryDelayMs = BACKOFF_BASE_MS;
        g_needHardReconnect = false;
        return;
      }
      delay(150);
      yield();
    }

    // No conectó rápido → activar AP para control local
    Serial.println(F("No se pudo conectar. Activando modo AP..."));
    stopWebServerFlag();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_AP);
    startAccessPoint();
    startWebServer();
    webServerUp = true;
    apMode = true;
    lastRetryTime = millis(); // tu global existente
    return;
  }

  // ===========================
  //         AP ACTIVO
  // ===========================
  // Reintentos con backoff para salir del AP a STA
  uint32_t elapsed = millis() - lastRetryTime;
  if (elapsed >= nextRetryDelayMs && nextRetryDelayMs >= AP_RETRY_MIN_MS) {
    Serial.printf("Reintentando conexión desde AP... (backoff %lus)\n", nextRetryDelayMs / 1000UL);
    lastRetryTime = millis();

    // Cerrar AP temporalmente y probar STA "limpio" (vía connectToWiFi)
    WiFi.softAPdisconnect(true);
    delay(150);

    connectToWiFi(ssid.c_str(), password.c_str());

    // Ventana corta para confirmar
    uint32_t t0 = millis();
    while ((millis() - t0) < CONNECT_SPIN_MS) {
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println(F("¡Conectado desde AP! Cerrando AP y continuando en STA."));
        apMode = false;
        stopWebServerFlag();
        ensureWebServer();
        nextRetryDelayMs = BACKOFF_BASE_MS;
        g_needHardReconnect = false;
        return;
      }
      delay(150);
      yield();
    }

    // Sigue sin conectar → volver a AP y aumentar backoff con jitter (±10%)
    Serial.println(F("Sigue sin conexión. Volviendo a AP y aumentando backoff."));
    WiFi.mode(WIFI_AP);
    startAccessPoint();
    stopWebServerFlag();
    startWebServer();
    webServerUp = true;

    uint32_t doubled = nextRetryDelayMs << 1; // *2
    if (doubled > BACKOFF_MAX_MS) doubled = BACKOFF_MAX_MS;
    uint32_t jitter = (doubled / 10U);
    uint32_t rnd = millis() % (jitter + 1U);
    nextRetryDelayMs = doubled - (jitter / 2U) + rnd;
    if (nextRetryDelayMs < AP_RETRY_MIN_MS) nextRetryDelayMs = AP_RETRY_MIN_MS;
  } else {
    // Aún no toca reintentar → asegurar server en AP
    ensureWebServer();
  }
}







void encenderRele3PorTiempo(int tiempoSegundos) {
  setRelayCH(RELAY3, true); // Enciende el relé
  delay(tiempoSegundos * 1000); // Mantiene encendido por el tiempo indicado
  setRelayCH(RELAY3, false); // Apaga el relé
  bot.sendMessage(chat_id, "Rele apagado después de " + String(tiempoSegundos) + " segundos", "");
  modoR3 = MANUAL;
  estadoR3 = 0;
  R3estado = HIGH;
  GuardadoR3();
}




// INICIO DE LA WEB APP (BACKEND)

void startAccessPoint() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid_AP, password_AP);
    Serial.println("Red WiFi creada con éxito");

    IPAddress IP = WiFi.softAPIP();
    Serial.print("IP del Access Point: ");
    Serial.println(IP);

    startWebServer();  // Reutilizás la misma función
}



void startWebServer() {

  // =========================
  // ARCHIVOS ESTÁTICOS DESDE RAM
  // =========================
  server.on("/style.css", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "max-age=300");
    server.sendHeader("Content-Length", String(cachedCSS.length()));
    server.send(200, "text/css", cachedCSS);
  });
  server.on("/app.js", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "max-age=300");
    server.sendHeader("Content-Length", String(cachedJS.length()));
    server.send(200, "application/javascript", cachedJS);
  });
  server.on("/api/state", HTTP_GET, handleApiState);
  server.on("/api/relay",   HTTP_POST, handleApiRelay);
server.on("/api/params",  HTTP_POST, handleApiParams);
server.on("/api/info",    HTTP_GET,  handleApiInfo);
server.on("/api/sensors", HTTP_GET,  handleApiSensors);
server.on("/api/sensor",  HTTP_POST, handleApiSensorSave);
server.on("/api/alerts",  HTTP_POST, handleApiAlerts);
server.on("/saveWiFi",    HTTP_POST, handleSaveWiFi);

  // =========================
  // ROOT / SPA
  // =========================
  server.on("/", HTTP_GET, handleRoot);

  // =========================
  // RUTAS VIEJAS GET -> SIEMPRE SPA
  // =========================
  

  // =========================
  // CONTROL R1 POST
  // =========================
  server.on("/controlR1On", HTTP_POST, handleControlR1On);
  server.on("/controlR1Off", HTTP_POST, handleControlR1Off);
  server.on("/controlR1Auto", HTTP_POST, handleControlR1Auto);
  server.on("/controlR1Timer", HTTP_POST, handleControlR1Timer);

  // =========================
  // CONTROL R2 POST
  // =========================
  server.on("/controlR2On", HTTP_POST, handleControlR2On);
  server.on("/controlR2Off", HTTP_POST, handleControlR2Off);
  server.on("/controlR2Auto", HTTP_POST, handleControlR2Auto);

  // =========================
  // CONTROL R3 POST
  // =========================
  server.on("/controlR3On", HTTP_POST, handleControlR3On);
  server.on("/controlR3Off", HTTP_POST, handleControlR3Off);
  server.on("/controlR3Auto", HTTP_POST, handleControlR3Auto);
  server.on("/controlR3OnFor", HTTP_POST, handleControlR3OnFor);

  // =========================
  // CONTROL R4 POST
  // =========================
  server.on("/controlR4On", HTTP_POST, handleControlR4On);
  server.on("/controlR4Off", HTTP_POST, handleControlR4Off);
  server.on("/controlR4Auto", HTTP_POST, handleControlR4Auto);
  server.on("/controlR4Superciclo", HTTP_POST, handleControlR4Superciclo);
 // server.on("/controlR4Nube", HTTP_POST, handleControlR4Nube);
 // server.on("/controlR4Mediodia", HTTP_POST, handleControlR4Mediodia);

  server.on("/startVege", HTTP_POST, handleStartVege);
  server.on("/startFlora", HTTP_POST, handleStartFlora);
  server.on("/resetVege", HTTP_POST, handleResetVege);
  server.on("/resetFlora", HTTP_POST, handleResetFlora);
  server.on("/startFloraSuper", HTTP_POST, handleStartFloraSuper);
  server.on("/setFloraDay", HTTP_POST, handleSetFloraDay);
  server.on("/setVegeDay", HTTP_POST, handleSetVegeDay);

  // =========================
  // CONTROL R5 POST
  // =========================
  server.on("/controlR5On", HTTP_POST, handleControlR5On);
  server.on("/controlR5Off", HTTP_POST, handleControlR5Off);
  server.on("/controlR5Auto", HTTP_POST, handleControlR5Auto);
  server.on("/controlR5Timer", HTTP_POST, handleControlR5Timer);

  // =========================
  // CONTROL R6 POST
  // =========================
  server.on("/controlR6On", HTTP_POST, handleControlR6On);
  server.on("/controlR6Off", HTTP_POST, handleControlR6Off);
  server.on("/controlR6Auto", HTTP_POST, handleControlR6Auto);
  server.on("/controlR6Timer", HTTP_POST, handleControlR6Timer);

  server.on("/controlR7On", HTTP_POST, handleControlR7On);
  server.on("/controlR7Off", HTTP_POST, handleControlR7Off);
  server.on("/controlR7Auto", HTTP_POST, handleControlR7Auto);
  server.on("/controlR7Superciclo", HTTP_POST, handleControlR7Superciclo);

  server.on("/controlR8On", HTTP_POST, handleControlR8On);
  server.on("/controlR8Off", HTTP_POST, handleControlR8Off);
  server.on("/controlR8Auto", HTTP_POST, handleControlR8Auto);
  server.on("/controlR8OnFor", HTTP_POST, handleControlR8OnFor);

  // =========================
  // GUARDAR CONFIG POST
  // =========================
  server.on("/saveConfig", HTTP_POST, handleSaveConfig);

  server.on("/saveConfigR1", HTTP_POST, saveConfigR1);
  server.on("/saveConfigR2", HTTP_POST, saveConfigR2);
  server.on("/saveConfigR3", HTTP_POST, saveConfigR3);
  server.on("/saveConfigR4", HTTP_POST, saveConfigR4);
  server.on("/saveConfigR5", HTTP_POST, saveConfigR5);
  server.on("/saveConfigR6", HTTP_POST, saveConfigR6);
  server.on("/saveConfigR7", HTTP_POST, saveConfigR7);
  server.on("/saveConfigR8", HTTP_POST, saveConfigR8);
  server.on("/saveConfigWiFi", HTTP_POST, saveConfigWiFi);

  // =========================
  // WIFI / RED
  // =========================
  server.on("/syncPhoneTime", HTTP_POST, handleSyncPhoneTime);
  server.on("/updateBackend",  HTTP_POST, handleUpdateBackend);
  server.on("/updateFrontend", HTTP_POST, handleUpdateFrontend);
  server.on("/updateGeneral",  HTTP_POST, handleUpdateGeneral);
  server.on("/connectWiFi", HTTP_POST, connectWiFi);
  server.on("/scanWiFi", HTTP_GET, handleScanWiFi);
  server.on("/disconnectWiFi", HTTP_POST, handleDisconnectWiFi);

  // =========================
  // ACCIONES GLOBALES
  // =========================
  server.on("/allOn", HTTP_POST, handleAllOn);
  server.on("/allOff", HTTP_POST, handleAllOff);

  // =========================
  // FALLBACK
  // =========================
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("Servidor web SPA iniciado");
}

void handleApiState() {
  float temperature = temperatura1;
  float humedad     = humedad1;

  bool zeroishBoth = (fabsf(temperature) < 0.05f && fabsf(humedad) < 0.05f);

  bool looksOfflinePattern =
    (fabsf(temperature) < 0.1f) &&
    (humedad >= 1.0f && humedad <= 3.5f) &&
    (fabsf(DPV) < 0.1f);

  bool impossibleValues =
    !isfinite(temperature) || !isfinite(humedad) || !isfinite(DPV) ||
    (temperature < -40.0f || temperature > 85.0f) ||
    (humedad < 0.1f || humedad > 100.0f);

  bool sensorOffline = zeroishBoth || looksOfflinePattern || impossibleValues;

  // =========================
  // HORA
  // =========================
  datetime_t now;
  PCF85063_Read_Time(&now);

  int horaBot = now.hour - 3;
  if (horaBot < 0) horaBot += 24;

  char fechaHora[32];
  snprintf(fechaHora, sizeof(fechaHora),
    "%d/%d/%d %02d:%02d:%02d",
    now.day, now.month, now.year,
    horaBot, now.minute, now.second
  );

  // =========================
  // DÍAS DE CICLO
  // =========================
  int vDaysVege = (vegeStartEpoch > 0 && vegeActive)
    ? realDaysSince(vegeStartEpoch, -3 * 3600)
    : 0;

  int vDaysFloraReal = (floraStartEpoch > 0 && floraActive)
    ? realDaysSince(floraStartEpoch, -3 * 3600)
    : 0;

  int vDaysSuperDisplay = 0;

  if (modoR4 == SUPERCICLO1313 && vDaysFloraReal > 0) {
    vDaysSuperDisplay = vDaysFloraReal - ((vDaysFloraReal - 1) / 12);
    if (vDaysSuperDisplay < 1) vDaysSuperDisplay = 1;
  } else {
    vDaysSuperDisplay = (floraStartEpoch > 0 && floraActive)
      ? virtualDaysSince(floraStartEpoch, horasLuz, horasOscuridad, -3 * 3600)
      : 0;
  }

  // =========================
  // HELPERS JSON
  // =========================
  auto jStr = [](const String& s) -> String {
    String out = s;
    out.replace("\\", "\\\\");
    out.replace("\"", "\\\"");
    return "\"" + out + "\"";
  };

  auto jBool = [](bool b) -> String {
    return b ? "true" : "false";
  };

  auto hhmm = [](int h, int m) -> String {
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", h, m);
    return String(buf);
  };

  auto modeText = [](int modo, int idx) -> String {
    switch (idx) {
      case 2:
        if (modo == MANUAL) return "MANUAL";
        if (modo == AUTO)   return "AUTO";
        if (modo == 9)      return "AUTO IA";
        return "--";

      case 4:
        if (modo == MANUAL)        return "MANUAL";
        if (modo == AUTO)          return "AUTO";
        if (modo == SUPERCICLO)    return "SUP CIC";
        if (modo == SUPERCICLO1313)return "13/13";
        return "--";

      default:
        if (modo == MANUAL) return "MANUAL";
        if (modo == AUTO)   return "AUTO";
        if (modo == TIMER)  return "TIMER";
        return "--";
    }
  };

  auto modeKind = [](int modo) -> String {
    if (modo == MANUAL) return "manual";
    if (modo == TIMER)  return "timer";
    return "auto";
  };

  // =========================
  // ESTADO FÍSICO RELÉS
  // =========================
  bool r1On = relayIsOnHW(RELAY1);
  bool r2On = relayIsOnHW(RELAY2);
  bool r3On = relayIsOnHW(RELAY3);
  bool r4On = relayIsOnHW(RELAY4);
  bool r5On = relayIsOnHW(RELAY5);
  bool r6On = relayIsOnHW(RELAY6);
  bool r7On = relayIsOnHW(RELAY7);
  bool r8On = relayIsOnHW(RELAY8);

  // =========================
  // JSON
  // =========================
  String json = "{";

  // Sensores
  json += "\"temp\":";
  json += sensorOffline ? jStr("--") : jStr(String(temperature, 1));

  json += ",\"hum\":";
  json += sensorOffline ? jStr("--") : jStr(String(humedad, 1));

  json += ",\"vpd\":";
  json += sensorOffline ? jStr("--") : jStr(String(DPV, 1));

  // Alias que usa el frontend nuevo
  json += ",\"dpv\":";
  json += sensorOffline ? jStr("--") : jStr(String(DPV, 1));

  json += ",\"fechaHora\":" + jStr(String(fechaHora));

  // WiFi / servicios
  json += ",\"ssid\":" + jStr(ssid);
  json += ",\"password\":" + jStr(password);
  json += ",\"chat_id\":" + jStr(chat_id);
  json += ",\"tiempoGoogle\":" + String(tiempoGoogle);
  json += ",\"tiempoTelegram\":" + String(tiempoTelegram);

  // Días
  json += ",\"diaVege\":"  + String(vDaysVege > 0 ? vDaysVege : 0);
  json += ",\"diaFlora\":" + String(vDaysFloraReal > 0 ? vDaysFloraReal : 0);

  if (modoR4 == SUPERCICLO || modoR4 == SUPERCICLO1313) {
    json += ",\"diaSuper\":" + String(vDaysSuperDisplay > 0 ? vDaysSuperDisplay : 0);
  }

  // Info general
  json += ",\"uptime\":" + String(millis() / 1000UL);
  json += ",\"version\":\"3.0\"";
  json += ",\"plan\":\"Druida BOT\"";

  // Parámetros globales para el frontend nuevo
  json += ",\"params\":{";
  json += "\"tempMin\":" + String(minR5, 1);
  json += ",\"tempMax\":" + String(maxR5, 1);
  json += ",\"humMin\":"  + String(minR1, 1);
  json += ",\"humMax\":"  + String(maxR2, 1);
  json += ",\"acRelay\":5";
  json += ",\"heaterRelay\":6";
  json += ",\"humidRelay\":1";
  json += ",\"dehumidRelay\":2";
  json += "}";

  // Relés
  json += ",\"relays\":{";

  json += "\"r1\":{";
  json += "\"name\":" + jStr(getRelayName(R1name));
  json += ",\"nameIndex\":" + String(R1name);
  json += ",\"mode\":" + jStr(modeText(modoR1, 1));
  json += ",\"modeKind\":" + jStr(modeKind(modoR1));
  json += ",\"on\":" + jBool(r1On);
  json += ",\"sensorCode\":" + String(sensorR1);
  json += ",\"sensorLabel\":" + jStr(sensorCodeLabel(sensorR1));
  json += ",\"onTime\":" + jStr(hhmm(horaOnR1, minOnR1));
  json += ",\"offTime\":" + jStr(hhmm(horaOffR1, minOffR1));
  json += "}";

  json += ",\"r2\":{";
  json += "\"name\":" + jStr(getRelayName(R2name));
  json += ",\"nameIndex\":" + String(R2name);
  json += ",\"mode\":" + jStr(modeText(modoR2, 2));
  json += ",\"modeKind\":" + jStr(modeKind(modoR2));
  json += ",\"on\":" + jBool(r2On);
  json += ",\"sensorCode\":" + String(sensorR2);
  json += ",\"sensorLabel\":" + jStr(sensorCodeLabel(sensorR2));
  json += "}";

  json += ",\"r3\":{";
  json += "\"name\":" + jStr(getRelayName(R3name));
  json += ",\"nameIndex\":" + String(R3name);
  json += ",\"mode\":" + jStr(modeText(modoR3, 3));
  json += ",\"modeKind\":" + jStr(modeKind(modoR3));
  json += ",\"on\":" + jBool(r3On);
  json += ",\"sensorCode\":" + String(sensorR3);
  json += ",\"sensorLabel\":" + jStr(sensorCodeLabel(sensorR3));
  json += ",\"onTime\":" + jStr(hhmm(horaOnR3, minOnR3));
  json += ",\"offTime\":" + jStr(hhmm(horaOffR3, minOffR3));
  json += "}";

  json += ",\"r4\":{";
  json += "\"name\":" + jStr(getRelayName(R4name));
  json += ",\"nameIndex\":" + String(R4name);
  json += ",\"mode\":" + jStr(modeText(modoR4, 4));
  json += ",\"modeKind\":" + jStr(modeKind(modoR4));
  json += ",\"on\":" + jBool(r4On);
  json += ",\"onTime\":" + jStr(hhmm(horaOnR4, minOnR4));
  json += ",\"offTime\":" + jStr(hhmm(horaOffR4, minOffR4));
  json += "}";

  json += ",\"r5\":{";
  json += "\"name\":" + jStr(getRelayName(R5name));
  json += ",\"nameIndex\":" + String(R5name);
  json += ",\"mode\":" + jStr(modeText(modoR5, 5));
  json += ",\"modeKind\":" + jStr(modeKind(modoR5));
  json += ",\"on\":" + jBool(r5On);
  json += ",\"sensorCode\":" + String(sensorR5);
  json += ",\"sensorLabel\":" + jStr(sensorCodeLabel(sensorR5));
  json += ",\"onTime\":" + jStr(hhmm(horaOnR5, minOnR5));
  json += ",\"offTime\":" + jStr(hhmm(horaOffR5, minOffR5));
  json += "}";

  json += ",\"r6\":{";
  json += "\"name\":" + jStr(getRelayName(R6name));
  json += ",\"nameIndex\":" + String(R6name);
  json += ",\"mode\":" + jStr(modeText(modoR6, 6));
  json += ",\"modeKind\":" + jStr(modeKind(modoR6));
  json += ",\"on\":" + jBool(r6On);
  json += ",\"sensorCode\":" + String(sensorR6);
  json += ",\"sensorLabel\":" + jStr(sensorCodeLabel(sensorR6));
  json += ",\"onTime\":" + jStr(hhmm(horaOnR6, minOnR6));
  json += ",\"offTime\":" + jStr(hhmm(horaOffR6, minOffR6));
  json += "}";

  json += ",\"r7\":{";
  json += "\"name\":" + jStr(getRelayName(R7name));
  json += ",\"nameIndex\":" + String(R7name);
  json += ",\"mode\":" + jStr(modeText(modoR7, 4));
  json += ",\"modeKind\":" + jStr(modeKind(modoR7));
  json += ",\"on\":" + jBool(r7On);
  json += ",\"onTime\":" + jStr(hhmm(horaOnR7, minOnR7));
  json += ",\"offTime\":" + jStr(hhmm(horaOffR7, minOffR7));
  json += "}";

  json += ",\"r8\":{";
  json += "\"name\":" + jStr(getRelayName(R8name));
  json += ",\"nameIndex\":" + String(R8name);
  json += ",\"mode\":" + jStr(modeText(modoR8, 3));
  json += ",\"modeKind\":" + jStr(modeKind(modoR8));
  json += ",\"on\":" + jBool(r8On);
  json += ",\"sensorCode\":" + String(sensorR8);
  json += ",\"sensorLabel\":" + jStr(sensorCodeLabel(sensorR8));
  json += ",\"onTime\":" + jStr(hhmm(horaOnR8, minOnR8));
  json += ",\"offTime\":" + jStr(hhmm(horaOffR8, minOffR8));
  json += "}";

  json += "}";  // cierra relays
  json += "}";  // cierra JSON principal

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleUpdateBackend() {
  Serial.println("[WEB] UPDATE BACKEND recibido");

  if (millis() - bootMillis < 15000UL) {
    Serial.println("[OTA] UPDATE muy cerca del boot. Ignorando.");
    server.send(429, "text/plain", "Esperá unos segundos después del reinicio y volvé a intentar");
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA FW] ERROR: No hay WiFi.");
    server.send(400, "text/plain", "No hay WiFi para actualizar Backend");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Iniciando actualización Backend...");

  delay(500);
  server.handleClient();
  delay(500);

  doOTAUpdateFirmware();
}

void handleUpdateFrontend() {
  Serial.println("[WEB] UPDATE FRONTEND recibido");

  if (millis() - bootMillis < 15000UL) {
    Serial.println("[OTA] UPDATE muy cerca del boot. Ignorando.");
    server.send(429, "text/plain", "Esperá unos segundos después del reinicio y volvé a intentar");
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA FFat] ERROR: No hay WiFi.");
    server.send(400, "text/plain", "No hay WiFi para actualizar Frontend");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Iniciando actualización Frontend...");

  delay(500);
  server.handleClient();
  delay(500);

  doOTAUpdateFFat();
}

void handleUpdateGeneral() {
  if (WiFi.status() != WL_CONNECTED) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(400, "text/plain", "No hay WiFi para actualización general");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Actualización general iniciada...");

  delay(500);
  doOTAUpdateAll();
}

// ================================================================
// handleSetVegeDay y handleSetFloraDay
// (si no los tenés, agregalos también)
// ================================================================

void handleSetVegeDay() {
  if (!server.hasArg("vegeDay")) {
    server.send(400, "text/plain", "Falta vegeDay");
    return;
  }
  int day = server.arg("vegeDay").toInt();
  if (day < 1 || day > 200) {
    server.send(400, "text/plain", "Día inválido");
    return;
  }

  datetime_t now;
  PCF85063_Read_Time(&now);
  uint32_t nowEpoch = unixFromPCF(now);

  // Ajustar epoch para que realDaysSince devuelva 'day'
  vegeStartEpoch = nowEpoch - (uint32_t)(day - 1) * 86400UL;
  vegeActive     = true;
  vegeDays       = day;
  GuardadoR4();

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Dia VEGE fijado en " + String(day));
}

void handleSetFloraDay() {
  if (!server.hasArg("floraDay")) {
    server.send(400, "text/plain", "Falta floraDay");
    return;
  }
  int day = server.arg("floraDay").toInt();
  if (day < 1 || day > 200) {
    server.send(400, "text/plain", "Día inválido");
    return;
  }

  datetime_t now;
  PCF85063_Read_Time(&now);
  uint32_t nowEpoch = unixFromPCF(now);

  floraStartEpoch = nowEpoch - (uint32_t)(day - 1) * 86400UL;
  floraActive     = true;
  floraDays       = day;
  GuardadoR4();

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Dia FLORA fijado en " + String(day));
}



void handleScanWiFi() {
  int n = WiFi.scanNetworks();

  String json = "{\"networks\":[";
  for (int i = 0; i < n; i++) {
    String ss = WiFi.SSID(i);
    ss.replace("\\", "\\\\");
    ss.replace("\"", "\\\"");
    json += "\"" + ss + "\"";
    if (i < n - 1) json += ",";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

void handleDisconnectWiFi() {
  modoWiFi = 0;
  GuardadoConfig();
  WiFi.disconnect(true);
  Serial.println("WiFi desconectado. modoWiFi = 0");

  server.send(200, "text/html", buildDisconnectWiFiPage("Desconectando WiFi…", "/"));
  delay(2000);
  ESP.restart();
}

void handleAllOn() {
  // Sincronizar todos los estados internos
  estadoR1 = 1; modoR1 = MANUAL; R1estado = LOW;
  estadoR2 = 1; modoR2 = MANUAL; R2estado = LOW;
  estadoR3 = 1; modoR3 = MANUAL; R3estado = LOW;
  estadoR4 = 1; modoR4 = MANUAL; R4estado = LOW;
  estadoR5 = 1; modoR5 = MANUAL; R5estado = LOW;
  estadoR6 = 1; modoR6 = MANUAL; R6estado = LOW;
  estadoR7 = 1; modoR7 = MANUAL; R7estado = LOW;
  estadoR8 = 1; modoR8 = MANUAL; R8estado = LOW;
  setAllRelays(true);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Encendiendo todo...");
}

void handleAllOff() {
  estadoR1 = 0; modoR1 = MANUAL; R1estado = HIGH;
  estadoR2 = 0; modoR2 = MANUAL; R2estado = HIGH;
  estadoR3 = 0; modoR3 = MANUAL; R3estado = HIGH;
  estadoR4 = 0; modoR4 = MANUAL; R4estado = HIGH;
  estadoR5 = 0; modoR5 = MANUAL; R5estado = HIGH;
  estadoR6 = 0; modoR6 = MANUAL; R6estado = HIGH;
  estadoR7 = 0; modoR7 = MANUAL; R7estado = HIGH;
  estadoR8 = 0; modoR8 = MANUAL; R8estado = HIGH;
  setAllRelays(false);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Apagando todo...");
}

void handleNotFound() {
  server.send(404, "text/plain", "404");
}

String buildDisconnectWiFiPage(const String& mensaje, const String& redireccion) {
  return "";
}

String buildAppShell(const String& innerHtml, const String& title = "Druida BOT") {
  return innerHtml;
}

//CONFIG WIFI IPLOCAL

void handleRoot() {
  if (cachedHTML.length() == 0) {
    server.send(503, "text/plain", "index.html no cargado en RAM");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Length", String(cachedHTML.length()));
  server.send(200, "text/html", cachedHTML);
}

String buildInlineAsyncControlHelper() {
  return "";
}




void handleConfirmation(const String& mensaje, const String& redireccion) {
  String json = "{";
  json += "\"ok\":true,";
  json += "\"message\":\"" + mensaje + "\",";
  json += "\"redirect\":\"" + redireccion + "\",";
  json += "\"delay\":3000";
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}




// ── R1 ──────────────────────────────────────────────────────────
void handleControlR1On() {
  estadoR1 = 1;
  modoR1   = MANUAL;
  R1estado = LOW;          // ← CRÍTICO: sincronizar con el loop()
  setRelayCH(RELAY1, true);
  GuardadoR1();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R1name) + " encendida");
}

void handleControlR1Off() {
  estadoR1 = 0;
  modoR1   = MANUAL;
  R1estado = HIGH;         // ← CRÍTICO
  setRelayCH(RELAY1, false);
  GuardadoR1();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R1name) + " apagada");
}

void handleControlR1Auto() {
  modoR1 = AUTO;
  GuardadoR1();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R1name) + " en modo automatico");
}

void handleControlR1Timer() {
  modoR1 = TIMER;
  GuardadoR1();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R1name) + " en modo TIMER");
}



// ── R5 ──────────────────────────────────────────────────────────
void handleControlR5On() {
  estadoR5 = 1;
  modoR5   = MANUAL;
  R5estado = LOW;          // ← CRÍTICO
  setRelayCH(RELAY5, true);
  GuardadoR5();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R5name) + " encendida");
}

void handleControlR5Off() {
  estadoR5 = 0;
  modoR5   = MANUAL;
  R5estado = HIGH;         // ← CRÍTICO
  setRelayCH(RELAY5, false);
  GuardadoR5();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R5name) + " apagada");
}

void handleControlR5Auto() {
  modoR5 = AUTO;
  GuardadoR5();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R5name) + " en modo automatico");
}

void handleControlR5Timer() {
  modoR5 = TIMER;
  GuardadoR5();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R5name) + " en modo TIMER");
}



// ── R6 ──────────────────────────────────────────────────────────
void handleControlR6On() {
  estadoR6 = 1;
  modoR6   = MANUAL;
  R6estado = LOW;          // ← CRÍTICO
  setRelayCH(RELAY6, true);
  GuardadoR6();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R6name) + " encendida");
}

void handleControlR6Off() {
  estadoR6 = 0;
  modoR6   = MANUAL;
  R6estado = HIGH;         // ← CRÍTICO
  setRelayCH(RELAY6, false);
  GuardadoR6();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R6name) + " apagada");
}

void handleControlR6Auto() {
  modoR6 = AUTO;
  GuardadoR6();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R6name) + " en modo automatico");
}

void handleControlR6Timer() {
  modoR6 = TIMER;
  GuardadoR6();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R6name) + " en modo TIMER");
}





// ── R2 ──────────────────────────────────────────────────────────
void handleControlR2On() {
  estadoR2 = 1;
  modoR2   = MANUAL;
  R2estado = LOW;          // ← CRÍTICO
  setRelayCH(RELAY2, true);
  GuardadoR2();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R2name) + " encendida");
}

void handleControlR2Off() {
  estadoR2 = 0;
  modoR2   = MANUAL;
  R2estado = HIGH;         // ← CRÍTICO
  setRelayCH(RELAY2, false);
  GuardadoR2();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R2name) + " apagada");
}

void handleControlR2Auto() {
  modoR2 = AUTO;
  GuardadoR2();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R2name) + " en modo automatico");
}




// ── R3 ──────────────────────────────────────────────────────────
void handleControlR3On() {
  estadoR3 = 1;
  modoR3   = MANUAL;
  R3estado = LOW;          // ← CRÍTICO
  setRelayCH(RELAY3, true);
  GuardadoR3();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R3name) + " encendida");
}

void handleControlR3OnFor() {
  if (!server.hasArg("duration")) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(400, "text/plain", "Duracion no especificada");
    return;
  }

  int duration = server.arg("duration").toInt();
  if (duration < 1) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(400, "text/plain", "Duracion invalida");
    return;
  }

  // Responder ANTES del delay para que el browser no haga timeout
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Riego por " + String(duration) + " segundos");

  // Ahora ejecutar el riego
  R3estado = LOW;
  setRelayCH(RELAY3, true);
  
  unsigned long inicio = millis();
  while (millis() - inicio < (unsigned long)duration * 1000UL) {
    server.handleClient();  // seguir atendiendo requests durante el riego
    delay(50);
    yield();
  }
  
  setRelayCH(RELAY3, false);
  R3estado = HIGH;
  modoR3   = MANUAL;
  estadoR3 = 0;
  GuardadoR3();
}

void handleControlR3Off() {
  estadoR3 = 0;
  modoR3   = MANUAL;
  R3estado = HIGH;         // ← CRÍTICO
  setRelayCH(RELAY3, false);
  GuardadoR3();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R3name) + " apagada");
}

void handleControlR3Auto() {
  modoR3 = AUTO;
  GuardadoR3();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R3name) + " en modo automatico");
}

// ── R4 ──────────────────────────────────────────────────────────
void handleControlR4On() {
  estadoR4 = 1;
  modoR4   = MANUAL;
  R4estado = LOW;          // ← CRÍTICO
  setRelayCH(RELAY4, true);
  GuardadoR4();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R4name) + " encendida");
}

void handleControlR4Off() {
  estadoR4 = 0;
  modoR4   = MANUAL;
  R4estado = HIGH;         // ← CRÍTICO
  setRelayCH(RELAY4, false);
  GuardadoR4();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R4name) + " apagada");
}

void handleControlR4Auto() {
  modoR4 = AUTO;
  GuardadoR4();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R4name) + " en modo automatico");
}

void handleControlR4Superciclo() {
  modoR4 = SUPERCICLO;   // valor 4
  GuardadoR4();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R4name) + " en modo Superciclo");
}

// R7 - clon de R4 (Iluminacion)
void handleControlR7On() {
  estadoR7 = 1;
  modoR7   = MANUAL;
  R7estado = LOW;
  setRelayCH(RELAY7, true);
  GuardadoR7();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R7name) + " encendida");
}

void handleControlR7Off() {
  estadoR7 = 0;
  modoR7   = MANUAL;
  R7estado = HIGH;
  setRelayCH(RELAY7, false);
  GuardadoR7();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R7name) + " apagada");
}

void handleControlR7Auto() {
  modoR7 = AUTO;
  GuardadoR7();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R7name) + " en modo automatico");
}

void handleControlR7Superciclo() {
  modoR7 = SUPERCICLO;
  GuardadoR7();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R7name) + " en modo Superciclo");
}

// R8 - clon de R3 (Irrigacion)
void handleControlR8On() {
  estadoR8 = 1;
  modoR8   = MANUAL;
  R8estado = LOW;
  setRelayCH(RELAY8, true);
  GuardadoR8();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R8name) + " encendida");
}

void handleControlR8OnFor() {
  if (!server.hasArg("duration")) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(400, "text/plain", "Duracion no especificada");
    return;
  }

  int duration = server.arg("duration").toInt();
  if (duration < 1) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(400, "text/plain", "Duracion invalida");
    return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Riego R8 por " + String(duration) + " segundos");

  R8estado = LOW;
  setRelayCH(RELAY8, true);

  unsigned long inicio = millis();
  while (millis() - inicio < (unsigned long)duration * 1000UL) {
    server.handleClient();
    delay(50);
    yield();
  }

  setRelayCH(RELAY8, false);
  R8estado = HIGH;
  modoR8   = MANUAL;
  estadoR8 = 0;
  GuardadoR8();
}

void handleControlR8Off() {
  estadoR8 = 0;
  modoR8   = MANUAL;
  R8estado = HIGH;
  setRelayCH(RELAY8, false);
  GuardadoR8();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R8name) + " apagada");
}

void handleControlR8Auto() {
  modoR8 = AUTO;
  GuardadoR8();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R8name) + " en modo automatico");
}




// ==== Helpers PCF85063 -> epoch + dateKey (reemplaza DateTime/RTClib) ====
static uint32_t unixFromPCF(const datetime_t& dt) {
  struct tm t{};
  t.tm_year = (int)dt.year - 1900;
  t.tm_mon  = (int)dt.month - 1;
  t.tm_mday = (int)dt.day;
  t.tm_hour = (int)dt.hour;
  t.tm_min  = (int)dt.minute;
  t.tm_sec  = (int)dt.second;
  t.tm_isdst = 0;
  time_t epoch = timegm_compat(&t);
  return (uint32_t)epoch;
}

static uint32_t localDateKeyFromPCF(const datetime_t& dt) {
  time_t t = (time_t)unixFromPCF(dt);
  t -= 3 * 3600;                      // offset local (AR -03)
  struct tm lt{};
  gmtime_r(&t, &lt);                   // lo tratamos como "UTC" luego del offset
  return (uint32_t)((lt.tm_year + 1900) * 10000 + (lt.tm_mon + 1) * 100 + lt.tm_mday);
}

void handleStartVege() {
  datetime_t now;
  PCF85063_Read_Time(&now);

  vegeStartEpoch = unixFromPCF(now);  // UTC
  vegeActive     = true;
  vegeDays       = 1;

  lastDateKey    = localDateKeyFromPCF(now);

  GuardadoR4();

  String msg = "VEGETATIVO iniciado - Dia 1";

  if (server.hasArg("ajax") || server.header("X-Requested-With") == "XMLHttpRequest") {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain; charset=utf-8", msg);
    return;
  }

  handleConfirmation(msg, "/");
}

void handleStartFlora() {
  datetime_t now;
  PCF85063_Read_Time(&now);

  floraStartEpoch = unixFromPCF(now); // UTC
  floraActive     = true;
  floraDays       = 1;

  lastDateKey     = localDateKeyFromPCF(now);

  GuardadoR4();

  String msg = "FLORACION iniciada - Dia 1";

  if (server.hasArg("ajax") || server.header("X-Requested-With") == "XMLHttpRequest") {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain; charset=utf-8", msg);
    return;
  }

  handleConfirmation(msg, "/");
}

void handleStartFloraSuper() {
  datetime_t now;
  PCF85063_Read_Time(&now);

  floraStartEpoch = unixFromPCF(now); // UTC
  floraActive     = true;
  floraDays       = 1;

  lastDateKey     = localDateKeyFromPCF(now);

  superEnabled    = true;

  GuardadoR4();

  String msg = "FLORACION + SUPERCICLO iniciados - Dia 1";

  if (server.hasArg("ajax") || server.header("X-Requested-With") == "XMLHttpRequest") {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain; charset=utf-8", msg);
    return;
  }

  handleConfirmation(msg, "/");
}

void handleResetVege() {
  vegeStartEpoch = 0;
  vegeActive     = false;
  vegeDays       = 0;

  GuardadoR4();

  String msg = "VEGETATIVO reiniciado a 0";

  if (server.hasArg("ajax") || server.header("X-Requested-With") == "XMLHttpRequest") {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain; charset=utf-8", msg);
    return;
  }

  handleConfirmation(msg, "/");
}

void handleResetFlora() {
  floraStartEpoch = 0;
  floraActive     = false;
  floraDays       = 0;
  superEnabled    = false;

  GuardadoR4();

  String msg = "FLORACION reiniciada a 0";

  if (server.hasArg("ajax") || server.header("X-Requested-With") == "XMLHttpRequest") {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain; charset=utf-8", msg);
    return;
  }

  handleConfirmation(msg, "/");
}









void handleSyncPhoneTime() {
  // Esperamos campos: y mo d h mi s w (w es opcional porque lo recalculamos)
  if (!server.hasArg("y")  || !server.hasArg("mo") || !server.hasArg("d") ||
      !server.hasArg("h")  || !server.hasArg("mi") || !server.hasArg("s")) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"faltan parametros\"}");
    return;
  }

  datetime_t dt;
  dt.year   = (uint16_t) server.arg("y").toInt();
  dt.month  = (uint8_t)  server.arg("mo").toInt();
  dt.day    = (uint8_t)  server.arg("d").toInt();
  dt.hour   = (uint8_t)  server.arg("h").toInt();
  dt.minute = (uint8_t)  server.arg("mi").toInt();
  dt.second = (uint8_t)  server.arg("s").toInt();

  // Si viene w lo tomamos, pero igual lo recalculamos tras el ajuste
  dt.dotw   = server.hasArg("w") ? (uint8_t)server.arg("w").toInt() : 0;

  // Validación rápida (si la tenés en tu proyecto)
  if (!RTC_TimeLooksReasonable(dt)) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"hora no razonable\"}");
    return;
  }

  // ---------------------------------------------------------
  // AJUSTE HORARIO (Argentina UTC-3)
  // En tu test te quedó 3h atrasado, así que corregimos +3.
  // ---------------------------------------------------------
  const int TZ_HOURS_CORRECTION = +3;  // <-- corrige el atraso de 3 horas

  struct tm t;
  t.tm_year  = (int)dt.year - 1900;
  t.tm_mon   = (int)dt.month - 1;
  t.tm_mday  = (int)dt.day;
  t.tm_hour  = (int)dt.hour;
  t.tm_min   = (int)dt.minute;
  t.tm_sec   = (int)dt.second;
  t.tm_isdst = -1;

  time_t epoch = mktime(&t);
  if (epoch < 0) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"mktime fallo\"}");
    return;
  }

  epoch += (time_t)TZ_HOURS_CORRECTION * 3600;

  struct tm out;
  localtime_r(&epoch, &out);

  dt.year   = (uint16_t)(out.tm_year + 1900);
  dt.month  = (uint8_t) (out.tm_mon + 1);
  dt.day    = (uint8_t)  out.tm_mday;
  dt.hour   = (uint8_t)  out.tm_hour;
  dt.minute = (uint8_t)  out.tm_min;
  dt.second = (uint8_t)  out.tm_sec;
  dt.dotw   = (uint8_t)  out.tm_wday; // 0=Domingo..6=Sábado

  // Re-validamos por seguridad
  if (!RTC_TimeLooksReasonable(dt)) {
    server.send(400, "application/json", "{\"ok\":false,\"msg\":\"hora ajustada invalida\"}");
    return;
  }

  // Set RTC
  PCF85063_Set_All(dt);

  // Refrescar estado/hora-minuto desde RTC recién ajustado
  RTC_UpdateStatusAndHM();

  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"RTC sincronizado\"}");
}




void connectWiFi() {
  modoWiFi = 1;
  reset    = 1;
  GuardadoConfig();

  if (server.hasArg("ajax") || server.header("X-Requested-With") == "XMLHttpRequest") {
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "text/plain", "Conectando a WiFi. El equipo se reiniciará.");
    return;
  }

  server.sendHeader("Location", "/");
  server.send(303);
}


void saveConfigWiFi() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }

  if (server.hasArg("ssid")) {
    ssid = server.arg("ssid");
  }
  if (server.hasArg("password")) {
    password = server.arg("password");
  }
  if (server.hasArg("chat_id")) {
    chat_id = server.arg("chat_id");
  }
  if (server.hasArg("tiempoGoogle")) {
    tiempoGoogle = server.arg("tiempoGoogle").toInt();
  }
  if (server.hasArg("tiempoTelegram")) {
    tiempoTelegram = server.arg("tiempoTelegram").toInt();
  }
  if (server.hasArg("modoWiFi")) {
    modoWiFi = server.arg("modoWiFi").toInt();
  }
  GuardadoConfig();

  

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Configuración guardada");
}


String formatTwoDigits(int number) {
    if (number < 10) {
        return "0" + String(number);
    }
    return String(number);
}

// Cambios en handleConfig




// ⬇️ Pega esta versión completa de handleConfigR5 (mismos nombres de variables y rutas genéricas)
// Ajusta SOLO las rutas de Encender/Apagar si usas otras (ej: /controlR5On y /controlR5Off)




void saveConfigR2() {

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }

  if (server.hasArg("modoR2"))  modoR2  = server.arg("modoR2").toInt();
  if (server.hasArg("minR2"))   minR2   = server.arg("minR2").toFloat();
  if (server.hasArg("maxR2"))   maxR2   = server.arg("maxR2").toFloat();
  if (server.hasArg("minTR2"))  minTR2  = server.arg("minTR2").toFloat();
  if (server.hasArg("maxTR2"))  maxTR2  = server.arg("maxTR2").toFloat();
  if (server.hasArg("paramR2")) paramR2 = server.arg("paramR2").toInt();
  if (server.hasArg("estadoR2")) estadoR2 = server.arg("estadoR2").toInt();
  if (server.hasArg("R2name"))  R2name  = server.arg("R2name").toInt();

  GuardadoR2();

  // 🚨 SIEMPRE RESPUESTA AJAX
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R2name) + " configurada");
}




// ⬇️ Reemplazá por completo tu handleConfigR2.
// Muestra SOLO lo que corresponde según modoR2 y paramR2 (HT=4 destapa MIN/MAX TEMP).
// Mantengo tus estilos, sliders, +/- y validaciones.






void saveConfigR3() {

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }

  if (server.hasArg("modoR3")) {
    modoR3 = server.arg("modoR3").toInt();
  }
  if (server.hasArg("sensorR3")) {
    sensorR3 = normalizarCodigoSensoresSuelo(server.arg("sensorR3").toInt());
  }
  if (server.hasArg("minR3")) {
    minR3 = server.arg("minR3").toInt();
  }
  if (server.hasArg("maxR3")) {
    maxR3 = server.arg("maxR3").toInt();
  }

  if (server.hasArg("horaOnR3")) {
    String horaOn = server.arg("horaOnR3");
    int sepIndex = horaOn.indexOf(':');
    if (sepIndex != -1) {
      horaOnR3 = horaOn.substring(0, sepIndex).toInt();
      minOnR3  = horaOn.substring(sepIndex + 1).toInt();
    }
  }

  if (server.hasArg("horaOffR3")) {
    String horaOff = server.arg("horaOffR3");
    int sepIndex = horaOff.indexOf(':');
    if (sepIndex != -1) {
      horaOffR3 = horaOff.substring(0, sepIndex).toInt();
      minOffR3  = horaOff.substring(sepIndex + 1).toInt();
    }
  }

  unidadRiego   = server.hasArg("unidadRiego")   ? server.arg("unidadRiego").toInt()   : 1;
  unidadNoRiego = server.hasArg("unidadNoRiego") ? server.arg("unidadNoRiego").toInt() : 1;

  tiempoRiego   = server.hasArg("tiempoRiego")   ? round(server.arg("tiempoRiego").toFloat() * unidadRiego)   : 0;
  tiempoNoRiego = server.hasArg("tiempoNoRiego") ? round(server.arg("tiempoNoRiego").toFloat() * unidadNoRiego) : 0;

  int cicloTotal = tiempoRiego + tiempoNoRiego;
  int maxCantidad = (cicloTotal > 0) ? 86400 / cicloTotal : 1;

  if (server.hasArg("cantidad")) {
    cantidadRiegos = server.arg("cantidad").toInt();
  } else {
    int segundosOn  = horaOnR3 * 3600 + minOnR3 * 60;
    int segundosOff = horaOffR3 * 3600 + minOffR3 * 60;
    int duracion = segundosOff - segundosOn;
    if (duracion < 0) duracion += 86400;

    cantidadRiegos = (cicloTotal > 0) ? duracion / cicloTotal : 1;
  }

  if (cantidadRiegos < 1) cantidadRiegos = 1;
  if (cantidadRiegos > maxCantidad) cantidadRiegos = maxCantidad;

  if (server.hasArg("estadoR3")) {
    estadoR3 = server.arg("estadoR3").toInt();
  }

  if (server.hasArg("R3name")) {
    R3name = server.arg("R3name").toInt();
  }

  for (int i = 0; i < 7; i++) diasRiego[i] = 0;
  for (int i = 0; i < 7; i++) {
    String paramName = "diaRiego" + String(i);
    if (server.hasArg(paramName)) {
      diasRiego[i] = 1;
    }
  }

  GuardadoR3();

  // 🚨 SIEMPRE RESPUESTA AJAX
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R3name) + " configurada");
}




void saveConfigR4() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }

  // ===== 1) Lectura =====
  if (server.hasArg("modoR4")) modoR4 = server.arg("modoR4").toInt();
  if (server.hasArg("R4name")) R4name = server.arg("R4name").toInt();

  if (server.hasArg("horaOnR4")) {
    int onMin = parseHHMMToMinutesSafe(server.arg("horaOnR4"));
    if (onMin >= 0) { horaOnR4 = onMin / 60; minOnR4 = onMin % 60; }
  }

  if (modoR4 != SUPERCICLO && server.hasArg("horaOffR4")) {
    int offMin = parseHHMMToMinutesSafe(server.arg("horaOffR4"));
    if (offMin >= 0) { horaOffR4 = offMin / 60; minOffR4 = offMin % 60; }
  }

  if (server.hasArg("horaAmanecer")) {
    int am = parseHHMMToMinutesSafe(server.arg("horaAmanecer"));
    if (am >= 0) horaAmanecer = am;
  }

  if (server.hasArg("horaAtardecer")) {
    int at = parseHHMMToMinutesSafe(server.arg("horaAtardecer"));
    if (at >= 0) horaAtardecer = at;
  }

  if (server.hasArg("horasLuz")) {
    int luz = parseHHMMToMinutesSafe(server.arg("horasLuz"));
    if (luz >= 0) horasLuz = luz;
  }

  if (server.hasArg("horasOscuridad")) {
    int osc = parseHHMMToMinutesSafe(server.arg("horasOscuridad"));
    if (osc >= 0) horasOscuridad = osc;
  }

  // ===== 2) Normalizaciones =====
  horaOnR4  = constrain(horaOnR4,  0, 23);
  minOnR4   = constrain(minOnR4,   0, 59);
  horaOffR4 = constrain(horaOffR4, 0, 23);
  minOffR4  = constrain(minOffR4,  0, 59);

  long periodo = (long)horasLuz + (long)horasOscuridad;
  if (modoR4 == SUPERCICLO && periodo <= 0) {
    horasLuz = 12 * 60;
    horasOscuridad = 12 * 60;
  }

  // ===== 3) SUPERCICLO =====
  if (modoR4 == SUPERCICLO) {
    int Lm = (int)horasLuz;
    if (Lm <= 0) Lm = 1;

    int onAbs  = (horaOnR4 * 60 + minOnR4) % 1440;
    int offAbs = (onAbs + Lm) % 1440;

    horaOffR4 = offAbs / 60;
    minOffR4  = offAbs % 60;

    nextOnR4Abs  = onAbs;
    nextOffR4Abs = offAbs;
  }

  // ===== 4) Guardado + respuesta AJAX =====
  GuardadoR4();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R4name) + " configurada");
}




void handleSaveConfig() {
    // Guardar configuraciones (ya definido en tu código)
    Guardado_General();

    // Mostrar mensaje de confirmación
    handleConfirmation("Configuracion guardada correcta mente", "/config");
}

void saveConfigR1() {

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }

  if (server.hasArg("modoR1")) modoR1 = server.arg("modoR1").toInt();
  if (server.hasArg("minR1")) minR1 = server.arg("minR1").toFloat();
  if (server.hasArg("maxR1")) maxR1 = server.arg("maxR1").toFloat();
  if (server.hasArg("paramR1")) paramR1 = server.arg("paramR1").toInt();
  if (server.hasArg("direccionR1")) direccionR1 = server.arg("direccionR1").toInt();
  if (server.hasArg("sensorR1")) sensorR1 = normalizarCodigoSensoresAmbiente(server.arg("sensorR1").toInt());

  if (server.hasArg("horaOnR1")) {
    String h = server.arg("horaOnR1");
    int sep = h.indexOf(':');
    if (sep > 0) {
      horaOnR1 = h.substring(0, sep).toInt();
      minOnR1  = h.substring(sep + 1).toInt();
    }
  }

  if (server.hasArg("horaOffR1")) {
    String h = server.arg("horaOffR1");
    int sep = h.indexOf(':');
    if (sep > 0) {
      horaOffR1 = h.substring(0, sep).toInt();
      minOffR1  = h.substring(sep + 1).toInt();
    }
  }

  if (server.hasArg("estadoR1")) estadoR1 = server.arg("estadoR1").toInt();
  if (server.hasArg("R1name")) R1name = server.arg("R1name").toInt();

  GuardadoR1();

  // 🚨 SIEMPRE RESPUESTA AJAX
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R1name) + " configurada");
}

void saveConfigR5() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }

  if (server.hasArg("modoR5")) {
    modoR5 = server.arg("modoR5").toInt();
  }
  if (server.hasArg("minR5")) {
    minR5 = server.arg("minR5").toFloat();
  }
  if (server.hasArg("maxR5")) {
    maxR5 = server.arg("maxR5").toFloat();
  }
  if (server.hasArg("paramR5")) {
    paramR5 = server.arg("paramR5").toInt();
  }
  if (server.hasArg("sensorR5")) {
    sensorR5 = normalizarCodigoSensoresAmbiente(server.arg("sensorR5").toInt());
  }
  if (server.hasArg("direccionR5")) {
    direccionR5 = server.arg("direccionR5").toInt();
  }

  if (server.hasArg("horaOnR5")) {
    String horaOn = server.arg("horaOnR5");
    int sep = horaOn.indexOf(':');
    if (sep > 0) {
      horaOnR5 = horaOn.substring(0, sep).toInt();
      minOnR5  = horaOn.substring(sep + 1).toInt();
    }
  }

  if (server.hasArg("horaOffR5")) {
    String horaOff = server.arg("horaOffR5");
    int sep = horaOff.indexOf(':');
    if (sep > 0) {
      horaOffR5 = horaOff.substring(0, sep).toInt();
      minOffR5  = horaOff.substring(sep + 1).toInt();
    }
  }

  if (server.hasArg("estadoR5")) {
    estadoR5 = server.arg("estadoR5").toInt();
  }

  if (server.hasArg("R5name")) {
    R5name = server.arg("R5name").toInt();
  }

  GuardadoR5();

  // 🚨 SIEMPRE RESPUESTA AJAX
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R5name) + " configurada");
}

void saveConfigR6() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Método no permitido");
    return;
  }

  if (server.hasArg("modoR6")) {
    modoR6 = server.arg("modoR6").toInt();
  }
  if (server.hasArg("minR6")) {
    minR6 = server.arg("minR6").toFloat();
  }
  if (server.hasArg("maxR6")) {
    maxR6 = server.arg("maxR6").toFloat();
  }
  if (server.hasArg("paramR6")) {
    paramR6 = server.arg("paramR6").toInt();
  }
  if (server.hasArg("sensorR6")) {
    sensorR6 = normalizarCodigoSensoresAmbiente(server.arg("sensorR6").toInt());
  }
  if (server.hasArg("direccionR6")) {
    direccionR6 = server.arg("direccionR6").toInt();
  }

  if (server.hasArg("horaOnR6")) {
    String horaOn = server.arg("horaOnR6");
    int sep = horaOn.indexOf(':');
    if (sep > 0) {
      horaOnR6 = horaOn.substring(0, sep).toInt();
      minOnR6  = horaOn.substring(sep + 1).toInt();
    }
  }

  if (server.hasArg("horaOffR6")) {
    String horaOff = server.arg("horaOffR6");
    int sep = horaOff.indexOf(':');
    if (sep > 0) {
      horaOffR6 = horaOff.substring(0, sep).toInt();
      minOffR6  = horaOff.substring(sep + 1).toInt();
    }
  }

  if (server.hasArg("estadoR6")) {
    estadoR6 = server.arg("estadoR6").toInt();
  }

  if (server.hasArg("R6name")) {
    R6name = server.arg("R6name").toInt();
  }

  GuardadoR6();

  // 🚨 SIEMPRE RESPUESTA AJAX
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R6name) + " configurada");
}

void saveConfigR7() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Metodo no permitido");
    return;
  }

  if (server.hasArg("modoR7")) modoR7 = server.arg("modoR7").toInt();
  if (server.hasArg("R7name")) R7name = server.arg("R7name").toInt();

  if (server.hasArg("horaOnR7")) {
    int onMin = parseHHMMToMinutesSafe(server.arg("horaOnR7"));
    if (onMin >= 0) { horaOnR7 = onMin / 60; minOnR7 = onMin % 60; }
  }

  if (modoR7 != SUPERCICLO && server.hasArg("horaOffR7")) {
    int offMin = parseHHMMToMinutesSafe(server.arg("horaOffR7"));
    if (offMin >= 0) { horaOffR7 = offMin / 60; minOffR7 = offMin % 60; }
  }

  if (server.hasArg("horaAmanecerR7")) {
    int am = parseHHMMToMinutesSafe(server.arg("horaAmanecerR7"));
    if (am >= 0) horaAmanecerR7 = am;
  }

  if (server.hasArg("horaAtardecerR7")) {
    int at = parseHHMMToMinutesSafe(server.arg("horaAtardecerR7"));
    if (at >= 0) horaAtardecerR7 = at;
  }

  if (server.hasArg("horasLuzR7") || server.hasArg("horasLuz")) {
    int luz = parseHHMMToMinutesSafe(server.hasArg("horasLuzR7") ? server.arg("horasLuzR7") : server.arg("horasLuz"));
    if (luz >= 0) horasLuzR7 = luz;
  }

  if (server.hasArg("horasOscuridadR7") || server.hasArg("horasOscuridad")) {
    int osc = parseHHMMToMinutesSafe(server.hasArg("horasOscuridadR7") ? server.arg("horasOscuridadR7") : server.arg("horasOscuridad"));
    if (osc >= 0) horasOscuridadR7 = osc;
  }

  horaOnR7  = constrain(horaOnR7,  0, 23);
  minOnR7   = constrain(minOnR7,   0, 59);
  horaOffR7 = constrain(horaOffR7, 0, 23);
  minOffR7  = constrain(minOffR7,  0, 59);

  long periodo = (long)horasLuzR7 + (long)horasOscuridadR7;
  if (modoR7 == SUPERCICLO && periodo <= 0) {
    horasLuzR7 = 12 * 60;
    horasOscuridadR7 = 12 * 60;
  }

  if (modoR7 == SUPERCICLO) {
    int Lm = (int)horasLuzR7;
    if (Lm <= 0) Lm = 1;

    int onAbs  = (horaOnR7 * 60 + minOnR7) % 1440;
    int offAbs = (onAbs + Lm) % 1440;

    horaOffR7 = offAbs / 60;
    minOffR7  = offAbs % 60;

    nextOnR7Abs  = onAbs;
    nextOffR7Abs = offAbs;
  }

  GuardadoR7();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R7name) + " configurada");
}

void saveConfigR8() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Metodo no permitido");
    return;
  }

  if (server.hasArg("modoR8")) modoR8 = server.arg("modoR8").toInt();
  if (server.hasArg("sensorR8")) sensorR8 = normalizarCodigoSensoresSuelo(server.arg("sensorR8").toInt());
  if (server.hasArg("minR8")) minR8 = server.arg("minR8").toInt();
  if (server.hasArg("maxR8")) maxR8 = server.arg("maxR8").toInt();

  if (server.hasArg("horaOnR8")) {
    String horaOn = server.arg("horaOnR8");
    int sepIndex = horaOn.indexOf(':');
    if (sepIndex != -1) {
      horaOnR8 = horaOn.substring(0, sepIndex).toInt();
      minOnR8  = horaOn.substring(sepIndex + 1).toInt();
    }
  }

  if (server.hasArg("horaOffR8")) {
    String horaOff = server.arg("horaOffR8");
    int sepIndex = horaOff.indexOf(':');
    if (sepIndex != -1) {
      horaOffR8 = horaOff.substring(0, sepIndex).toInt();
      minOffR8  = horaOff.substring(sepIndex + 1).toInt();
    }
  }

  unidadRiegoR8 = server.hasArg("unidadRiegoR8")
    ? server.arg("unidadRiegoR8").toInt()
    : (server.hasArg("unidadRiego") ? server.arg("unidadRiego").toInt() : 1);
  unidadNoRiegoR8 = server.hasArg("unidadNoRiegoR8")
    ? server.arg("unidadNoRiegoR8").toInt()
    : (server.hasArg("unidadNoRiego") ? server.arg("unidadNoRiego").toInt() : 1);

  tiempoRiegoR8 = server.hasArg("tiempoRiegoR8")
    ? round(server.arg("tiempoRiegoR8").toFloat() * unidadRiegoR8)
    : (server.hasArg("tiempoRiego") ? round(server.arg("tiempoRiego").toFloat() * unidadRiegoR8) : 0);
  tiempoNoRiegoR8 = server.hasArg("tiempoNoRiegoR8")
    ? round(server.arg("tiempoNoRiegoR8").toFloat() * unidadNoRiegoR8)
    : (server.hasArg("tiempoNoRiego") ? round(server.arg("tiempoNoRiego").toFloat() * unidadNoRiegoR8) : 0);

  int cicloTotal = tiempoRiegoR8 + tiempoNoRiegoR8;
  int maxCantidad = (cicloTotal > 0) ? 86400 / cicloTotal : 1;

  if (server.hasArg("cantidadR8") || server.hasArg("cantidad")) {
    cantidadRiegosR8 = server.hasArg("cantidadR8") ? server.arg("cantidadR8").toInt() : server.arg("cantidad").toInt();
  } else {
    int segundosOn  = horaOnR8 * 3600 + minOnR8 * 60;
    int segundosOff = horaOffR8 * 3600 + minOffR8 * 60;
    int duracion = segundosOff - segundosOn;
    if (duracion < 0) duracion += 86400;

    cantidadRiegosR8 = (cicloTotal > 0) ? duracion / cicloTotal : 1;
  }

  if (cantidadRiegosR8 < 1) cantidadRiegosR8 = 1;
  if (cantidadRiegosR8 > maxCantidad) cantidadRiegosR8 = maxCantidad;

  if (server.hasArg("estadoR8")) estadoR8 = server.arg("estadoR8").toInt();
  if (server.hasArg("R8name")) R8name = server.arg("R8name").toInt();

  for (int i = 0; i < 7; i++) diasRiegoR8[i] = 0;
  for (int i = 0; i < 7; i++) {
    String paramName = "diaRiegoR8" + String(i);
    String genericParamName = "diaRiego" + String(i);
    if (server.hasArg(paramName) || server.hasArg(genericParamName)) {
      diasRiegoR8[i] = 1;
    }
  }

  GuardadoR8();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", getRelayName(R8name) + " configurada");
}

// =====================================================
// API NUEVA PARA FRONTEND DRUIDA 3.0
// =====================================================

String modeKindFromModo(int modo) {
  if (modo == MANUAL) return "manual";
  if (modo == TIMER)  return "timer";
  return "auto";
}

String hhmmFromParts(int h, int m) {
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", h, m);
  return String(buf);
}

int parseHHMMToMinutesApi(const String& s) {
  int sep = s.indexOf(':');
  if (sep < 0) return -1;

  int h = s.substring(0, sep).toInt();
  int m = s.substring(sep + 1).toInt();

  if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
  return h * 60 + m;
}

void handleApiInfo() {
  String json = "{";
  json += "\"version\":\"3.0\",";
  json += "\"plan\":\"Druida BOT\",";
  json += "\"uptime\":\"" + String(millis() / 1000UL) + " s\",";
  json += "\"ssid\":\"" + ssid + "\"";
  json += "}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleSaveWiFi() {
  if (server.hasArg("ssid")) {
    ssid = server.arg("ssid");
  }

  if (server.hasArg("pass")) {
    password = server.arg("pass");
  }

  if (server.hasArg("password")) {
    password = server.arg("password");
  }

  GuardadoConfig();

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "WiFi guardado");
}

void handleApiAlerts() {
  // Por ahora queda como endpoint compatible.
  // Más adelante se puede conectar con Telegram o EEPROM.
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Alertas guardadas");
}

void handleApiSensors() {
  String json = "{\"sensors\":[";

  json += "{\"id\":\"s1\",\"name\":\"Ambiente 1\",\"type\":\"ambient\",\"addr\":1,\"enabled\":" + jsonBool(sensorAir1Activo) + ",\"connected\":" + jsonBool(sensorAmbOK1);
  json += ",\"temp\":" + String(temperatura1, 1);
  json += ",\"hum\":" + String(humedad1, 1);
  json += ",\"dpv\":" + String(calcularDPV(temperatura1, humedad1), 1);
  json += "}";

  json += ",{\"id\":\"s2\",\"name\":\"Ambiente 2\",\"type\":\"ambient\",\"addr\":2,\"enabled\":" + jsonBool(sensorAir2Activo) + ",\"connected\":" + jsonBool(sensorAmbOK2);
  json += ",\"temp\":" + jsonFlt(temperatura2, 1) + ",\"hum\":" + jsonFlt(humedad2, 1) + ",\"dpv\":" + jsonFlt(calcularDPV(temperatura2, humedad2), 1) + "}";

  json += ",{\"id\":\"s3\",\"name\":\"Ambiente 3\",\"type\":\"ambient\",\"addr\":3,\"enabled\":" + jsonBool(sensorAir3Activo) + ",\"connected\":" + jsonBool(sensorAmbOK3);
  json += ",\"temp\":" + jsonFlt(temperatura3, 1) + ",\"hum\":" + jsonFlt(humedad3, 1) + ",\"dpv\":" + jsonFlt(calcularDPV(temperatura3, humedad3), 1) + "}";

  json += ",{\"id\":\"s4\",\"name\":\"Ambiente 4\",\"type\":\"ambient\",\"addr\":4,\"enabled\":" + jsonBool(sensorAir4Activo) + ",\"connected\":" + jsonBool(sensorAmbOK4);
  json += ",\"temp\":" + jsonFlt(temperatura4, 1) + ",\"hum\":" + jsonFlt(humedad4, 1) + ",\"dpv\":" + jsonFlt(calcularDPV(temperatura4, humedad4), 1) + "}";

  json += ",{\"id\":\"s5\",\"name\":\"Suelo 1\",\"type\":\"soil\",\"addr\":5,\"enabled\":" + jsonBool(sensorSoil5Activo) + ",\"connected\":" + jsonBool(sensorSueloOK5);
  json += ",\"temp\":" + jsonFlt(temperaturaSuelo5, 1) + ",\"hum\":" + jsonFlt(humedadSuelo5, 1) + ",\"ec\":" + jsonFlt(ECSuelo5, 0) + "}";

  json += ",{\"id\":\"s6\",\"name\":\"Suelo 2\",\"type\":\"soil\",\"addr\":6,\"enabled\":" + jsonBool(sensorSoil6Activo) + ",\"connected\":" + jsonBool(sensorSueloOK6);
  json += ",\"temp\":" + jsonFlt(temperaturaSuelo6, 1) + ",\"hum\":" + jsonFlt(humedadSuelo6, 1) + ",\"ec\":" + jsonFlt(ECSuelo6, 0) + "}";

  json += "],\"relaySensors\":{";
  json += "\"r1\":" + String(sensorR1) + ",\"r2\":" + String(sensorR2) + ",\"r3\":" + String(sensorR3);
  json += ",\"r5\":" + String(sensorR5) + ",\"r6\":" + String(sensorR6) + ",\"r8\":" + String(sensorR8);
  json += "}}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleApiSensorSave() {
  if (server.hasArg("relay")) {
    int relay = parseIntFlexible(server.arg("relay"));
    int code = server.hasArg("code") ? parseIntFlexible(server.arg("code"))
             : (server.hasArg("sensor") ? parseIntFlexible(server.arg("sensor")) : 0);

    switch (relay) {
      case 1: sensorR1 = normalizarCodigoSensoresAmbiente(code); break;
      case 2: sensorR2 = normalizarCodigoSensoresAmbiente(code); break;
      case 3: sensorR3 = normalizarCodigoSensoresSuelo(code); break;
      case 5: sensorR5 = normalizarCodigoSensoresAmbiente(code); break;
      case 6: sensorR6 = normalizarCodigoSensoresAmbiente(code); break;
      case 8: sensorR8 = normalizarCodigoSensoresSuelo(code); break;
      default:
        server.send(400, "text/plain", "Relay sin sensor configurable");
        return;
    }
  }

  if (server.hasArg("sensor") && server.hasArg("enabled") && !server.hasArg("relay")) {
    int id = parseIntFlexible(server.arg("sensor"));
    bool enabled = (server.arg("enabled") == "1" || server.arg("enabled") == "true");

    if (id == 1) sensorAir1Activo = enabled;
    else if (id == 2) sensorAir2Activo = enabled;
    else if (id == 3) sensorAir3Activo = enabled;
    else if (id == 4) sensorAir4Activo = enabled;
    else if (id == 5) sensorSoil5Activo = enabled;
    else if (id == 6) sensorSoil6Activo = enabled;
  }

  GuardadoSensoresConfig();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Sensor guardado");
}

void handleApiParams() {
  if (server.hasArg("tempMin")) {
    minR5 = server.arg("tempMin").toFloat();
    minR6 = server.arg("tempMin").toFloat();
  }

  if (server.hasArg("tempMax")) {
    maxR5 = server.arg("tempMax").toFloat();
    maxR6 = server.arg("tempMax").toFloat();
  }

  if (server.hasArg("humMin")) {
    minR1 = server.arg("humMin").toFloat();
  }

  if (server.hasArg("humMax")) {
    maxR2 = server.arg("humMax").toFloat();
  }

  GuardadoR1();
  GuardadoR2();
  GuardadoR5();
  GuardadoR6();

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "Parámetros guardados");
}

void handleApiRelay() {
  if (!server.hasArg("n")) {
    server.send(400, "text/plain", "Falta n");
    return;
  }

  int n = parseIntFlexible(server.arg("n"));

  String mode = server.hasArg("mode") ? server.arg("mode") : "auto";
  bool on = server.hasArg("on") && server.arg("on") == "1";

  String onTime  = server.hasArg("onTime")  ? server.arg("onTime")  : "";
  String offTime = server.hasArg("offTime") ? server.arg("offTime") : "";

  int modoNuevo = AUTO;
  if (mode == "manual") modoNuevo = MANUAL;
  if (mode == "timer")  modoNuevo = TIMER;
  if (mode == "auto")   modoNuevo = AUTO;

  int onMin  = parseHHMMToMinutesApi(onTime);
  int offMin = parseHHMMToMinutesApi(offTime);
  int sensorCode = server.hasArg("sensorCode") ? parseIntFlexible(server.arg("sensorCode"))
                 : (server.hasArg("sensor") ? parseIntFlexible(server.arg("sensor")) : 0);

  switch (n) {
    case 1:
      modoR1 = modoNuevo;
      if (sensorCode > 0) sensorR1 = normalizarCodigoSensoresAmbiente(sensorCode);
      if (onMin >= 0)  { horaOnR1 = onMin / 60;   minOnR1 = onMin % 60; }
      if (offMin >= 0) { horaOffR1 = offMin / 60; minOffR1 = offMin % 60; }

      if (modoR1 == MANUAL) {
        estadoR1 = on ? 1 : 0;
        R1estado = on ? LOW : HIGH;
        setRelayCH(RELAY1, on);
      }

      GuardadoR1();
      break;

    case 2:
      modoR2 = (modoNuevo == TIMER) ? AUTO : modoNuevo;
      if (sensorCode > 0) sensorR2 = normalizarCodigoSensoresAmbiente(sensorCode);

      if (modoR2 == MANUAL) {
        estadoR2 = on ? 1 : 0;
        R2estado = on ? LOW : HIGH;
        setRelayCH(RELAY2, on);
      }

      GuardadoR2();
      break;

    case 3:
      modoR3 = (modoNuevo == TIMER) ? AUTO : modoNuevo;
      if (sensorCode > 0) sensorR3 = normalizarCodigoSensoresSuelo(sensorCode);
      if (server.hasArg("soilMin")) minR3 = server.arg("soilMin").toInt();
      if (server.hasArg("soilMax")) maxR3 = server.arg("soilMax").toInt();
      if (onMin >= 0)  { horaOnR3 = onMin / 60;   minOnR3 = onMin % 60; }
      if (offMin >= 0) { horaOffR3 = offMin / 60; minOffR3 = offMin % 60; }

      if (modoR3 == MANUAL) {
        estadoR3 = on ? 1 : 0;
        R3estado = on ? LOW : HIGH;
        setRelayCH(RELAY3, on);
      }

      GuardadoR3();
      break;

    case 4:
      modoR4 = modoNuevo;
      if (onMin >= 0)  { horaOnR4 = onMin / 60;   minOnR4 = onMin % 60; }
      if (offMin >= 0) { horaOffR4 = offMin / 60; minOffR4 = offMin % 60; }

      if (modoR4 == MANUAL) {
        estadoR4 = on ? 1 : 0;
        R4estado = on ? LOW : HIGH;
        setRelayCH(RELAY4, on);
      }

      GuardadoR4();
      break;

    case 5:
      modoR5 = modoNuevo;
      if (sensorCode > 0) sensorR5 = normalizarCodigoSensoresAmbiente(sensorCode);
      if (onMin >= 0)  { horaOnR5 = onMin / 60;   minOnR5 = onMin % 60; }
      if (offMin >= 0) { horaOffR5 = offMin / 60; minOffR5 = offMin % 60; }

      if (modoR5 == MANUAL) {
        estadoR5 = on ? 1 : 0;
        R5estado = on ? LOW : HIGH;
        setRelayCH(RELAY5, on);
      }

      GuardadoR5();
      break;

    case 6:
      modoR6 = modoNuevo;
      if (sensorCode > 0) sensorR6 = normalizarCodigoSensoresAmbiente(sensorCode);
      if (onMin >= 0)  { horaOnR6 = onMin / 60;   minOnR6 = onMin % 60; }
      if (offMin >= 0) { horaOffR6 = offMin / 60; minOffR6 = offMin % 60; }

      if (modoR6 == MANUAL) {
        estadoR6 = on ? 1 : 0;
        R6estado = on ? LOW : HIGH;
        setRelayCH(RELAY6, on);
      }

      GuardadoR6();
      break;

    case 7:
      modoR7 = modoNuevo;
      if (onMin >= 0)  { horaOnR7 = onMin / 60;   minOnR7 = onMin % 60; }
      if (offMin >= 0) { horaOffR7 = offMin / 60; minOffR7 = offMin % 60; }

      if (modoR7 == MANUAL) {
        estadoR7 = on ? 1 : 0;
        R7estado = on ? LOW : HIGH;
        setRelayCH(RELAY7, on);
      }

      GuardadoR7();
      break;

    case 8:
      modoR8 = (modoNuevo == TIMER) ? AUTO : modoNuevo;
      if (sensorCode > 0) sensorR8 = normalizarCodigoSensoresSuelo(sensorCode);
      if (server.hasArg("soilMin")) minR8 = server.arg("soilMin").toInt();
      if (server.hasArg("soilMax")) maxR8 = server.arg("soilMax").toInt();
      if (onMin >= 0)  { horaOnR8 = onMin / 60;   minOnR8 = onMin % 60; }
      if (offMin >= 0) { horaOffR8 = offMin / 60; minOffR8 = offMin % 60; }

      if (modoR8 == MANUAL) {
        estadoR8 = on ? 1 : 0;
        R8estado = on ? LOW : HIGH;
        setRelayCH(RELAY8, on);
      }

      GuardadoR8();
      break;

    default:
      server.send(400, "text/plain", "Relé inválido");
      return;
  }

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "OK");
}

// FIN DE LA WEB APP
// HASTA ACA



// Mensaje de confirmación



// Función para solicitar datos al Arduino Nano (I2C)
// Función para solicitar datos al Arduino Nano (I2C)



static const char* errToStr(uint8_t e){
  switch(e){
    case 0: return "OK";
    case 1: return "TIME";
    case 2: return "CRC";
    case 3: return "EXC";
    case 4: return "FMT";
    case 5: return "ID";
    default: return "?";
  }
}

static void printHex8(uint8_t y, const uint8_t* buf, uint8_t n){
  display.setCursor(0, y);
  display.print("RX:");
  if (!buf || n == 0) { display.print("--"); return; }
  uint8_t m = (n < 8) ? n : 8;
  for(uint8_t i=0;i<m;i++){
    display.print(" ");
    uint8_t b = buf[i];
    if (b < 0x10) display.print("0");
    display.print(b, HEX);
  }
  if (n > 8) display.print("..");
}

static bool airSensorValido(uint8_t id, float &t, float &h) {
  switch (id) {
    case 1:
      t = temperatura1; h = humedad1;
      return sensorAmbOK1 && isfinite(t) && isfinite(h) &&
             t > -40.0f && t < 85.0f &&
             h >= 0.0f && h <= 100.0f;

    case 2:
      t = temperatura2; h = humedad2;
      return sensorAmbOK2 && isfinite(t) && isfinite(h) &&
             t > -40.0f && t < 85.0f &&
             h >= 0.0f && h <= 100.0f;

    case 3:
      t = temperatura3; h = humedad3;
      return sensorAmbOK3 && isfinite(t) && isfinite(h) &&
             t > -40.0f && t < 85.0f &&
             h >= 0.0f && h <= 100.0f;

    case 4:
      t = temperatura4; h = humedad4;
      return sensorAmbOK4 && isfinite(t) && isfinite(h) &&
             t > -40.0f && t < 85.0f &&
             h >= 0.0f && h <= 100.0f;
  }

  t = NAN; h = NAN;
  return false;
}

static bool soilSensorValido(uint8_t id, float &t, float &h, float &ec) {
  switch (id) {
    case 5:
      t = temperaturaSuelo5; h = humedadSuelo5; ec = ECSuelo5;
      return sensorSueloOK5 && isfinite(t) && isfinite(h) && isfinite(ec) &&
             t > -40.0f && t < 85.0f &&
             h >= 0.0f && h <= 100.0f &&
             ec >= 0.0f && ec <= 20000.0f;

    case 6:
      t = temperaturaSuelo6; h = humedadSuelo6; ec = ECSuelo6;
      return sensorSueloOK6 && isfinite(t) && isfinite(h) && isfinite(ec) &&
             t > -40.0f && t < 85.0f &&
             h >= 0.0f && h <= 100.0f &&
             ec >= 0.0f && ec <= 20000.0f;
  }

  t = NAN; h = NAN; ec = NAN;
  return false;
}

void mostrarEnPantallaOLED(float temperature, float humedad, float DPV,
                           float soilTemp, float soilHum, float soilEC,
                           String hora) {
  (void)temperature;
  (void)humedad;
  (void)DPV;
  (void)soilTemp;
  (void)soilHum;
  (void)soilEC;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  struct PageInfo {
    char tipo;   // 'A' ambiente, 'S' suelo
    uint8_t id;
    float t;
    float h;
    float v;     // DPV para ambiente, EC para suelo
  };

  PageInfo pages[6];
  uint8_t pageCount = 0;

  // =====================================================
  // Leer ambientes IDs 1..4
  // =====================================================
  struct { float t; float h; bool ok; } airData[5] = {};
airData[1] = {temperatura1, humedad1, sensorAmbOK1};
airData[2] = {temperatura2, humedad2, sensorAmbOK2};
airData[3] = {temperatura3, humedad3, sensorAmbOK3};
airData[4] = {temperatura4, humedad4, sensorAmbOK4};

for (uint8_t id = 1; id <= 4; id++) {
  if (!airData[id].ok) continue;
  float t = airData[id].t, h = airData[id].h;
  if (!isfinite(t) || !isfinite(h)) continue;
  if (t <= -40.0f || t >= 85.0f) continue;
  if (h < 0.0f || h > 100.0f) continue;
  pages[pageCount] = {'A', id, t, h, calcularDPV(t, h)};
  pageCount++;
}

  // =====================================================
  // Leer suelo IDs 5..6
  // =====================================================
  // ✅ REEMPLAZÁ por esto:
if (sensorSueloOK5 && isfinite(temperaturaSuelo5)) {
  pages[pageCount] = {'S', 5, temperaturaSuelo5, humedadSuelo5, ECSuelo5};
  pageCount++;
}
if (sensorSueloOK6 && isfinite(temperaturaSuelo6)) {
  pages[pageCount] = {'S', 6, temperaturaSuelo6, humedadSuelo6, ECSuelo6};
  pageCount++;
}

  // =====================================================
  // Si no encontró ningún sensor válido
  // =====================================================
  if (pageCount == 0) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Sin");
    display.setCursor(0, 20);
    display.print("sensores");
    display.setCursor(0, 40);
    display.print("validos");

    display.setTextSize(1);

    IPAddress ip = (WiFi.getMode() == WIFI_STA) ? WiFi.localIP() : WiFi.softAPIP();
    String ipStr = ip.toString();

    display.setCursor(0, 57);
    display.print(ipStr);

    int horaX = 128 - (hora.length() * 6);
    if (horaX < 92) horaX = 92;

    display.setCursor(horaX, 57);
    display.print(hora);

    display.display();
    return;
  }

  // =====================================================
  // Alternancia entre páginas válidas
  // =====================================================
  static uint32_t lastSwitch = 0;
  static uint8_t currentPage = 0;

  if ((uint32_t)(millis() - lastSwitch) >= 2000UL) {
    lastSwitch = millis();
    currentPage++;
    if (currentPage >= pageCount) currentPage = 0;
  }

  if (currentPage >= pageCount) currentPage = 0;

  // =====================================================
  // Dibujar página actual
  // =====================================================
  display.setTextSize(2);

  if (pages[currentPage].tipo == 'A') {
    uint8_t id = pages[currentPage].id;

    display.setCursor(0, 0);
    display.print("T");
    display.print(id);
    display.print(":");
    display.print(String(pages[currentPage].t, 1));

    display.setCursor(0, 20);
    display.print("H");
    display.print(id);
    display.print(":");
    display.print(String(pages[currentPage].h, 1));

    display.setCursor(0, 40);
    display.print("D");
    display.print(id);
    display.print(":");
    display.print(String(pages[currentPage].v, 1));
  } else {
    uint8_t id = pages[currentPage].id;

    display.setCursor(0, 0);
    display.print("Ts");
    display.print(id);
    display.print(":");
    display.print(String(pages[currentPage].t, 1));

    display.setCursor(0, 20);
    display.print("Hs");
    display.print(id);
    display.print(":");
    display.print(String(pages[currentPage].h, 1));

    display.setCursor(0, 40);
    display.print("EC");
    display.print(id);
    display.print(":");
    display.print(String(pages[currentPage].v, 0));
  }

  // =====================================================
  // Pie: IP + hora
  // =====================================================
  display.setTextSize(1);

  IPAddress ip = (WiFi.getMode() == WIFI_STA) ? WiFi.localIP() : WiFi.softAPIP();
  String ipStr = ip.toString();

  display.setCursor(0, 57);
  display.print(ipStr);

  int horaX = 128 - (hora.length() * 6);
  if (horaX < 92) horaX = 92;

  display.setCursor(horaX, 57);
  display.print(hora);

  display.display();
}

// Debug OLED para entender por qué R5 no se comporta como R1
// Muestra: modo/param/direccion, min/max, estado cacheado vs pin real,
// ventana (si aplica) y decisión que tomaría el AUTO (ON/OFF/HOLD)
  void mostrarEnPantallaOLEDDebug(float temperature, float humedad, float DPV, String hora) {
  (void)temperature; (void)humedad; (void)DPV; // no usamos esto en debug RS485

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  // Línea 0: Estado general
  display.setCursor(0, 0);
  display.print("RS485 SCAN ");
  display.print(g_scanFound ? "FOUND" : "RUN");

  // Línea 1: Baud actual
  display.setCursor(0, 10);
  display.print("baud:");
  display.print(g_scanBaud);

  // Línea 2: Paridad + ID
  display.setCursor(0, 20);
  display.print("par:");
  display.print(g_scanParity ? "E" : "N");
  display.print(" id:");
  display.print(g_scanId);

  // Línea 3: Resultado ultima lectura
  display.setCursor(0, 30);
  display.print("ok:");
  display.print(g_scanLastOk ? "1" : "0");
  display.print(" out:");
  display.print(g_scanLastOut);

  // Línea 4: Intentos (te sirve para ver que avanza)
  display.setCursor(0, 40);
  display.print("tries:");
  display.print(g_scanTries);

  // Línea 5: Hora
  display.setCursor(96, 56);
  display.print(hora);

  display.display();
}



void mostrarMensajeBienvenida() {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SH110X_WHITE); // ✅ Correcta para Adafruit_SH110X
  // Tamaño del texto más grande
  //display.setTextColor(SSD1306_WHITE  );  // Color del texto

  // Mostrar el mensaje de bienvenida "Druida"
  display.setCursor((128 - (6 * 3 * 6)) / 2, 5);    // Centrando "Druida" en X
  display.println("Druida");

  // Mostrar "Bot" justo debajo, centrado
  display.setCursor((128 - (3 * 3 * 6)) / 2, 35);    // Centrando "Bot" en X
  display.println("Bot");

  display.display();            // Actualiza la pantalla
}

String obtenerMotivoReinicio() {
  esp_reset_reason_t resetReason = esp_reset_reason();
  String motivoReinicio;

  switch (resetReason) {
    case ESP_RST_POWERON:
      motivoReinicio = "Power-on";
      break;
    case ESP_RST_EXT:
      motivoReinicio = "External reset";
      break;
    case ESP_RST_SW:
      motivoReinicio = "Software reset";
      break;
    case ESP_RST_PANIC:
      motivoReinicio = "Panic";
      break;
    case ESP_RST_INT_WDT:
      motivoReinicio = "Interrupt WDT";
      break;
    case ESP_RST_TASK_WDT:
      motivoReinicio = "Task WDT";
      break;
    case ESP_RST_WDT:
      motivoReinicio = "Other WDT";
      break;
    case ESP_RST_DEEPSLEEP:
      motivoReinicio = "Deep sleep";
      break;
    case ESP_RST_BROWNOUT:
      motivoReinicio = "Brownout";
      break;
    case ESP_RST_SDIO:
      motivoReinicio = "SDIO reset";
      break;
    default:
      motivoReinicio = "Unknown";
      break;
  }

  return motivoReinicio;
}

String convertirModo(int modo) {
    switch (modo) {
        case MANUAL:
            return "Manual";
        case AUTO:
            return "Automático";
        case CONFIG:
            return "Configuración";
        case STATUS:
            return "Estado";
        case SUPERCICLO:
            return "Superciclo";
        case TIMER:
            return "Temporizador";
        case RIEGO:
            return "Riego";
        default:
            return "Desconocido";
    }
}

String convertirParametro(int parametro) {
    switch (parametro) {
        case H:
            return "Humedad";
        case T:
            return "Temperatura";
        case D:
            return "DPV";
        default:
            return "Desconocido";
    }
}


String convertirDia(int dia) {
    switch (dia) {
        case 0:
            return "Domingo";
        case 1:
            return "Lunes";
        case 2:
            return "Martes";
        case 3:
            return "Miércoles";
        case 4:
            return "Jueves";
        case 5:
            return "Viernes";
        case 6:
            return "Sábado";
        default:
            return "Desconocido";
    }
}


String formatoHora(int hora, int minuto) {
    char buffer[6]; // Buffer para almacenar la cadena formateada
    sprintf(buffer, "%02d:%02d", hora, minuto); // Formatear con dos dígitos
    return String(buffer); // Devolver como String
}



void riegoIntermitente() {
  unsigned long currentMillis = millis();

  // Si ya hicimos todos los riegos programados, mantener la bomba apagada
  if (riegosHechos >= cantidadRiegos) {
    setRelayCH(RELAY3, false); // OFF
    R3estado = HIGH;
    enRiego = false;
    return;
  }

  if (!enRiego) { 
    // Estamos en pausa (NO riego)
    // Primer ciclo: si previousMillisRiego == 0, arrancamos enseguida
    if (previousMillisRiego == 0 ||
        currentMillis - previousMillisRiego >= (unsigned long)tiempoNoRiego * 1000UL) {

      // Encender el relé
      setRelayCH(RELAY3, true);
      R3estado = LOW;
      previousMillisRiego = currentMillis;
      enRiego = true;

      // Podrías guardar, pero no es tan crítico aquí
      GuardadoR3();
    }

  } else {
    // Estamos regando
    if (currentMillis - previousMillisRiego >= (unsigned long)tiempoRiego * 1000UL) {
      // Apagar el relé
      setRelayCH(RELAY3, false);
      R3estado = HIGH;
      previousMillisRiego = currentMillis;
      enRiego = false;

      // Contabilizar un riego completo
      riegosHechos++;
      GuardadoR3();   // 🔴 guardamos el nuevo valor de riegosHechos (y ultimoDiaRiego)
    }
  }
}

void riegoIntermitenteR8() {
  unsigned long currentMillis = millis();

  if (riegosHechosR8 >= cantidadRiegosR8) {
    setRelayCH(RELAY8, false);
    R8estado = HIGH;
    enRiegoR8 = false;
    return;
  }

  if (!enRiegoR8) {
    if (previousMillisRiegoR8 == 0 ||
        currentMillis - previousMillisRiegoR8 >= (unsigned long)tiempoNoRiegoR8 * 1000UL) {

      setRelayCH(RELAY8, true);
      R8estado = LOW;
      previousMillisRiegoR8 = currentMillis;
      enRiegoR8 = true;

      GuardadoR8();
    }

  } else {
    if (currentMillis - previousMillisRiegoR8 >= (unsigned long)tiempoRiegoR8 * 1000UL) {
      setRelayCH(RELAY8, false);
      R8estado = HIGH;
      previousMillisRiegoR8 = currentMillis;
      enRiegoR8 = false;

      riegosHechosR8++;
      GuardadoR8();
    }
  }
}






void moveServoSlowly(int targetPosition) {
  if (targetPosition > currentPosition) {
    for (int pos = currentPosition; pos <= targetPosition; pos++) {
      dimmerServo.write(pos); // Mover un paso
      delay(15); // Controla la velocidad del movimiento (ajusta si es necesario)
    }
  } else {
    for (int pos = currentPosition; pos >= targetPosition; pos--) {
      dimmerServo.write(pos); // Mover un paso
      delay(15); // Controla la velocidad del movimiento (ajusta si es necesario)
    }
  }
  currentPosition = targetPosition; // Actualizar la posición actual
}

String getRelayName(int relayIndex) {
    if (relayIndex >= 0 && relayIndex < 10) {
        return relayNames[relayIndex];
    }
    return "Desconocido";
}

String htmlEncode(const String& data) {
  String encoded = "";
  for (unsigned int i = 0; i < data.length(); ++i) {
    char c = data.charAt(i);
    switch (c) {
      case '&': encoded += "&amp;"; break;
      case '<': encoded += "&lt;"; break;
      case '>': encoded += "&gt;"; break;
      case '"': encoded += "&quot;"; break;
      case '\'': encoded += "&#39;"; break;
      default:
        // Solo incluir caracteres imprimibles normales
        if (c >= 32 && c <= 126) {
          encoded += c;
        } else {
          encoded += '?';  // Caracteres no imprimibles se muestran como '?'
        }
        break;
    }
  }
  return encoded;
}






void requestSensorData() {
  sensorDataValid = false;

  I2CNano.requestFrom(8, 8);  // Dirección 8, 8 bytes esperados

  unsigned long start = millis();
  while (I2CNano.available() < 8 && millis() - start < 100) {
    delay(1);
  }

  if (I2CNano.available() == 8) {
    sensor1Value = I2CNano.read() << 8 | I2CNano.read();  // H1
    sensor2Value = I2CNano.read() << 8 | I2CNano.read();  // H2
    sensor3Value = I2CNano.read() << 8 | I2CNano.read();  // H3
    int rawPH = I2CNano.read() << 8 | I2CNano.read();     // pH
    sensorPH = rawPH / 100.0;

    sensorDataValid = true;
    Serial.printf("H1: %d%%, H2: %d%%, H3: %d%%, pH: %.2f\n", sensor1Value, sensor2Value, sensor3Value, sensorPH);
  } else {
    //Serial.println("⚠️ Error: no se recibieron datos I2C desde Arduino Nano.");
    sensor1Value = sensor2Value = sensor3Value = 0;
    sensorPH = 0.0;
  }
}


void debugPrintConfig() {
  Serial.println(F("===== CONFIGURACIÓN CARGADA ====="));

  // R1–R4
  Serial.print(F("R1 -> min: ")); Serial.print(minR1);
  Serial.print(F(" | max: ")); Serial.print(maxR1);
  Serial.print(F(" | modo: ")); Serial.print(modoR1);
  Serial.print(F(" | sensor: ")); Serial.print(sensorR1);
  Serial.print(F(" | estado: ")); Serial.println(estadoR1);

  Serial.print(F("R2 -> min: ")); Serial.print(minR2);
  Serial.print(F(" | max: ")); Serial.print(maxR2);
  Serial.print(F(" | modo: ")); Serial.print(modoR2);
  Serial.print(F(" | sensor: ")); Serial.print(sensorR2);
  Serial.print(F(" | estado: ")); Serial.println(estadoR2);

  Serial.print(F("R3 -> min: ")); Serial.print(minR3);
  Serial.print(F(" | max: ")); Serial.print(maxR3);
  Serial.print(F(" | modo: ")); Serial.print(modoR3);
  Serial.print(F(" | sensor: ")); Serial.print(sensorR3);
  Serial.print(F(" | estado: ")); Serial.println(estadoR3);

  Serial.print(F("R4 -> modo: ")); Serial.print(modoR4);
  Serial.print(F(" | estado: ")); Serial.println(estadoR4);

  // R5 (el crítico)
  Serial.print(F("R5 -> min: ")); Serial.print(minR5);
  Serial.print(F(" | max: ")); Serial.print(maxR5);
  Serial.print(F(" | minOn: ")); Serial.print(minOnR5);
  Serial.print(F(" | minOff: ")); Serial.print(minOffR5);
  Serial.print(F(" | horaOn: ")); Serial.print(horaOnR5);
  Serial.print(F(" | horaOff: ")); Serial.print(horaOffR5);
  Serial.print(F(" | sensor: ")); Serial.print(sensorR5);
  Serial.print(F(" | param: ")); Serial.print(paramR5);
  Serial.print(F(" | direccion: ")); Serial.print(direccionR5);
  Serial.print(F(" | modo: ")); Serial.print(modoR5);
  Serial.print(F(" | estado: ")); Serial.println(estadoR5);

  // WiFi
  Serial.print(F("SSID: ")); Serial.println(ssid);
  Serial.print(F("Password: ")); Serial.println(password);

  // Telegram
  Serial.print(F("Chat ID: ")); Serial.println(chat_id);

  Serial.println(F("================================="));
}

// === Escritura de String ACOTADA por capacidad del bloque ===
void writeStringBounded(int addr, const String& s, size_t capacity) {
  if (capacity < 2) return;                 // necesita al menos len + '\0'
  size_t maxChars = capacity - 2;           // espacio útil real
  size_t n = s.length();
  if (n > maxChars) n = maxChars;

  EEPROM.write(addr, (uint8_t)n);           // longitud efectiva
  for (size_t i = 0; i < n; ++i) {
    EEPROM.write(addr + 1 + i, s[i]);
  }
  EEPROM.write(addr + 1 + n, '\0');         // terminador

  // Limpia sobrante del bloque (evita residuos de strings previas más largas)
  for (size_t i = n + 1; i < capacity; ++i) {
    EEPROM.write(addr + i, 0x00);
  }
  // En ESP8266/ESP32: acordate de EEPROM.commit() tras guardar todo.
}

// === Lectura de String ACOTADA por capacidad del bloque ===
String readStringBounded(int addr, size_t capacity) {
  if (capacity < 2) return String();
  uint8_t n = EEPROM.read(addr);
  size_t maxChars = capacity - 2;
  if (n > maxChars) n = maxChars;

  String out;
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    out += char(EEPROM.read(addr + 1 + i));
  }
  return out;
}

// Convierte "HH:MM" a minutos (0..1439). Si String inválido, devuelve -1.
int parseHHMMToMinutes(const String& hhmm) {
  if (hhmm.length() < 4) return -1;
  int sep = hhmm.indexOf(':');
  if (sep < 0) return -1;
  int h = hhmm.substring(0, sep).toInt();
  int m = hhmm.substring(sep + 1).toInt();
  if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
  return h * 60 + m;
}

// Normaliza minutos de reloj a rango [0,1439] (usalo sólo si lo necesitás)
int normClockMin(int mm) {
  int x = mm % 1440;
  if (x < 0) x += 1440;
  return x;
}

// ===== Tiempo absoluto (RTC) y conversión a reloj =====
// Minuto absoluto desde RTC (DS3231)
unsigned long nowAbsMin() {
  datetime_t now;
  PCF85063_Read_Time(&now);

  // Convertimos a epoch UTC y pasamos a minutos
  return unixFromPCF(now) / 60UL;
}


// Convierte minuto absoluto a reloj del día (0..1439, -1 si inválido)
int absToClockMin(long absMin) {
  if (absMin < 0) return -1;
  long m = absMin % 1440L;
  if (m < 0) m += 1440L;
  return (int)m;
}

// Setea HH:MM visibles desde minuto absoluto
void setClockFromAbs(long absMin, int &outHour, int &outMin) {
  int cm = absToClockMin(absMin);
  if (cm >= 0) {
    outHour = cm / 60;
    outMin  = cm % 60;
  }
}

// ===== Semilla clara del SUPERCICLO (idempotente) =====
void seedSupercicloR4(bool encendidaAhora) {
  Serial.println(F("[SUPERCICLO] Seed R4"));

  int nowM  = nowMinutesLocal();                // 0..1439
  int onUI  = (horaOnR4 * 60 + minOnR4) % 1440; // ancla UI
  int offUI = (onUI + horasLuz) % 1440;

  auto inWindow = [&](int now, int start, int end) {
    return (start < end) ? (now >= start && now < end)
                         : (now >= start || now < end);
  };

  if (encendidaAhora) {
    // Encendida: próximo evento = apagar en now + horasLuz
    nextOffR4Abs = (nowM + (int)horasLuz) % 1440;
    nextOnR4Abs  = (nextOffR4Abs + (int)horasOscuridad) % 1440;
  } else {
    if (inWindow(nowM, onUI, offUI)) {
      // Está dentro de la ventana pero apagada: apaga pronto (failsafe) y rearmá ciclo
      nextOffR4Abs = (nowM + (int)horasLuz) % 1440;    // o ahora mismo si querés
      nextOnR4Abs  = (nextOffR4Abs + (int)horasOscuridad) % 1440;
    } else {
      // Próximo encendido en el onUI válido (hoy o siguiente ciclo)
      // si ya pasó onUI hoy, programá el próximo ciclo completo
      int ciclo = ((int)horasLuz + (int)horasOscuridad) % 1440;
      if (ciclo == 0) ciclo = 1; // evitar ciclos nulos
      int targetOn = (nowM <= onUI) ? onUI : (onUI + ciclo) % 1440;

      nextOnR4Abs  = targetOn;
      nextOffR4Abs = (targetOn + (int)horasLuz) % 1440;
    }
  }

  // Saneá por si acaso (nunca números grandes)
  nextOnR4Abs  = (nextOnR4Abs  % 1440 + 1440) % 1440;
  nextOffR4Abs = (nextOffR4Abs % 1440 + 1440) % 1440;

  Serial.print(F("  nextOnR4Abs : "));  Serial.println(nextOnR4Abs);
  Serial.print(F("  nextOffR4Abs: "));  Serial.println(nextOffR4Abs);
}


// Convierte minutos [0..1439] a "HH:MM"
String minutesToHHMM(int mins) {
  if (mins < 0) mins = 0;
  mins %= 1440;
  return formatTwoDigits(mins/60) + ":" + formatTwoDigits(mins%60);
}

int parseHHMMToMinutesSafe(const String& hhmm) {
  String s = hhmm; s.trim();
  int colon = s.indexOf(':');
  if (colon < 1 || colon > (int)s.length()-2) return -1;
  int h = s.substring(0, colon).toInt();
  int m = s.substring(colon+1).toInt();
  if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
  return h*60 + m;
}


// Requiere: RTC_DS3231 rtc; (o equivalente) ya inicializado en s().
inline int nowMinutesLocal() {
  datetime_t now;
  PCF85063_Read_Time(&now);

  int h = now.hour;     // 0..23 (hora del RTC)
  int m = now.minute;   // 0..59

  // Ajuste horario local (AR -03)
  h -= 3;
  if (h < 0) h += 24;   // wrap a rango 0..23

  // devolver minutos locales 0..1439
  return h * 60 + m;
}


// Devuelve YYYYMMDD local (aplicando -3h si tu RTC está en UTC)
uint32_t localDateKey(const DateTime& now) {
  // Ajuste local: -3 horas (si tu DS3231 ya está en hora local, poné TZ=0)
  const int32_t TZ = -3 * 3600;
  uint32_t t = now.unixtime();
  t += TZ;
  DateTime loc(t);

  // yyyymmdd como entero: 20250915
  return (uint32_t)(loc.year()) * 10000UL + (uint32_t)(loc.month()) * 100UL + (uint32_t)(loc.day());
}


void tickDaily() {
  if (!canPersist) return;

  datetime_t now;
  PCF85063_Read_Time(&now);

  uint32_t todayKey = localDateKeyFromPCF(now);
  if (todayKey == 0) return;

  if (lastDateKey == 0) {
    lastDateKey = todayKey;
    return;
  }

  if (todayKey != lastDateKey) {
    if (vegeActive  && vegeDays  > 0) vegeDays++;
    if (floraActive && floraDays > 0) floraDays++;
    lastDateKey = todayKey;
    Guardado_General();
  }
}



// ===== Ajuste horario (cambiar a 0 si tu DS3231 ya está en hora local) =====
static const int32_t TZ_SECONDS = -3 * 3600; // Argentina (-3)

uint32_t nowLocalEpoch() {
  datetime_t now;
  PCF85063_Read_Time(&now);

  uint32_t t = unixFromPCF(now);   // epoch UTC desde PCF85063
  t += TZ_SECONDS;                 // si tu RTC ya está en local, dejá TZ_SECONDS=0
  return t;
}


// Convierte horas a segundos con sanidad
static inline uint32_t hoursToSec(int h) {
  if (h < 0) h = 0;
  return (uint32_t)h * 3600UL;
}



// ===== Utilidades
static inline int64_t nowUtcSec64() {
  datetime_t now;
  PCF85063_Read_Time(&now);
  return (int64_t)unixFromPCF(now);
}


//constexpr uint32_t SEC_PER_DAY = 86400UL;


// ===== OPCIONAL: versión en minutos para ciclos no enteros (12.5 h = 750 min)
int virtualDaysSinceMinutes(uint32_t startEpoch,
                            uint32_t minutosLuz,
                            uint32_t minutosOscuridad,
                            int tzOffsetSec = (-3 * 3600)) {
  if (startEpoch == 0) return 0;

  uint64_t cycleMin = (uint64_t)minutosLuz + (uint64_t)minutosOscuridad;
  if (cycleMin == 0) return 0;

  uint64_t cycleSec = cycleMin * 60ULL;

  int64_t nowLocal   = nowUtcSec64()        + (int64_t)tzOffsetSec;
  int64_t startLocal = (int64_t)startEpoch  + (int64_t)tzOffsetSec;
  if (nowLocal < startLocal) return 0;

  uint64_t elapsed = (uint64_t)(nowLocal - startLocal);
  uint64_t cyclesCompleted = elapsed / cycleSec;
  return (int)(cyclesCompleted + 1);
}








void setAllRelays(bool turnOn) {
  // R1
  estadoR1 = turnOn ? 1 : 0;
  modoR1   = MANUAL;
  setRelayActiveLow(RELAY1, turnOn);

  // R2
  estadoR2 = turnOn ? 1 : 0;
  modoR2   = MANUAL;
  setRelayActiveLow(RELAY2, turnOn);

  // R3
  estadoR3 = turnOn ? 1 : 0;
  modoR3   = MANUAL;
  setRelayActiveLow(RELAY3, turnOn);

  // R4
  estadoR4 = turnOn ? 1 : 0;
  modoR4   = MANUAL;
  setRelayActiveLow(RELAY4, turnOn);

  // R5
  estadoR5 = turnOn ? 1 : 0;
  modoR5   = MANUAL;
  setRelayActiveLow(RELAY5, turnOn);

  // R6
  estadoR6 = turnOn ? 1 : 0;
  modoR6   = MANUAL;
  setRelayActiveLow(RELAY6, turnOn);

  // R7
  estadoR7 = turnOn ? 1 : 0;
  modoR7   = MANUAL;
  R7estado = turnOn ? LOW : HIGH;
  setRelayActiveLow(RELAY7, turnOn);

  // R8
  estadoR8 = turnOn ? 1 : 0;
  modoR8   = MANUAL;
  R8estado = turnOn ? LOW : HIGH;
  setRelayActiveLow(RELAY8, turnOn);

  Guardado_General();
}


void registerWiFiEvents() {
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    Serial.print(F("✅ GOT IP: "));
    Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
    g_needHardReconnect = false; // ya estamos bien
  }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
    // Razones típicas: 201 NO_AP_FOUND, 202 AUTH_FAIL, 205 ASSOC_LEAVE, etc.
    Serial.printf("⚠️  STA DISCONNECTED. reason=%d\n", info.wifi_sta_disconnected.reason);
    g_needHardReconnect = true;  // pedir reconexión fuerte
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
}



void relayDriverInit() {
  // I2C por los pines correctos
  Wire.begin(I2C_SDA_MAIN, I2C_SCL_MAIN);
  Wire.setClock(100000);

  // Polarity normal
  tcaWriteReg(TCA9554_POL_REG, 0x00);

  // Todos output (0=output)
  tcaWriteReg(TCA9554_CONFIG_REG, 0x00);

  // Todo OFF
  g_relayMask = 0x00;
  tcaWriteReg(TCA9554_OUTPUT_REG, g_relayMask);
}

// Escribe 1 relé (ch = 1..8)
void setRelayCH(uint8_t ch, bool on) {
  if (ch < 1 || ch > 8) return;
  uint8_t bit = (1u << (ch - 1));

  if (on) g_relayMask |= bit;   // ON = 1
  else    g_relayMask &= ~bit;  // OFF = 0

  tcaWriteReg(TCA9554_OUTPUT_REG, g_relayMask);
}

// Helpers opcionales
void allRelays(bool on) {
  g_relayMask = on ? 0xFF : 0x00;
  tcaWriteReg(TCA9554_OUTPUT_REG, g_relayMask);
}

static bool i2cWriteReg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data, len);
  return Wire.endTransmission() == 0;
}
static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  return Wire.requestFrom(addr, (uint8_t)len) == len && (Wire.readBytes(data, len) == len);
}


void PCF85063_Read_Time(datetime_t *time) {
  uint8_t buf[7] = {0};
  if (!i2cReadReg(PCF85063_ADDRESS, RTC_SECOND_ADDR, buf, sizeof(buf))) {
    // fallback seguro
    time->year=1970; time->month=1; time->day=1; time->dotw=0; time->hour=0; time->minute=0; time->second=0;
    return;
  }
  time->second = bcdToDec(buf[0] & 0x7F);
  time->minute = bcdToDec(buf[1] & 0x7F);
  time->hour   = bcdToDec(buf[2] & 0x3F);
  time->day    = bcdToDec(buf[3] & 0x3F);
  time->dotw   = bcdToDec(buf[4] & 0x07);
  time->month  = bcdToDec(buf[5] & 0x1F);
  time->year   = (uint16_t)(bcdToDec(buf[6]) + 1970);
}

void PCF85063_Set_All(datetime_t time) {
  uint8_t buf[7] = {
    (uint8_t)decToBcd(time.second),
    (uint8_t)decToBcd(time.minute),
    (uint8_t)decToBcd(time.hour),
    (uint8_t)decToBcd(time.day),
    (uint8_t)decToBcd(time.dotw),
    (uint8_t)decToBcd(time.month),
    (uint8_t)decToBcd((int)time.year - 1970)
  };
  i2cWriteReg(PCF85063_ADDRESS, RTC_SECOND_ADDR, buf, sizeof(buf));
}

uint8_t tcaReadReg(uint8_t reg) {
  Wire.beginTransmission(TCA9554_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);                 // repeated start
  Wire.requestFrom(TCA9554_ADDRESS, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0x00; // si falla, asumimos OFF
}

inline bool relayIsOnHW(uint8_t ch) {
  if (ch < 1 || ch > 8) return false;
  uint8_t out = tcaReadReg(TCA9554_OUTPUT_REG);
  return (out & (1u << (ch - 1))) != 0;        // ON = bit 1
}




//ACA EMPIEZA RS485

// Copia segura a g_lastRx para OLED
static void dbgCopyRx(const uint8_t* src, uint8_t n) {
  uint8_t m = n;
  if (m > sizeof(g_lastRx)) m = sizeof(g_lastRx);
  for (uint8_t i = 0; i < m; i++) g_lastRx[i] = src[i];
  g_lastRxN = m;
}

// CRC16 Modbus (poly 0xA001)
static uint16_t modbusCRC(const uint8_t* buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

// Leer 1 Holding Register (func 0x03). Devuelve true si OK y coloca outValue.
static bool readFrameWithOrder(uint8_t id, uint16_t regAddr, bool crcHiLo, uint16_t &outValue) {

  // ----- para OLED: qué pedimos -----
  g_lastId  = id;
  g_lastFn  = 0x03;
  g_lastReg = regAddr;
  g_lastQty = 1;

  g_lastErr   = 0;
  g_lastRxLen = 0;
  g_lastRxN   = 0;

  uint8_t req[8];
  req[0] = id;
  req[1] = 0x03;
  req[2] = (regAddr >> 8) & 0xFF;
  req[3] = (regAddr >> 0) & 0xFF;
  req[4] = 0x00;
  req[5] = 0x01;

  uint16_t crc = modbusCRC(req, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8) & 0xFF;

  // Orden CRC
  if (crcHiLo) {      // [HI][LO]
    req[6] = crcHi;
    req[7] = crcLo;
  } else {            // [LO][HI] (estándar)
    req[6] = crcLo;
    req[7] = crcHi;
  }

  // Vaciar RX antes de enviar
  while (RS485.available()) (void)RS485.read();

  // Enviar
  RS485.write(req, sizeof(req));
  RS485.flush();

  // Recibir respuesta típica: 7 bytes
  uint8_t resp[7];
  size_t got = 0;
  uint32_t t0 = millis();

  while (millis() - t0 < 400) {
    while (RS485.available() && got < sizeof(resp)) {
      resp[got++] = (uint8_t)RS485.read();
    }
    if (got == sizeof(resp)) break;
  }

  g_rs485_lastMs = millis();
  g_lastRxLen = (uint8_t)got;

  if (got == 0) {
    g_lastErr = 1; // timeout
    return false;
  }

  // Copiar lo recibido (aunque esté incompleto) para ver en OLED
  dbgCopyRx(resp, (uint8_t)got);

  if (got != sizeof(resp)) {
    g_lastErr = 4; // formato / incompleto
    return false;
  }

  // Chequeo ID
  if (resp[0] != id) {
    g_lastErr = 5; // wrongID
    return false;
  }

  // Excepción Modbus (func|0x80)
  if (resp[1] == (uint8_t)(0x03 | 0x80)) {
    g_lastErr = 3; // exception
    return false;
  }

  // Formato esperado
  if (resp[1] != 0x03 || resp[2] != 0x02) {
    g_lastErr = 4; // formato
    return false;
  }

  // Validar CRC (aceptar ambos órdenes por robustez)
  uint16_t crcCalc = modbusCRC(resp, 5);
  uint16_t crcRecv_LoHi = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8); // [LO][HI]
  uint16_t crcRecv_HiLo = (uint16_t)resp[6] | ((uint16_t)resp[5] << 8); // [HI][LO]

  if (crcCalc != crcRecv_LoHi && crcCalc != crcRecv_HiLo) {
    g_lastErr = 2; // badCRC
    return false;
  }

  outValue = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

bool modbusReadRegisters(uint8_t slaveId, uint8_t funcCode, uint16_t startReg, uint16_t regCount, uint16_t *outRegs) {
  if (outRegs == nullptr || regCount == 0 || regCount > 20) {
    return false;
  }

  uint8_t req[8];
  req[0] = slaveId;
  req[1] = funcCode;
  req[2] = highByte(startReg);
  req[3] = lowByte(startReg);
  req[4] = highByte(regCount);
  req[5] = lowByte(regCount);

  uint16_t crc = modbusCRC(req, 6);
  req[6] = lowByte(crc);
  req[7] = highByte(crc);

  // Limpiar buffer
  while (RS485.available()) {
    RS485.read();
  }

  // Enviar
  RS485.write(req, 8);
  RS485.flush();

  // Esperar respuesta
  const uint8_t expectedLen = 5 + regCount * 2;
  uint8_t resp[64];
  uint8_t idx = 0;

  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < 120UL) {
    while (RS485.available()) {
      uint8_t b = RS485.read();
      if (idx < sizeof(resp)) {
        resp[idx++] = b;
      }
    }

    if (idx >= expectedLen) break;
    yield();
  }

  if (idx < expectedLen) return false;
  if (resp[0] != slaveId) return false;
  if (resp[1] == (funcCode | 0x80)) return false;
  if (resp[1] != funcCode) return false;
  if (resp[2] != (regCount * 2)) return false;

  uint16_t crcRx   = ((uint16_t)resp[idx - 1] << 8) | resp[idx - 2];
  uint16_t crcCalc = modbusCRC(resp, idx - 2);

  if (crcRx != crcCalc) return false;

  for (uint16_t i = 0; i < regCount; i++) {
    outRegs[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
  }

  return true;
}

// Esta prueba ambas órdenes de CRC
static bool modbusReadHolding1(uint8_t id, uint16_t regAddr, uint16_t &outValue) {
  if (readFrameWithOrder(id, regAddr, false, outValue)) return true; // estándar LO/HI
  if (readFrameWithOrder(id, regAddr, true,  outValue)) return true; // HI/LO por si acaso
  return false;
}

// Llamar en setup
void rs485Init() {
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  delay(50);
}

// Lee temp/hum y devuelve true si ambos OK
bool rs485ReadTH(uint8_t sensorId, float &temperature, float &humedad) {

  // Flag manual: desactivado permanentemente
  if ((sensorId == AIR_ID_1 && !sensorAir1Activo) ||
      (sensorId == AIR_ID_2 && !sensorAir2Activo) ||
      (sensorId == AIR_ID_3 && !sensorAir3Activo) ||
      (sensorId == AIR_ID_4 && !sensorAir4Activo)) {
    temperature = NAN;
    humedad     = NAN;
    return false;
  }

  // Suspensión temporal por fallos consecutivos
  uint32_t nowMs = millis();
  if (rs485SuspendedUntil[sensorId] != 0 &&
      (int32_t)(nowMs - rs485SuspendedUntil[sensorId]) < 0) {
    temperature = NAN;
    humedad     = NAN;
    return false; // retorno inmediato, sin tocar el bus
  }

  uint16_t rawH = 0, rawT = 0;

  bool okH = modbusReadHolding1(sensorId, REG_HUM, rawH);
  g_rs485_okH = okH;
  if (okH) g_rawH = rawH;

  bool okT = modbusReadHolding1(sensorId, REG_TEMP, rawT);
  g_rs485_okT = okT;
  if (okT) g_rawT = rawT;

  bool ok = okH && okT;

  if (!ok) {
    rs485FailCount[sensorId]++;
    if (rs485FailCount[sensorId] >= RS485_MAX_FAILS) {
      rs485FailCount[sensorId]      = 0;
      rs485SuspendedUntil[sensorId] = millis() + RS485_SUSPEND_MS;
      Serial.printf("[RS485] AIR sensor %d suspendido 60s por fallos\n", sensorId);
    }
    temperature = NAN;
    humedad     = NAN;
    return false;
  }

  // Éxito: rehabilitar
  rs485FailCount[sensorId]      = 0;
  rs485SuspendedUntil[sensorId] = 0;

  humedad     = rawH / 10.0f;
  temperature = rawT / 10.0f;
  return true;
}

bool rs485ReadSoil(uint8_t sensorId, float &soilTemperature, float &soilHumedad, float &soilECValue) {

  // Flag manual: desactivado permanentemente
  if ((sensorId == SOIL_ID_5 && !sensorSoil5Activo) ||
      (sensorId == SOIL_ID_6 && !sensorSoil6Activo)) {
    soilTemperature = NAN;
    soilHumedad     = NAN;
    soilECValue     = NAN;
    return false;
  }

  // Suspensión temporal por fallos consecutivos
  uint32_t nowMs = millis();
  if (rs485SuspendedUntil[sensorId] != 0 &&
      (int32_t)(nowMs - rs485SuspendedUntil[sensorId]) < 0) {
    soilTemperature = NAN;
    soilHumedad     = NAN;
    soilECValue     = NAN;
    return false; // retorno inmediato, sin tocar el bus
  }

  uint16_t regs[3] = {0, 0, 0};

  bool ok = modbusReadRegisters(sensorId, 0x03, 0x0000, 3, regs);
  if (!ok) {
    ok = modbusReadRegisters(sensorId, 0x04, 0x0000, 3, regs);
  }

  if (!ok) {
    rs485FailCount[sensorId]++;
    if (rs485FailCount[sensorId] >= RS485_MAX_FAILS) {
      rs485FailCount[sensorId]      = 0;
      rs485SuspendedUntil[sensorId] = millis() + RS485_SUSPEND_MS;
      Serial.printf("[RS485] SOIL sensor %d suspendido 60s por fallos\n", sensorId);
    }
    soilTemperature = NAN;
    soilHumedad     = NAN;
    soilECValue     = NAN;
    return false;
  }

  // Éxito: rehabilitar
  rs485FailCount[sensorId]      = 0;
  rs485SuspendedUntil[sensorId] = 0;

  soilHumedad     = regs[0] / 10.0f;
  soilTemperature = (int16_t)regs[1] / 10.0f;
  soilECValue     = regs[2];
  return true;
}

static void rs485Rebegin(uint32_t baud, bool evenParity) {
  RS485.end();
  delay(50);
  RS485.begin(baud, evenParity ? SERIAL_8E1 : SERIAL_8N1, RS485_RX, RS485_TX);
  delay(50);
}

static void rs485AutoScanStep() {
  if (g_scanFound) return;

  const uint32_t bauds[]     = {4800, 9600};
  const bool     parities[]  = {false, true}; // false=N, true=E
  const uint8_t  ids[]       = {1, 2, 3, 4, 5, 6};

  static uint8_t bi = 0, pi = 0, ii = 0;

  uint32_t baud = bauds[bi];
  bool evenP    = parities[pi];
  uint8_t id    = ids[ii];

  // Mostrar estado actual del scan
  g_scanBaud   = baud;
  g_scanParity = evenP ? 1 : 0;
  g_scanId     = id;

  rs485Rebegin(baud, evenP);

  uint16_t out = 0;
  bool ok = modbusReadHolding1(id, REG_HUM, out);

  // ---- NUEVO: guardar "telemetría" para OLED ----
  g_scanTries++;
  g_scanLastOk  = ok ? 1 : 0;
  g_scanLastOut = out;

  if (ok) {
    g_scanFound = 1;
    return;
  }

  // Avanzar índices correctamente
  ii++;
  if (ii >= ARR_LEN(ids)) {
    ii = 0;
    pi++;
  }
  if (pi >= ARR_LEN(parities)) {
    pi = 0;
    bi++;
  }
  if (bi >= ARR_LEN(bauds)) {
    bi = 0;
  }
}

// HASTA ACA rs485



// ============================
// RTC helpers (PCF85063)
// ============================

bool PCF85063_HasVLFlag() {
  uint8_t sec = 0;
  if (!i2cReadReg(PCF85063_ADDRESS, RTC_SECOND_ADDR, &sec, 1)) {
    // si no puedo leer => no confiable
    return true;
  }
  return (sec & 0x80u) != 0;  // VL = bit7
}

// Ojo: con RTC sin pila, el año suele venir 1970/2000/2099 o basura.
// Como vos seteás "fecha base 2026", este check está bien.
// Si quisieras ser más permisivo, bajá el 2022.
bool RTC_TimeLooksReasonable(const datetime_t& dt) {
  if (dt.year < 2022 || dt.year > 2099) return false;
  if (dt.month < 1 || dt.month > 12) return false;
  if (dt.day < 1 || dt.day > 31) return false;
  if (dt.hour > 23) return false;
  if (dt.minute > 59) return false;
  if (dt.second > 59) return false;
  return true;
}

// Lee RTC y actualiza: rtcHoraValida, rtcTienePila, horaActual, minutoActual
// Mejoras:
// - Si lectura falla o VL=1 => rtcHoraValida=false
// - No pisa horaActual/minutoActual si RTC inválido (así podés quedarte con EEPROM/RAM)
// - rtcTienePila se infiere por VL, NO por razonable (es una aproximación más honesta)
void RTC_UpdateStatusAndHM() {
  datetime_t dt;
  PCF85063_Read_Time(&dt);

  bool vl = PCF85063_HasVLFlag();
  bool razonable = RTC_TimeLooksReasonable(dt);

  rtcHoraValida = (!vl) && razonable;

  // Inferencia práctica más sana:
  // - VL=1 => seguro no mantuvo reloj (sin pila / pila agotada / corte largo)
  // - VL=0 no garantiza pila, pero sí que el RTC no reporta caída de tensión.
  rtcTienePila = (!vl);

  if (rtcHoraValida) {
    horaActual   = (int)dt.hour;
    minutoActual = (int)dt.minute;
  }
}


// ============================
// Sincroniza NTP (si hay WiFi) y setea RTC
// Devuelve true si sincronizó.
// ============================
bool SyncNTP_to_RTC() {
  if (WiFi.status() != WL_CONNECTED) return false;

  bool horaSincronizada = false;

  for (uint8_t i = 0; NTP_SERVERS[i] != nullptr && !horaSincronizada; ++i) {
    Serial.printf("\n⏱️  Probando NTP: %s\n", NTP_SERVERS[i]);
    configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVERS[i]);

    time_t now = 0;
    uint32_t t0 = millis();
    while (now < 24 * 3600 && (millis() - t0) < NTP_TIMEOUT_MS) {
      now = time(nullptr);
      delay(200);
      yield();
    }
    horaSincronizada = (now >= 24 * 3600);
  }

  if (!horaSincronizada) {
    Serial.println("❌ NTP no sincronizado");
    ntpSincronizado = false;
    return false;
  }

  Serial.println("✅ Hora NTP sincronizada");

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("❌ getLocalTime() falló");
    ntpSincronizado = false;
    return false;
  }

  datetime_t dt;
  dt.year   = (uint16_t)(timeinfo.tm_year + 1900);
  dt.month  = (uint8_t)(timeinfo.tm_mon + 1);
  dt.day    = (uint8_t) timeinfo.tm_mday;
  dt.dotw   = (uint8_t) timeinfo.tm_wday;
  dt.hour   = (uint8_t) timeinfo.tm_hour;
  dt.minute = (uint8_t) timeinfo.tm_min;
  dt.second = (uint8_t) timeinfo.tm_sec;

  PCF85063_Set_All(dt);

  ntpSincronizado = true;

  // refresco estado/hora-minuto desde RTC recién ajustado
  RTC_UpdateStatusAndHM();

  return true;
}


// ============================
// EEPROM: HM guardado en 464/465
// ============================
bool EEPROM_LoadHM(uint8_t &h, uint8_t &m) {
  uint8_t hh = 0xFF, mm = 0xFF;
  EEPROM.get(464, hh);
  EEPROM.get(465, mm);

  if (hh > 23 || mm > 59) return false;
  h = hh; m = mm;
  return true;
}

void GuardarHoraMinuto_EEPROM() {
  RTC_UpdateStatusAndHM();

  if (!rtcHoraValida) {
    Serial.println("⚠️ RTC inválido, no se guarda en EEPROM");
    return;
  }

  uint8_t h = (uint8_t)horaActual;
  uint8_t m = (uint8_t)minutoActual;

  uint16_t minutosUp = (uint16_t)(millis() / 60000UL);
  if (minutosUp > 1440) minutosUp = 1440;

  uint8_t oldH     = EEPROM.read(EEPROM_ADDR_HORA_ACTUAL);
  uint8_t oldM     = EEPROM.read(EEPROM_ADDR_MINUTO_ACTUAL);
  uint8_t oldMagic = EEPROM.read(EEPROM_ADDR_MAGIC);

  if (oldH != h || oldM != m || oldMagic != EEPROM_MAGIC_VALUE) {
    EEPROM.write(EEPROM_ADDR_HORA_ACTUAL,   h);
    EEPROM.write(EEPROM_ADDR_MINUTO_ACTUAL, m);
    EEPROM.put(EEPROM_ADDR_MINUTES_UP,      minutosUp);
    EEPROM.write(EEPROM_ADDR_MAGIC,         EEPROM_MAGIC_VALUE);
    EEPROM.commit();
    Serial.printf("💾 EEPROM: %02d:%02d (uptime ~%u min)\n", h, m, minutosUp);
  }
}

static void oledPrintHex2(uint8_t b) {
  const char* h = "0123456789ABCDEF";
  display.print(h[(b >> 4) & 0x0F]);
  display.print(h[(b >> 0) & 0x0F]);
}

static const char* rs485ErrStr(uint8_t e) {
  switch (e) {
    case 0: return "OK";
    case 1: return "TO";   // timeout
    case 2: return "CRC";  // badCRC
    case 3: return "EXC";  // exception
    case 4: return "FMT";  // formato/incompleto
    case 5: return "WID";  // wrong ID
    default:return "ERR";
  }
}

bool isFragmentRequest() {
  return server.hasArg("fragment") && server.arg("fragment") == "1";
}

void GuardadoR1() {
  Serial.println("Guardando R1 en memoria..");

  EEPROM.put(0,   minR1);
  EEPROM.put(4,   maxR1);
  EEPROM.put(16,  (uint8_t)paramR1);
  EEPROM.put(18,  (uint8_t)modoR1);
  EEPROM.put(137, (uint8_t)estadoR1);

  EEPROM.put(276, (int32_t)horaOnR1);
  EEPROM.put(280, (int32_t)horaOffR1);
  EEPROM.put(284, (int32_t)minOnR1);
  EEPROM.put(288, (int32_t)minOffR1);

  EEPROM.put(300, (uint8_t)direccionR1);

  EEPROM.put(320, (int32_t)R1name);
  EEPROM.put(EEPROM_SENSOR_R1, (int32_t)sensorR1);

  EEPROM.commit();
  Serial.println("Guardado R1 realizado con exito.");
}

void GuardadoR2() {
  Serial.println("Guardando R2 en memoria..");

  EEPROM.put(8,   minR2);
  EEPROM.put(12,  maxR2);
  EEPROM.put(17,  (uint8_t)paramR2);
  EEPROM.put(19,  (uint8_t)modoR2);
  EEPROM.put(138, (uint8_t)estadoR2);

  EEPROM.put(330, (int32_t)minTR2);
  EEPROM.put(334, (int32_t)maxTR2);
  EEPROM.put(EEPROM_SENSOR_R2, (int32_t)sensorR2);

  EEPROM.commit();
  Serial.println("Guardado R2 realizado con exito.");
}

void GuardadoR3() {
  Serial.println("Guardando R3 en memoria..");

  EEPROM.put(20,  (uint8_t)modoR3);

  EEPROM.put(21,  (uint16_t)timeOnR3);
  EEPROM.put(23,  (uint16_t)timeOffR3);

  for (int y = 0; y < 7; y++) {
    int p = 30 + y;
    EEPROM.put(p, (uint8_t)diasRiego[y]);
  }

  EEPROM.put(139, (uint8_t)estadoR3);

  EEPROM.put(141, (int32_t)horaOnR3);
  EEPROM.put(145, (int32_t)minOnR3);
  EEPROM.put(149, (int32_t)horaOffR3);
  EEPROM.put(153, (int32_t)minOffR3);

  EEPROM.put(240, minR3);
  EEPROM.put(245, maxR3);

  EEPROM.put(292, (int32_t)tiempoRiego);
  EEPROM.put(296, (int32_t)tiempoNoRiego);

  EEPROM.put(362, (int32_t)cantidadRiegos);
  EEPROM.put(366, (int32_t)unidadRiego);
  EEPROM.put(370, (int32_t)unidadNoRiego);

  int32_t i32;
  i32 = riegosHechos;
  EEPROM.put(ADDR_RIEGOS_HECHOS, i32);

  i32 = ultimoDiaRiego;
  EEPROM.put(ADDR_ULTIMO_DIA_RIEGO, i32);
  EEPROM.put(EEPROM_SENSOR_R3, (int32_t)sensorR3);

  EEPROM.commit();
  Serial.println("Guardado R3 realizado con exito.");
}

void GuardadoR4() {
  Serial.println("Guardando R4 en memoria..");

  EEPROM.put(25,  (uint8_t)modoR4);

  EEPROM.put(26,  (uint16_t)timeOnR4);
  EEPROM.put(28,  (uint16_t)timeOffR4);

  EEPROM.put(140, (uint8_t)estadoR4);

  EEPROM.put(158, (int32_t)horaOnR4);
  EEPROM.put(162, (int32_t)minOnR4);
  EEPROM.put(166, (int32_t)horaOffR4);
  EEPROM.put(170, (int32_t)minOffR4);

  EEPROM.put(304, (int32_t)currentPosition);
  EEPROM.put(308, (int32_t)horaAmanecer);
  EEPROM.put(312, (int32_t)horaAtardecer);

  EEPROM.put(346, (int32_t)horasLuz);
  EEPROM.put(350, (int32_t)horasOscuridad);

  EEPROM.put(455, (int32_t)supercycleStartEpochR4);

  EEPROM.put(ADDR_VEGE_START,   (uint32_t)vegeStartEpoch);
  EEPROM.put(ADDR_FLORA_START,  (uint32_t)floraStartEpoch);
  EEPROM.put(ADDR_VEGE_DAYS,    (int32_t)vegeDays);
  EEPROM.put(ADDR_FLORA_DAYS,   (int32_t)floraDays);
  EEPROM.put(ADDR_LAST_DATEKEY, (uint32_t)lastDateKey);
  EEPROM.put(ADDR_VEGE_ACTIVE,  (uint8_t)(vegeActive ? 1 : 0));
  EEPROM.put(ADDR_FLORA_ACTIVE, (uint8_t)(floraActive ? 1 : 0));

  EEPROM.commit();
  Serial.println("Guardado R4 realizado con exito.");
}

void GuardadoR5() {
  Serial.println("Guardando R5 en memoria..");

  EEPROM.put(250, minR5);
  EEPROM.put(255, maxR5);
  EEPROM.put(260, (uint8_t)modoR5);

  EEPROM.put(265, (uint8_t)paramR5);
  EEPROM.put(270, (uint8_t)estadoR5);
  EEPROM.put(272, (uint8_t)direccionR5);

  EEPROM.put(324, (int32_t)minOnR5);
  EEPROM.put(460, (int32_t)minOffR5);

  EEPROM.put(338, (int32_t)horaOnR5);
  EEPROM.put(450, (int32_t)horaOffR5);

  EEPROM.put(380, (int32_t)R5name);
  EEPROM.put(EEPROM_SENSOR_R5, (int32_t)sensorR5);

  EEPROM.commit();
  Serial.println("Guardado R5 realizado con exito.");
}

void GuardadoR6() {
  Serial.println("Guardando R6 en memoria..");

  EEPROM.put(500, minR6);
  EEPROM.put(506, maxR6);
  EEPROM.put(512, (uint8_t)modoR6);

  EEPROM.put(515, (uint8_t)paramR6);
  EEPROM.put(518, (uint8_t)estadoR6);
  EEPROM.put(521, (uint8_t)direccionR6);

  EEPROM.put(524, (int32_t)minOnR6);
  EEPROM.put(530, (int32_t)minOffR6);

  EEPROM.put(536, (int32_t)horaOnR6);
  EEPROM.put(542, (int32_t)horaOffR6);

  EEPROM.put(548, (int32_t)R6name);
  EEPROM.put(EEPROM_SENSOR_R6, (int32_t)sensorR6);

  EEPROM.commit();
  Serial.println("Guardado R6 realizado con exito.");
}

void GuardadoSensoresConfig() {
  Serial.println("Guardando configuracion de sensores en memoria..");

  sensorR1 = normalizarCodigoSensoresAmbiente(sensorR1);
  sensorR2 = normalizarCodigoSensoresAmbiente(sensorR2);
  sensorR3 = normalizarCodigoSensoresSuelo(sensorR3);
  sensorR5 = normalizarCodigoSensoresAmbiente(sensorR5);
  sensorR6 = normalizarCodigoSensoresAmbiente(sensorR6);
  sensorR8 = normalizarCodigoSensoresSuelo(sensorR8);

  EEPROM.put(EEPROM_SENSOR_R1, (int32_t)sensorR1);
  EEPROM.put(EEPROM_SENSOR_R2, (int32_t)sensorR2);
  EEPROM.put(EEPROM_SENSOR_R3, (int32_t)sensorR3);
  EEPROM.put(EEPROM_SENSOR_R5, (int32_t)sensorR5);
  EEPROM.put(EEPROM_SENSOR_R6, (int32_t)sensorR6);
  EEPROM.put(EEPROM_SENSOR_R8, (int32_t)sensorR8);

  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 0, (uint8_t)(sensorAir1Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 1, (uint8_t)(sensorAir2Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 2, (uint8_t)(sensorAir3Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_AIR_ACTIVE + 3, (uint8_t)(sensorAir4Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_SOIL_ACTIVE + 0, (uint8_t)(sensorSoil5Activo ? 1 : 0));
  EEPROM.put(EEPROM_SENSOR_SOIL_ACTIVE + 1, (uint8_t)(sensorSoil6Activo ? 1 : 0));

  EEPROM.commit();
  Serial.println("Guardado de sensores realizado con exito.");
}

void GuardadoR7() {
  Serial.println("Guardando R7 en memoria..");

  timeOnR7 = horaOnR7 * 60 + minOnR7;
  timeOffR7 = horaOffR7 * 60 + minOffR7;

  EEPROM.put(EEPROM_R7_MODO, (uint8_t)modoR7);
  EEPROM.put(EEPROM_R7_TIME_ON, (uint16_t)timeOnR7);
  EEPROM.put(EEPROM_R7_TIME_OFF, (uint16_t)timeOffR7);
  EEPROM.put(EEPROM_R7_ESTADO, (uint8_t)estadoR7);

  EEPROM.put(EEPROM_R7_HORA_ON, (int32_t)horaOnR7);
  EEPROM.put(EEPROM_R7_MIN_ON, (int32_t)minOnR7);
  EEPROM.put(EEPROM_R7_HORA_OFF, (int32_t)horaOffR7);
  EEPROM.put(EEPROM_R7_MIN_OFF, (int32_t)minOffR7);

  EEPROM.put(EEPROM_R7_HORA_AMANECER, (int32_t)horaAmanecerR7);
  EEPROM.put(EEPROM_R7_HORA_ATARDECER, (int32_t)horaAtardecerR7);
  EEPROM.put(EEPROM_R7_HORAS_LUZ, (int32_t)horasLuzR7);
  EEPROM.put(EEPROM_R7_HORAS_OSCURIDAD, (int32_t)horasOscuridadR7);
  EEPROM.put(EEPROM_R7_SUPER_START, (int32_t)supercycleStartEpochR7);
  EEPROM.put(EEPROM_R7_NAME, (int32_t)R7name);

  EEPROM.commit();
  Serial.println("Guardado R7 realizado con exito.");
}

void GuardadoR8() {
  Serial.println("Guardando R8 en memoria..");

  timeOnR8 = horaOnR8 * 60 + minOnR8;
  timeOffR8 = horaOffR8 * 60 + minOffR8;

  EEPROM.put(EEPROM_R8_MODO, (uint8_t)modoR8);
  EEPROM.put(EEPROM_R8_TIME_ON, (uint16_t)timeOnR8);
  EEPROM.put(EEPROM_R8_TIME_OFF, (uint16_t)timeOffR8);

  for (int y = 0; y < 7; y++) {
    EEPROM.put(EEPROM_R8_DIAS + y, (uint8_t)diasRiegoR8[y]);
  }

  EEPROM.put(EEPROM_R8_ESTADO, (uint8_t)estadoR8);
  EEPROM.put(EEPROM_R8_HORA_ON, (int32_t)horaOnR8);
  EEPROM.put(EEPROM_R8_MIN_ON, (int32_t)minOnR8);
  EEPROM.put(EEPROM_R8_HORA_OFF, (int32_t)horaOffR8);
  EEPROM.put(EEPROM_R8_MIN_OFF, (int32_t)minOffR8);

  EEPROM.put(EEPROM_R8_MIN, minR8);
  EEPROM.put(EEPROM_R8_MAX, maxR8);
  EEPROM.put(EEPROM_R8_TIEMPO_RIEGO, (int32_t)tiempoRiegoR8);
  EEPROM.put(EEPROM_R8_TIEMPO_NO_RIEGO, (int32_t)tiempoNoRiegoR8);
  EEPROM.put(EEPROM_R8_CANTIDAD, (int32_t)cantidadRiegosR8);
  EEPROM.put(EEPROM_R8_UNIDAD_RIEGO, (int32_t)unidadRiegoR8);
  EEPROM.put(EEPROM_R8_UNIDAD_NO_RIEGO, (int32_t)unidadNoRiegoR8);
  EEPROM.put(EEPROM_R8_RIEGOS_HECHOS, (int32_t)riegosHechosR8);
  EEPROM.put(EEPROM_R8_ULTIMO_DIA, (int32_t)ultimoDiaRiegoR8);
  EEPROM.put(EEPROM_R8_NAME, (int32_t)R8name);
  EEPROM.put(EEPROM_SENSOR_R8, (int32_t)sensorR8);

  EEPROM.commit();
  Serial.println("Guardado R8 realizado con exito.");
}

void GuardadoConfig() {
  Serial.println("Guardando configuracion en memoria..");

  writeStringToEEPROM(37,  ssid);
  writeStringToEEPROM(87,  password);
  writeStringToEEPROM(215, chat_id);

  EEPROM.put(316, (uint8_t)modoWiFi);

  EEPROM.put(354, (int32_t)tiempoGoogle);
  EEPROM.put(358, (int32_t)tiempoTelegram);

  GuardarHoraMinuto_EEPROM();

  EEPROM.commit();
  Serial.println("Guardado de configuracion realizado con exito.");
}

bool isSoilSensorAlive() {
  return (millis() - lastSoilRead) < SOIL_TIMEOUT;
}

void initSensoresVirtuales() {
  // Limpiar estados
  for (uint8_t i = 0; i < 7; i++) {
    sensorVirtualTH[i].activo = false;
    sensorVirtualTH[i].cantidad = 0;
    for (uint8_t j = 0; j < 4; j++) sensorVirtualTH[i].ids[j] = 0;

    sensorVirtualSoil[i].activo = false;
    sensorVirtualSoil[i].cantidad = 0;
    for (uint8_t j = 0; j < 2; j++) sensorVirtualSoil[i].ids[j] = 0;

    virtualTemp[i] = NAN;
    virtualHum[i] = NAN;
    virtualSoilTemp[i] = NAN;
    virtualSoilHum[i] = NAN;
    virtualSoilEC[i] = NAN;
    virtualTH_OK[i] = false;
    virtualSoil_OK[i] = false;
  }

  // =====================================================
  // CONFIGURACION INICIAL SIMPLE
  // =====================================================
  // Virtual 1 = físico 1
  sensorVirtualTH[1].activo = true;
  sensorVirtualTH[1].cantidad = 1;
  sensorVirtualTH[1].ids[0] = 1;

  // Virtual 2 = físico 2
  sensorVirtualTH[2].activo = true;
  sensorVirtualTH[2].cantidad = 1;
  sensorVirtualTH[2].ids[0] = 2;

  // Virtual 3 = físico 3
  sensorVirtualTH[3].activo = true;
  sensorVirtualTH[3].cantidad = 1;
  sensorVirtualTH[3].ids[0] = 3;

  // Virtual 4 = físico 4
  sensorVirtualTH[4].activo = true;
  sensorVirtualTH[4].cantidad = 1;
  sensorVirtualTH[4].ids[0] = 4;

  // Virtual 5 = promedio de 1 y 2
  sensorVirtualTH[5].activo = true;
  sensorVirtualTH[5].cantidad = 2;
  sensorVirtualTH[5].ids[0] = 1;
  sensorVirtualTH[5].ids[1] = 2;

  // Virtual 6 = promedio de 1, 2 y 3
  sensorVirtualTH[6].activo = true;
  sensorVirtualTH[6].cantidad = 3;
  sensorVirtualTH[6].ids[0] = 1;
  sensorVirtualTH[6].ids[1] = 2;
  sensorVirtualTH[6].ids[2] = 3;

  // =====================================================
  // SUELO
  // =====================================================
  // Virtual 1 = físico 5
  sensorVirtualSoil[1].activo = true;
  sensorVirtualSoil[1].cantidad = 1;
  sensorVirtualSoil[1].ids[0] = 5;

  // Virtual 2 = físico 6
  sensorVirtualSoil[2].activo = true;
  sensorVirtualSoil[2].cantidad = 1;
  sensorVirtualSoil[2].ids[0] = 6;

  // Virtual 3 = promedio de 5 y 6
  sensorVirtualSoil[3].activo = true;
  sensorVirtualSoil[3].cantidad = 2;
  sensorVirtualSoil[3].ids[0] = 5;
  sensorVirtualSoil[3].ids[1] = 6;
}

bool getAirFisicoById(uint8_t id, float &t, float &h) {
  switch (id) {
    case 1:
      t = temperatura1; h = humedad1;
      return sensorAmbOK1;
    case 2:
      t = temperatura2; h = humedad2;
      return sensorAmbOK2;
    case 3:
      t = temperatura3; h = humedad3;
      return sensorAmbOK3;
    case 4:
      t = temperatura4; h = humedad4;
      return sensorAmbOK4;
    default:
      t = NAN; h = NAN;
      return false;
  }
}

bool getSoilFisicoById(uint8_t id, float &t, float &h, float &ec) {
  switch (id) {
    case 5:
      t = temperaturaSuelo5; h = humedadSuelo5; ec = ECSuelo5;
      return sensorSueloOK5;
    case 6:
      t = temperaturaSuelo6; h = humedadSuelo6; ec = ECSuelo6;
      return sensorSueloOK6;
    default:
      t = NAN; h = NAN; ec = NAN;
      return false;
  }
}

void calcularSensoresVirtualesTH() {
  for (uint8_t i = 1; i < 7; i++) {
    virtualTemp[i] = NAN;
    virtualHum[i] = NAN;
    virtualTH_OK[i] = false;

    if (!sensorVirtualTH[i].activo || sensorVirtualTH[i].cantidad == 0) {
      continue;
    }

    float sumaT = 0.0f;
    float sumaH = 0.0f;
    uint8_t validos = 0;

    for (uint8_t j = 0; j < sensorVirtualTH[i].cantidad; j++) {
      float t = NAN, h = NAN;
      uint8_t idFisico = sensorVirtualTH[i].ids[j];

      if (getAirFisicoById(idFisico, t, h) && isfinite(t) && isfinite(h)) {
        sumaT += t;
        sumaH += h;
        validos++;
      }
    }

    if (validos > 0) {
      virtualTemp[i] = sumaT / validos;
      virtualHum[i]  = sumaH / validos;
      virtualTH_OK[i] = true;
    }
  }
}

void calcularSensoresVirtualesSoil() {
  for (uint8_t i = 1; i < 7; i++) {
    virtualSoilTemp[i] = NAN;
    virtualSoilHum[i]  = NAN;
    virtualSoilEC[i]   = NAN;
    virtualSoil_OK[i]  = false;

    if (!sensorVirtualSoil[i].activo || sensorVirtualSoil[i].cantidad == 0) {
      continue;
    }

    float sumaT = 0.0f;
    float sumaH = 0.0f;
    float sumaEC = 0.0f;
    uint8_t validos = 0;

    for (uint8_t j = 0; j < sensorVirtualSoil[i].cantidad; j++) {
      float t = NAN, h = NAN, ec = NAN;
      uint8_t idFisico = sensorVirtualSoil[i].ids[j];

      if (getSoilFisicoById(idFisico, t, h, ec) && isfinite(t) && isfinite(h) && isfinite(ec)) {
        sumaT += t;
        sumaH += h;
        sumaEC += ec;
        validos++;
      }
    }

    if (validos > 0) {
      virtualSoilTemp[i] = sumaT / validos;
      virtualSoilHum[i]  = sumaH / validos;
      virtualSoilEC[i]   = sumaEC / validos;
      virtualSoil_OK[i]  = true;
    }
  }
}


bool resolverSensorVirtualAmbiente(int codigo, float &t, float &h) {
  t = NAN;
  h = NAN;

  int tempCodigo = codigo;
  float sumaT = 0.0f;
  float sumaH = 0.0f;
  int cantidad = 0;

  while (tempCodigo > 0) {
    int id = tempCodigo % 10;
    tempCodigo /= 10;

    // Solo IDs de ambiente
    if (id < 1 || id > 4) continue;

    float tf = NAN, hf = NAN;
    if (getAirFisicoById((uint8_t)id, tf, hf) && isfinite(tf) && isfinite(hf)) {
      sumaT += tf;
      sumaH += hf;
      cantidad++;
    }
  }

  if (cantidad == 0) return false;

  t = sumaT / cantidad;
  h = sumaH / cantidad;
  return true;
}

bool resolverSensorVirtualSuelo(int codigo, float &t, float &h, float &ec) {
  t = NAN;
  h = NAN;
  ec = NAN;

  int tempCodigo = codigo;
  float sumaT = 0.0f;
  float sumaH = 0.0f;
  float sumaEC = 0.0f;
  int cantidad = 0;

  while (tempCodigo > 0) {
    int id = tempCodigo % 10;
    tempCodigo /= 10;

    // Solo IDs de suelo
    if (id < 5 || id > 6) continue;

    float tf = NAN, hf = NAN, ecf = NAN;
    if (getSoilFisicoById((uint8_t)id, tf, hf, ecf) &&
        isfinite(tf) && isfinite(hf) && isfinite(ecf)) {
      sumaT += tf;
      sumaH += hf;
      sumaEC += ecf;
      cantidad++;
    }
  }

  if (cantidad == 0) return false;

  t  = sumaT / cantidad;
  h  = sumaH / cantidad;
  ec = sumaEC / cantidad;
  return true;
}

int normalizarCodigoSensoresAmbiente(int codigo) {
  int out = 0;
  bool usado[5] = {false, false, false, false, false};

  while (codigo > 0) {
    int id = codigo % 10;
    codigo /= 10;

    if (id < 1 || id > 4 || usado[id]) continue;
    usado[id] = true;
  }

  for (int id = 1; id <= 4; id++) {
    if (usado[id]) out = out * 10 + id;
  }

  return (out > 0) ? out : 1;
}

int normalizarCodigoSensoresSuelo(int codigo) {
  bool usa5 = false;
  bool usa6 = false;

  while (codigo > 0) {
    int id = codigo % 10;
    codigo /= 10;

    if (id == 5) usa5 = true;
    if (id == 6) usa6 = true;
  }

  if (usa5 && usa6) return 56;
  if (usa6) return 6;
  return 5;
}

int parseIntFlexible(String value) {
  value.trim();
  if (value.length() > 0 && (value[0] == 'r' || value[0] == 'R' || value[0] == 's' || value[0] == 'S')) {
    value.remove(0, 1);
  }
  return value.toInt();
}

bool riegoPermitidoPorSuelo(bool sensorOK, float humedadSuelo, int minH, int maxH, bool regandoAhora) {
  if (minH <= 0 && maxH <= 0) return true;
  if (!sensorOK || !isfinite(humedadSuelo)) return false;

  if (maxH > 0 && humedadSuelo >= maxH) return false;
  if (minH <= 0) return true;
  if (humedadSuelo <= minH) return true;

  return regandoAhora;
}

String sensorCodeLabel(int code) {
  String s = "";
  int divisor = 1;
  int temp = code;
  while (temp >= 10) {
    divisor *= 10;
    temp /= 10;
  }

  while (divisor > 0) {
    int digit = (code / divisor) % 10;
    if (digit > 0) {
      if (s.length() > 0) s += "+";
      s += String(digit);
    }
    divisor /= 10;
  }

  return s.length() ? s : "--";
}

static String jsonStr(const String& s) {
  String out = "\"";
  for (char c : s) {
    if (c == '"')  { out += "\\\""; }
    else if (c == '\\') { out += "\\\\"; }
    else { out += c; }
  }
  out += "\"";
  return out;
}
 
static String jsonBool(bool v) { return v ? "true" : "false"; }
static String jsonNum(int v)   { return String(v); }
static String jsonFlt(float v, int dec = 1) { return isfinite(v) ? String(v, dec) : "null"; }
 

 
// SECCION DE ACTUALIZACION REMOTA OTA: ACTUALIZACION DE FRONTEND Y BACKEND

void mostrarOLED_OTA(const String& linea1,
                     const String& linea2 = "",
                     const String& linea3 = "",
                     const String& linea4 = "") {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println(linea1);

  if (linea2.length()) {
    display.setCursor(0, 16);
    display.println(linea2);
  }

  if (linea3.length()) {
    display.setCursor(0, 32);
    display.println(linea3);
  }

  if (linea4.length()) {
    display.setCursor(0, 48);
    display.println(linea4);
  }

  display.display();
}
 
// ── Main endpoint ──

void doOTAUpdateFirmware() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA FW] ERROR: No hay WiFi.");
    mostrarOLED_OTA("OTA BACKEND", "ERROR:", "Sin WiFi");
    delay(2000);
    return;
  }

  Serial.println("[OTA FW] Actualizando firmware...");
  Serial.print("[OTA FW] URL: ");
  Serial.println(OTA_FIRMWARE_URL);

  mostrarOLED_OTA("OTA BACKEND", "ACTUALIZANDO...", "Descargando FW");

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.rebootOnUpdate(false);

  unsigned long t0 = millis();

  t_httpUpdate_return ret = httpUpdate.update(client, OTA_FIRMWARE_URL);

  unsigned long dt = (millis() - t0) / 1000UL;

  switch (ret) {
    case HTTP_UPDATE_FAILED: {
      int err = httpUpdate.getLastError();
      String errStr = httpUpdate.getLastErrorString();

      Serial.printf("[OTA FW] ERROR (%d): %s\n", err, errStr.c_str());

      mostrarOLED_OTA(
        "OTA BACKEND",
        "ERROR",
        "Codigo: " + String(err),
        errStr.substring(0, 20)
      );

      delay(5000);
      break;
    }

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA FW] Sin actualización.");

      mostrarOLED_OTA(
        "OTA BACKEND",
        "Sin actualizacion",
        "Tiempo: " + String(dt) + "s"
      );

      delay(3000);
      break;

    case HTTP_UPDATE_OK:
      Serial.println("[OTA FW] OK. Reiniciando...");

      mostrarOLED_OTA(
        "OTA BACKEND",
        "OK",
        "Tiempo: " + String(dt) + "s",
        "Reiniciando..."
      );

      delay(1500);
      ESP.restart();
      break;
  }
}



void doOTAUpdateFFat() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA FFat] ERROR: No hay WiFi.");
    mostrarOLED_OTA("OTA FRONTEND", "ERROR:", "Sin WiFi");
    delay(2000);
    return;
  }

  Serial.println("[OTA FFat] Actualizando archivos webApp...");
  Serial.print("[OTA FFat] URL: ");
  Serial.println(OTA_FFAT_URL);

  mostrarOLED_OTA("OTA FRONTEND", "ACTUALIZANDO...", "Cerrando FFat");

  // CLAVE: desmontar FFat antes de escribir la partición
  FFat.end();
  delay(300);

  mostrarOLED_OTA("OTA FRONTEND", "ACTUALIZANDO...", "Descargando FFat");

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.rebootOnUpdate(false);

  unsigned long t0 = millis();

  t_httpUpdate_return ret = httpUpdate.updateFatfs(client, OTA_FFAT_URL);

  unsigned long dt = (millis() - t0) / 1000UL;

  switch (ret) {
    case HTTP_UPDATE_FAILED: {
      int err = httpUpdate.getLastError();
      String errStr = httpUpdate.getLastErrorString();

      Serial.printf("[OTA FFat] ERROR (%d): %s\n", err, errStr.c_str());

      mostrarOLED_OTA(
        "OTA FRONTEND",
        "ERROR",
        "Codigo: " + String(err),
        errStr.substring(0, 20)
      );

      delay(5000);
      ESP.restart();  // reinicio para remontar FFat limpio
      break;
    }

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA FFat] Sin actualización.");

      mostrarOLED_OTA(
        "OTA FRONTEND",
        "Sin actualizacion",
        "Tiempo: " + String(dt) + "s",
        "Reiniciando..."
      );

      delay(2000);
      ESP.restart();
      break;

    case HTTP_UPDATE_OK:
      Serial.println("[OTA FFat] Actualización correcta. Reiniciando...");

      mostrarOLED_OTA(
        "OTA FRONTEND",
        "OK",
        "Tiempo: " + String(dt) + "s",
        "Reiniciando..."
      );

      delay(1500);
      ESP.restart();
      break;
  }
}

bool doOTAUpdateFFatNoRestart() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA FFat] ERROR: No hay WiFi.");
    mostrarOLED_OTA("OTA GENERAL", "ERROR:", "Sin WiFi");
    delay(2000);
    return false;
  }

  Serial.println("[OTA FFat] Actualizando archivos webApp SIN reinicio...");
  Serial.print("[OTA FFat] URL: ");
  Serial.println(OTA_FFAT_URL);

  mostrarOLED_OTA("OTA GENERAL", "Paso 1/2", "Frontend FFat", "Cerrando FFat");

  FFat.end();
  delay(300);

  mostrarOLED_OTA("OTA GENERAL", "Paso 1/2", "Descargando FFat");

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.rebootOnUpdate(false);

  unsigned long t0 = millis();

  t_httpUpdate_return ret = httpUpdate.updateFatfs(client, OTA_FFAT_URL);

  unsigned long dt = (millis() - t0) / 1000UL;

  switch (ret) {
    case HTTP_UPDATE_FAILED: {
      int err = httpUpdate.getLastError();
      String errStr = httpUpdate.getLastErrorString();

      Serial.printf("[OTA FFat] ERROR (%d): %s\n", err, errStr.c_str());

      mostrarOLED_OTA(
        "OTA GENERAL",
        "FFat ERROR",
        "Codigo: " + String(err),
        errStr.substring(0, 20)
      );

      delay(5000);
      return false;
    }

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[OTA FFat] Sin actualización.");

      mostrarOLED_OTA(
        "OTA GENERAL",
        "FFat sin cambios",
        "Tiempo: " + String(dt) + "s"
      );

      delay(1500);
      return true;

    case HTTP_UPDATE_OK:
      Serial.println("[OTA FFat] Actualización correcta. Continúa sin reiniciar.");

      mostrarOLED_OTA(
        "OTA GENERAL",
        "FFat OK",
        "Tiempo: " + String(dt) + "s",
        "Sigue Backend"
      );

      delay(1500);
      return true;
  }

  return false;
}

void doOTAUpdateAll() {
  mostrarOLED_OTA("OTA GENERAL", "Iniciando...", "Frontend + Backend");

  bool ffatOk = doOTAUpdateFFatNoRestart();

  if (!ffatOk) {
    Serial.println("[OTA ALL] FFat falló. Cancelo firmware.");

    mostrarOLED_OTA(
      "OTA GENERAL",
      "ERROR",
      "Fallo Frontend",
      "Cancelo Backend"
    );

    delay(5000);
    ESP.restart();  // importante: FFat quedó desmontado
    return;
  }

  delay(1000);

  mostrarOLED_OTA(
    "OTA GENERAL",
    "Paso 2/2",
    "Actualizando Backend"
  );

  doOTAUpdateFirmware();
}


void preloadFilesToRAM() {
  File f;

  f = FFat.open("/index.html", "r");
  if (f) { cachedHTML = f.readString(); f.close(); Serial.println("index.html cargado en RAM"); }
  else     Serial.println("ERROR: index.html no encontrado");

  f = FFat.open("/style.css", "r");
  if (f) { cachedCSS = f.readString(); f.close(); Serial.println("style.css cargado en RAM"); }
  else     Serial.println("ERROR: style.css no encontrado");

  f = FFat.open("/app.js", "r");
  if (f) { cachedJS = f.readString(); f.close(); Serial.println("app.js cargado en RAM"); }
  else     Serial.println("ERROR: app.js no encontrado");
}
