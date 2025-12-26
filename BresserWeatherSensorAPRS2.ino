///////////////////////////////////////////////////////////////////////////////////////////////////
// Bresser_APRS_Final.ino
// 
// Integracja: Bresser 7-in-1 + ESP8266 + BMP280 + CC1101 -> APRS (TCP)
// Biblioteka: BresserWeatherSensorReceiver v0.37.0
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

// --- NAPRAWA BŁĘDU LINKERA ---
// Biblioteka Preferences (używana przez Bresser v0.37.0) wymaga LittleFS na ESP8266
#include <LittleFS.h> 

// Dołączamy Twój działający plik konfiguracyjny
#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"

// ==========================================================================
// --- KONFIGURACJA APRS I WIFI (UZUPEŁNIJ SWOJE DANE) ----------------------
static const char* WIFI_SSID     = "NAZWA_SIECI";    // <-- Wpisz nazwę WiFi
static const char* WIFI_PASS     = "HASLO_SIECI";    // <-- Wpisz hasło WiFi
static const char* APRS_CALLSIGN = "SP0ABC-13";      // <-- Twój znak i SSID
static const char* APRS_PASSCODE = "12345";          // <-- Twój Passcode APRS
static const double SITE_LAT     = 52.1234;          // <-- Szerokość
static const double SITE_LON     = 16.1234;          // <-- Długość 
// ==========================================================================

// Ustawienia serwera APRS
static const char* APRS_HOST = "euro.aprs2.net";
static const uint16_t APRS_PORT = 14580;
static const char* APRS_SOFTWARE_NAME = "RodosWX_2";

// Częstotliwość wysyłania (15 minut)
static const unsigned long REPORT_INTERVAL_MS = 15UL * 60UL * 1000UL; 

// Obiekty
WeatherSensor ws;
Adafruit_BME280 bmp;
bool bmp_available = false;

// Zmienne do uśredniania wiatru i porywów (resetowane co 15 min)
float wind_speed_sum = 0.0;
int wind_sample_count = 0;
float wind_gust_max_period = 0.0;

// Zmienne przechowujące ostatnie poprawne odczyty
struct WeatherData {
  float temp_c = NAN;
  uint8_t humidity = 0;
  float wind_dir = NAN;
  float rain_total_mm = NAN;
  float uv_index = NAN;
  float light_klx = NAN;
  bool valid_data = false;
} current_wx;

// Bufor kołowy do obliczania deszczu z ostatniej godziny (4 sloty po 15 min)
struct RainHistory {
  float total_mm;
  bool valid;
};
RainHistory rain_buffer[4]; 
uint8_t rain_idx = 0;

unsigned long last_report_time = 0;

// --- FUNKCJE POMOCNICZE (KONWERSJE) ---
int convert_c_to_f(float c) { return (int)lround(c * 1.8 + 32); }
int convert_ms_to_mph(float ms) { return (int)lround(ms * 2.23694); }
int convert_mm_to_hun_inch(float mm) { return (int)lround(mm * 3.93701); }

// Formatowanie szerokości geograficznej dla APRS
String format_lat(double lat) {
  char buff[20];
  char hemi = (lat >= 0) ? 'N' : 'S';
  lat = fabs(lat);
  int deg = (int)lat;
  double min = (lat - deg) * 60.0;
  snprintf(buff, sizeof(buff), "%02d%05.2f%c", deg, min, hemi);
  return String(buff);
}

// Formatowanie długości geograficznej dla APRS
String format_lon(double lon) {
  char buff[20];
  char hemi = (lon >= 0) ? 'E' : 'W';
  lon = fabs(lon);
  int deg = (int)lon;
  double min = (lon - deg) * 60.0;
  snprintf(buff, sizeof(buff), "%03d%05.2f%c", deg, min, hemi);
  return String(buff);
}

// Formatowanie 3 cyfr z zerami wiodącymi (np. wiatr)
String pad3(int val) {
  if (val < 0) val = 0;
  if (val > 999) val = 999;
  char buff[5];
  snprintf(buff, sizeof(buff), "%03d", val);
  return String(buff);
}

// Generowanie czasu DDHHMMz
String get_aprs_timestamp() {
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buff[10];
  snprintf(buff, sizeof(buff), "%02d%02d%02dz", t->tm_mday, t->tm_hour, t->tm_min);
  return String(buff);
}

// --- WYSYŁKA APRS ---
void send_aprs_frame() {
  Serial.println("\n[APRS] Przygotowanie danych do wysyłki...");

  // 1. Obliczenia wiatru (Średnia i Poryw z 15 min)
  float avg_wind_speed = 0.0;
  if (wind_sample_count > 0) {
    avg_wind_speed = wind_speed_sum / wind_sample_count;
  }
  float gust_speed = wind_gust_max_period;
  // Jeśli brak próbek, bierzemy ostatnie znane (lub 0)
  if (wind_sample_count == 0 && !isnan(current_wx.wind_dir)) {
      avg_wind_speed = 0; 
      gust_speed = 0;
  }

  // 2. Obliczenia deszczu (Delta 1h)
  int rain_1h_hin = 0;
  // Zapisz obecny stan licznika deszczu do historii
  if (!isnan(current_wx.rain_total_mm)) {
      rain_buffer[rain_idx].total_mm = current_wx.rain_total_mm;
      rain_buffer[rain_idx].valid = true;
  }
  
  // Oblicz różnicę: Obecny - (Ten sprzed 1h, czyli następny w buforze kołowym)
  uint8_t old_idx = (rain_idx + 1) % 4;
  if (rain_buffer[rain_idx].valid && rain_buffer[old_idx].valid) {
      float diff = rain_buffer[rain_idx].total_mm - rain_buffer[old_idx].total_mm;
      if (diff < 0) diff = 0; // Zabezpieczenie przed ujemnym (np. wymiana baterii)
      rain_1h_hin = convert_mm_to_hun_inch(diff);
  }
  // Przesuń indeks
  rain_idx = (rain_idx + 1) % 4;

  // 3. Odczyt BME280
  int baro_val = 0;
  if (bmp_available) {
    bmp.takeForcedMeasurement();
    float pres_pa = bmp.readPressure();
    // APRS format: dziesiąte części hPa (np. 1013.2 hPa -> 10132)
    baro_val = (int)(pres_pa / 10.0); 
  }

  // --- BUDOWANIE PAKIETU ---
  String lat_str = format_lat(SITE_LAT);
  String lon_str = format_lon(SITE_LON);
  String timestamp = get_aprs_timestamp();
  
  // Kierunek wiatru (z ostatniego odczytu)
  int wind_dir_int = (int)current_wx.wind_dir;
  if (wind_dir_int <= 0 || wind_dir_int > 360) wind_dir_int = 0;
  if (wind_dir_int == 0 && avg_wind_speed > 0) wind_dir_int = 360; // Północ

  String body = "@" + timestamp + lat_str + "/" + lon_str + "_";
  body += pad3(wind_dir_int) + "/" + pad3(convert_ms_to_mph(avg_wind_speed));
  body += "g" + pad3(convert_ms_to_mph(gust_speed));
  body += "t" + pad3(convert_c_to_f(current_wx.temp_c));
  
  if (rain_1h_hin > 0) body += "r" + pad3(rain_1h_hin);
  
  // Wilgotność
  if (current_wx.humidity > 0) {
      char hum_b[5]; snprintf(hum_b, sizeof(hum_b), "h%02d", current_wx.humidity);
      body += hum_b;
  }
  
  // Ciśnienie
  if (baro_val > 0) {
      char baro_b[7]; snprintf(baro_b, sizeof(baro_b), "b%05d", baro_val);
      body += baro_b;
  }

  // Luminancja (Lxxx) w W/m^2. Przybliżenie: Lux * 0.0079
  if (!isnan(current_wx.light_klx)) {
      int wm2 = (int)(current_wx.light_klx * 1000.0 * 0.0079);
      if (wm2 >= 1000) wm2 = 999;
      char lum_b[6]; snprintf(lum_b, sizeof(lum_b), "L%03d", wm2);
      body += lum_b;
  }

  // Komentarz + Telemetria
  String comment = "";
  
  // Status baterii
  if (ws.sensor[0].valid) {
      comment += ws.sensor[0].battery_ok ? " Bat:OK" : " Bat:LOW";
  }
  
  if (!isnan(current_wx.uv_index)) {
      comment += " UV:" + String(current_wx.uv_index, 1);
  }

  // Dopisek z instrukcji
  comment += " RodosWX_2";

  String packet = String(APRS_CALLSIGN) + ">APRS,TCPIP*:" + body + comment;

  // Wysyłanie TCP
  WiFiClient client;
  if (client.connect(APRS_HOST, APRS_PORT)) {
    client.print("user " + String(APRS_CALLSIGN) + " pass " + String(APRS_PASSCODE) + " vers " + String(APRS_SOFTWARE_NAME) + "\n");
    delay(500);
    client.print(packet + "\n");
    client.flush();
    delay(500);
    client.stop();
    Serial.println("-> Wysłano pakiet: " + packet);
    
    // Reset liczników po wysyłce
    wind_speed_sum = 0;
    wind_sample_count = 0;
    wind_gust_max_period = 0;
    
  } else {
    Serial.println("! Błąd połączenia z serwerem APRS");
  }
}

// --- POŁĄCZENIE WIFI ---
void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Łączenie z WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK!");
  
  // Konfiguracja czasu (potrzebna do timestampu APRS)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Czekam na czas NTP...");
  while (time(nullptr) < 100000) {
      delay(100);
      Serial.print(".");
  }
  Serial.println(" Czas OK");
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- START STACJI ---");

  // Inicjalizacja systemu plików (dla Preferences)
  LittleFS.begin(); 

  // 1. Inicjalizacja I2C BME280
Wire.begin(2, 5);   // SDA = GPIO2, SCL = GPIO5
delay(200);

if (bmp.begin(0x76)) {
    Serial.println("BME280: OK");
    bmp_available = true;

    bmp.setSampling(
        Adafruit_BME280::MODE_NORMAL,   // NAJSTABILNIEJSZY
        Adafruit_BME280::SAMPLING_X2,    // temp
        Adafruit_BME280::SAMPLING_X16,   // press
        Adafruit_BME280::SAMPLING_X1,    // hum (może być 1)
        Adafruit_BME280::FILTER_X16,
        Adafruit_BME280::STANDBY_MS_500
    );
} else {
    Serial.println("BME280: BRAK (I2C)");
}

  // 2. Radio
  Serial.println("Inicjalizacja Radia...");
  ws.begin();

  // 3. WiFi
  setup_wifi();
  
  // Inicjalizacja bufora deszczu
  for(int i=0; i<4; i++) { rain_buffer[i].valid = false; rain_buffer[i].total_mm = 0; }
}

void loop() {
  // Odbiór danych z radia
  ws.clearSlots();
  int decode_status = ws.getMessage();

  if (decode_status == DECODE_OK) {
      Serial.println("[RADIO] Odebrano ramkę!");
      
      if (ws.sensor[0].valid) {
          // Zapisz dane do struktury
          if (ws.sensor[0].w.temp_ok) current_wx.temp_c = ws.sensor[0].w.temp_c;
          if (ws.sensor[0].w.humidity_ok) current_wx.humidity = ws.sensor[0].w.humidity;
          if (ws.sensor[0].w.rain_ok) current_wx.rain_total_mm = ws.sensor[0].w.rain_mm;
          
          #if defined BRESSER_6_IN_1 || defined BRESSER_7_IN_1
            if (ws.sensor[0].w.uv_ok) current_wx.uv_index = ws.sensor[0].w.uv;
          #endif
          #ifdef BRESSER_7_IN_1
            if (ws.sensor[0].w.light_ok) current_wx.light_klx = ws.sensor[0].w.light_klx;
          #endif

          // Logika wiatru
          if (ws.sensor[0].w.wind_ok) {
              current_wx.wind_dir = ws.sensor[0].w.wind_direction_deg;
              
              // Pobieramy prędkość i poryw
              float speed = ws.sensor[0].w.wind_avg_meter_sec;
              float gust = ws.sensor[0].w.wind_gust_meter_sec;
              
              Serial.printf("Wind: Spd=%.1f Gust=%.1f Dir=%.0f\n", speed, gust, current_wx.wind_dir);
              
              wind_speed_sum += speed;
              wind_sample_count++;
              
              if (gust > wind_gust_max_period) {
                  wind_gust_max_period = gust;
              }
          }
          current_wx.valid_data = true;
      }
  }

  // Sprawdzenie czasu wysyłki (co 15 min)
  if (millis() - last_report_time >= REPORT_INTERVAL_MS) {
      if (current_wx.valid_data) {
          send_aprs_frame();
      } else {
          Serial.println("Minęło 15 min, ale brak danych z czujnika. Czekam...");
      }
      last_report_time = millis();
  }

  delay(50);
}