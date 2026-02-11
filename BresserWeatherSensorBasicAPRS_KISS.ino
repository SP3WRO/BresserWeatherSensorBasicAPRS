///////////////////////////////////////////////////////////////////////////////////////////////////
// BresserWeatherSensorBasicAPRS_KISS.ino - WERSJA FINALNA
// Rodzaj połączenia: KISS over TCP (share-tnc)
// Teoretycznie działa.
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include <LittleFS.h> 

#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"

// --- KONFIGURACJA SIECI ---
static const char* WIFI_SSID     = "TWOJA_SIEC_WIFI";   // <---- ZMIEŃ NAZWĘ WIFI
static const char* WIFI_PASS     = "TWOJE_HASLO_WIFI";  // <---- ZMIEŃ HASŁO WIFI  

// --- KONFIGURACJA SHARE-TNC ---
static const char* TNC_IP        = "192.168.1.2";  // <--- ZMIEŃ NA IP TNC
static const uint16_t TNC_PORT   = 8001; // <--- ZMIEŃ NA PORT TNC           

// --- KONFIGURACJA APRS ---
static const char* APRS_CALLSIGN = "SP0ABC"; // <--- CALLSIGN
static const int   APRS_SSID     = 13; // <---- SSID (WX - 13)       
static const char* APRS_DEST     = "APRS";   // <--- DESTINATION
static const int   APRS_DEST_SSID= 0;  // <-- NIE RUSZAJ

static const double SITE_LAT     = 52.1234;  // <-- USTAW LOKALIZACJE STACJI        
static const double SITE_LON     = 16.1234;  // <-- USTAW LOKALIZACJE STACJI        
static const unsigned long REPORT_INTERVAL_MS = 15UL * 60UL * 1000UL; 

// ---------------------------------

WeatherSensor ws;
Adafruit_BME280 bme;
bool bme_available = false;

float wind_speed_sum = 0.0;
int wind_sample_count = 0;
float wind_gust_max_period = 0.0;

struct WeatherData {
  float temp_c = NAN;
  uint8_t humidity = 0;
  float wind_dir = NAN;
  float rain_total_mm = NAN;
  float uv_index = NAN;
  float light_klx = NAN; // Tutaj trzymamy surowe Luxy
  int radio_rssi = -100;  
  bool battery_ok = true; 
  bool valid_data = false;
} current_wx;

struct RainHistory { float total_mm; bool valid; };
RainHistory rain_buffer[4]; 
uint8_t rain_idx = 0;
unsigned long last_report_time = 0;

// --- DEFINICJE KISS ---
#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD

// --- KONWERSJE ---
int c_to_f(float c) { return (int)lround(c * 1.8 + 32); }
int ms_to_mph(float ms) { return (int)lround(ms * 2.23694); }
int mm_to_hin(float mm) { return (int)lround(mm * 3.93701); }

String format_lat(double lat) {
  char b[20]; char h = (lat >= 0) ? 'N' : 'S'; lat = fabs(lat);
  int d = (int)lat; double m = (lat - d) * 60.0;
  snprintf(b, sizeof(b), "%02d%05.2f%c", d, m, h);
  return String(b);
}
String format_lon(double lon) {
  char b[20]; char h = (lon >= 0) ? 'E' : 'W'; lon = fabs(lon);
  int d = (int)lon; double m = (lon - d) * 60.0;
  snprintf(b, sizeof(b), "%03d%05.2f%c", d, m, h);
  return String(b);
}
String p3(int v) { char b[5]; snprintf(b, sizeof(b), "%03d", (v<0?0:(v>999?999:v))); return String(b); }

String get_timestamp() {
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buff[10];
  snprintf(buff, sizeof(buff), "%02d%02d%02dz", t->tm_mday, t->tm_hour, t->tm_min);
  return String(buff);
}

// --- FUNKCJE KISS ---
void send_kiss_byte(WiFiClient &client, uint8_t b) {
    if (b == FEND) {
        client.write(FESC);
        client.write(TFEND);
    } else if (b == FESC) {
        client.write(FESC);
        client.write(TFESC);
    } else {
        client.write(b);
    }
}

void send_ax25_frame(const String &payload) {
    WiFiClient client;
    if (!client.connect(TNC_IP, TNC_PORT)) {
        Serial.println(F("BLAD: Brak polaczenia z share-tnc!"));
        return;
    }

    Serial.println(F("[KISS] Wysylanie ramki..."));
    client.write(FEND); 
    client.write(0x00); 

    // DEST
    char dest_padded[7]; memset(dest_padded, ' ', 6);
    for(int i=0; i<strlen(APRS_DEST) && i<6; i++) dest_padded[i] = APRS_DEST[i];
    for(int i=0; i<6; i++) send_kiss_byte(client, dest_padded[i] << 1);
    send_kiss_byte(client, 0x60 | ((APRS_DEST_SSID & 0x0F) << 1)); 

    // SRC
    char src_padded[7]; memset(src_padded, ' ', 6);
    for(int i=0; i<strlen(APRS_CALLSIGN) && i<6; i++) src_padded[i] = APRS_CALLSIGN[i];
    for(int i=0; i<6; i++) send_kiss_byte(client, src_padded[i] << 1);
    send_kiss_byte(client, 0x60 | ((APRS_SSID & 0x0F) << 1)); 
    
    // DIGI (WIDE1-1)
    const char* digi = "WIDE1";
    char digi_padded[7]; memset(digi_padded, ' ', 6);
    for(int i=0; i<strlen(digi) && i<6; i++) digi_padded[i] = digi[i];
    for(int i=0; i<6; i++) send_kiss_byte(client, digi_padded[i] << 1);
    send_kiss_byte(client, 0x60 | (1 << 1) | 0x01); 

    // CTRL/PID
    send_kiss_byte(client, 0x03); 
    send_kiss_byte(client, 0xF0); 

    // PAYLOAD
    for (int i = 0; i < payload.length(); i++) {
        send_kiss_byte(client, (uint8_t)payload[i]);
    }

    client.write(FEND);
    client.stop();
    Serial.println("-> OK: Wyslano KISS");
}

void send_aprs() {
  float avg_w = (wind_sample_count > 0) ? (wind_speed_sum / wind_sample_count) : 0.0;
  
  int r1h = 0;
  if (!isnan(current_wx.rain_total_mm)) rain_buffer[rain_idx] = {current_wx.rain_total_mm, true};
  uint8_t oid = (rain_idx + 1) % 4;
  if (rain_buffer[rain_idx].valid && rain_buffer[oid].valid) {
      float d = rain_buffer[rain_idx].total_mm - rain_buffer[oid].total_mm;
      r1h = mm_to_hin(d < 0 ? 0 : d);
  }
  rain_idx = (rain_idx + 1) % 4;

  int baro = 0;
  if (bme_available) {
    float p = bme.readPressure();
    if (!isnan(p) && p > 80000.0) baro = (int)(p / 10.0);
  }

  // PRZELICZANIE SŁOŃCA (Lux -> W/m2)
  int lum_wm2 = 0;
  if (!isnan(current_wx.light_klx)) {
      lum_wm2 = (int)(current_wx.light_klx * 7.9); // 1 klx ~= 7.9 W/m2
  }

  String ts = get_timestamp();
  String body = "@" + ts + format_lat(SITE_LAT) + "/" + format_lon(SITE_LON) + "_";
  
  int wd = (int)current_wx.wind_dir;
  body += p3(wd <= 0 ? 0 : wd) + "/" + p3(ms_to_mph(avg_w));
  body += "g" + p3(ms_to_mph(wind_gust_max_period));
  body += "t" + p3(c_to_f(current_wx.temp_c));
  if (r1h > 0) body += "r" + p3(r1h);
  if (current_wx.humidity > 0) { char hb[5]; snprintf(hb, sizeof(hb), "h%02d", (int)current_wx.humidity); body += hb; }
  if (baro > 0) { char bb[10]; snprintf(bb, sizeof(bb), "b%05d", baro); body += bb; }
  
  // Dodajemy SŁOŃCE do ramki (Lxxx)
  if (lum_wm2 > 0) {
      if (lum_wm2 > 999) lum_wm2 = 999;
      char lb[6]; snprintf(lb, sizeof(lb), "L%03d", lum_wm2);
      body += lb;
  }
  
  String comment = " RodosWX_2";
  comment += " Sig:" + String(current_wx.radio_rssi) + "dBm";
  if (!isnan(current_wx.uv_index)) comment += " UV:" + String(current_wx.uv_index, 1);
  comment += current_wx.battery_ok ? " Bat:OK" : " Bat:LOW";

  send_ax25_frame(body + comment);
  
  wind_speed_sum = 0; wind_sample_count = 0; wind_gust_max_period = 0;
}

void setup() {
  delay(3000); 
  Serial.begin(115200);
  Serial.println(F("\n\n--- START RodosWX_2 (KISS) ---"));

  Wire.begin(2, 5); 
  Wire.setClock(100000);
  
  if (bme.begin(0x76)) {
      Serial.println(F("BME280: OK"));
      bme_available = true;
  } else {
      Serial.println(F("BME280: NIE WYKRYTO"));
  }

  LittleFS.begin();
  ws.begin();

  Serial.println(F("Laczenie WiFi..."));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  int wifi_timeout = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_timeout < 40) { 
      delay(500); Serial.print("."); wifi_timeout++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
      Serial.println(F(" WiFi OK"));
      configTime(0, 0, "pool.ntp.org");
      while (time(nullptr) < 100000) { delay(500); Serial.print("T"); }
      Serial.println(F(" Time OK"));
  }
  
  for(int i=0; i<4; i++) rain_buffer[i].valid = false;
  Serial.println(F("--- SETUP ZAKONCZONY ---"));
}

void loop() {
  ws.clearSlots();
  if (ws.getMessage() == DECODE_OK && ws.sensor[0].valid) {
      Serial.println(F("[RADIO] Odebrano dane"));
      
      if (ws.sensor[0].w.temp_ok) current_wx.temp_c = ws.sensor[0].w.temp_c;
      if (ws.sensor[0].w.humidity_ok) current_wx.humidity = ws.sensor[0].w.humidity;
      if (ws.sensor[0].w.rain_ok) current_wx.rain_total_mm = ws.sensor[0].w.rain_mm;
      
      // ODCZYT UV I ŚWIATŁA (Przywrócony!)
      #if defined BRESSER_6_IN_1 || defined BRESSER_7_IN_1
        if (ws.sensor[0].w.uv_ok) current_wx.uv_index = ws.sensor[0].w.uv;
      #endif
      #ifdef BRESSER_7_IN_1
        if (ws.sensor[0].w.light_ok) current_wx.light_klx = ws.sensor[0].w.light_klx;
      #endif
      
      current_wx.radio_rssi = ws.sensor[0].rssi;
      current_wx.battery_ok = ws.sensor[0].battery_ok;

      if (ws.sensor[0].w.wind_ok) {
          current_wx.wind_dir = ws.sensor[0].w.wind_direction_deg;
          float s = ws.sensor[0].w.wind_avg_meter_sec;
          float g = ws.sensor[0].w.wind_gust_meter_sec;
          wind_speed_sum += s; wind_sample_count++;
          if (g > wind_gust_max_period) wind_gust_max_period = g;
          
          Serial.printf("Wind: %.1f m/s | RSSI: %d dBm | Lux: %.1f\n", 
                        s, current_wx.radio_rssi, current_wx.light_klx);
      }
      current_wx.valid_data = true;
  }

  if (millis() - last_report_time >= REPORT_INTERVAL_MS) {
      if (current_wx.valid_data && WiFi.status() == WL_CONNECTED) {
          send_aprs();
      }
      last_report_time = millis();
  }
  delay(50);
}
