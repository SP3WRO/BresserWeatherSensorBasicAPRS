///////////////////////////////////////////////////////////////////////////////////////////////////
// BresserWeatherSensorBasic.ino
//
// Example for BresserWeatherSensorReceiver - 
// Using getMessage() for non-blocking reception of a single data message.
//
// The data may be incomplete, because certain sensors need two messages to
// transmit a complete data set.
// Which sensor data is received in case of multiple sensors are in range
// depends on the timing of transmitter and receiver.  
//
// https://github.com/matthias-bs/BresserWeatherSensorReceiver
//
//
// created: 05/2022
//
//
// MIT License
//
// Copyright (c) 2022 Matthias Prinke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// History:
//
// 20220523 Created from https://github.com/matthias-bs/Bresser5in1-CC1101
// 20220524 Moved code to class WeatherSensor
// 20220810 Changed to modified WeatherSensor class; fixed Soil Moisture Sensor Handling
// 20220815 Changed to modified WeatherSensor class; added support of multiple sensors
// 20221227 Replaced DEBUG_PRINT/DEBUG_PRINTLN by Arduino logging functions
// 20230624 Added Bresser Lightning Sensor decoder
// 20230804 Added Bresser Water Leakage Sensor decoder
// 20231023 Modified detection of Lightning Sensor
// 20231025 Added Bresser Air Quality (Particulate Matter) Sensor decoder
// 20240209 Added Leakage, Air Quality (HCHO/VOC) and CO2 Sensors
// 20240213 Added PM1.0 to Air Quality (Particulate Matter) Sensor decoder
// 20240716 Fixed output of invalid battery state with 6-in-1 decoder
// 20250127 Added Globe Thermometer Temperature (8-in-1 Weather Sensor)
//
// ToDo: 
// - 
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "WeatherSensorCfg.h"
#include "WeatherSensor.h"
#include "InitBoard.h"

#include <ESP8266WiFi.h>
#include <time.h>
#include <math.h>

WeatherSensor ws;


// ======= APRS-IS configuration (add your own credentials/position) =======
static const char* APRS_HOST = "euro.aprs2.net";
static const uint16_t APRS_PORT = 14580;
static const char* APRS_CALL = "SP0ABC-10";     // <-- CHANGE: your callsign-SSID (e.g., -13)
static const char* APRS_PASS = "APRSPASSCODE";         // <-- CHANGE: APRS-IS passcode for your base callsign
static const char* APRS_SOFT = "RodosWX 1.0";
static const char* APRS_WIFI_SSID = "NAZWAWIFI";   // <-- CHANGE
static const char* APRS_WIFI_PASS = "HASLOWIFI";     // <-- CHANGE
static const double APRS_LAT_DEC = 52.1234;     // <-- CHANGE: latitude in decimal degrees
static const double APRS_LON_DEC = 16.1234;     // <-- CHANGE: longitude in decimal degrees
static const unsigned long APRS_PERIOD_MS = 15UL * 60UL * 1000UL; // send every 15 minutes

// Keep last WX values
struct APRS_WX {
  float temp_c = NAN;
  uint8_t humidity = 255;          // 0..100, 255 = unknown
  float wind_avg_ms = NAN;
  float wind_gust_ms = NAN;
  float wind_dir_deg = NAN;
  float rain_mm_total = NAN;       // cumulative rain counter, if available
} aprs_wx;

struct RainSample { float mm; unsigned long ms; };
static RainSample aprs_rain_ring[4];
static uint8_t aprs_rain_idx = 0;
static unsigned long aprs_last_send_ms = 0;

// Convert helpers
static String aprs_dd_to_lat(double dd){
  char buf[16]; char hemi = (dd>=0)?'N':'S'; dd=fabs(dd);
  int d=(int)dd; double m=(dd-d)*60.0; snprintf(buf,sizeof(buf),"%02d%05.2f%c",d,m,hemi);
  return String(buf);
}
static String aprs_dd_to_lon(double dd){
  char buf[16]; char hemi = (dd>=0)?'E':'W'; dd=fabs(dd);
  int d=(int)dd; double m=(dd-d)*60.0; snprintf(buf,sizeof(buf),"%03d%05.2f%c",d,m,hemi);
  return String(buf);
}
static inline int aprs_ms_to_mph(float ms){ return (int)lround(ms * 2.23694f); }
static inline int aprs_c_to_f(float c)   { return (int)lround(c * 9.0f/5.0f + 32.0f); }
static inline int aprs_mm_to_hin(float mm){ return (int)lround(mm * 0.0393701f * 100.0f); }
static int aprs_clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
static String aprs_pad2(int v){ char b[6]; snprintf(b,sizeof(b),"%02d", aprs_clampi(v,0,99)); return String(b); }
static String aprs_pad3(int v){ char b[6]; snprintf(b,sizeof(b),"%03d", aprs_clampi(v,0,999)); return String(b); }

static String aprs_ztimestamp(){
  time_t now = time(nullptr); struct tm* g = gmtime(&now);
  if (!g) return String(""); // if NTP not ready, omit timestamp (position still okay)
  char b[16]; snprintf(b,sizeof(b),"%02d%02d%02dz", g->tm_mday, g->tm_hour, g->tm_min);
  return String(b);
}

static void aprs_send_now(){
  // Compute 1h rain from cumulative counter (if present)
  int rain1h_hin = 0;
  {
    uint8_t oldest = (aprs_rain_idx + 1) & 3;
    float now_mm = aprs_wx.rain_mm_total, old_mm = aprs_rain_ring[oldest].mm;
    if (!isnan(now_mm) && !isnan(old_mm)) {
      float dmm = now_mm - old_mm; if (dmm < 0) dmm = 0;
      rain1h_hin = aprs_clampi(aprs_mm_to_hin(dmm), 0, 999);
    }
  }

  String lat = aprs_dd_to_lat(APRS_LAT_DEC);
  String lon = aprs_dd_to_lon(APRS_LON_DEC);
  String ts  = aprs_ztimestamp();

  // Weather with position, primary table '/', weather station symbol '_'
  String body = ts.length() ? ("@" + ts + lat + "/" + lon + "_")
                            : ("!"      + lat + "/" + lon + "_");

  int dir = isnan(aprs_wx.wind_dir_deg)?0:(int)lround(aprs_wx.wind_dir_deg);
  int spd = isnan(aprs_wx.wind_avg_ms)?0:aprs_ms_to_mph(aprs_wx.wind_avg_ms);
  int gst = isnan(aprs_wx.wind_gust_ms)?spd:aprs_ms_to_mph(aprs_wx.wind_gust_ms);

  body += aprs_pad3((dir+360)%360) + "/" + aprs_pad3(spd);
  body += "g" + aprs_pad3(gst);

  if (!isnan(aprs_wx.temp_c)) {
    int tf = aprs_c_to_f(aprs_wx.temp_c);
    char tb[6]; snprintf(tb,sizeof(tb),"t%03d", abs(tf));
    body += tb;
  }
  if (aprs_wx.humidity <= 100) {
    int h = (aprs_wx.humidity >= 100) ? 0 : (int)aprs_wx.humidity; // 100% -> h00
    body += "h" + aprs_pad2(h);
  }
  if (rain1h_hin > 0) {
    char rb[8]; snprintf(rb,sizeof(rb),"r%03d", rain1h_hin);
    body += rb;
  }

  String frame = String(APRS_CALL) + ">APRS,TCPIP*:" + body;

  WiFiClient cli;
  if (!cli.connect(APRS_HOST, APRS_PORT)) { Serial.println(F("[APRS] connect fail")); return; }
  cli.printf("user %s pass %s vers %s\r\n", APRS_CALL, APRS_PASS, APRS_SOFT);
  delay(80);
  cli.println(frame);
  cli.flush();
  delay(40);
  cli.stop();
  Serial.print(F("[APRS] SENT: ")); Serial.println(frame);
}

// Feed latest fields from ws.sensor[] into aprs_wx (call after ws.getMessage()).
static void aprs_feed_from_ws(){
  // Basic example uses a single slot (i=0)
  int const i = 0;
  // If we just decoded a weather message, copy available fields
  if (ws.sensor[i].s_type == SENSOR_TYPE_WEATHER1){
    if (ws.sensor[i].w.temp_ok)     aprs_wx.temp_c = ws.sensor[i].w.temp_c;
    if (ws.sensor[i].w.humidity_ok) aprs_wx.humidity = ws.sensor[i].w.humidity;
    if (ws.sensor[i].w.wind_ok){
      aprs_wx.wind_gust_ms = ws.sensor[i].w.wind_gust_meter_sec;
      aprs_wx.wind_avg_ms  = ws.sensor[i].w.wind_avg_meter_sec;
      aprs_wx.wind_dir_deg = ws.sensor[i].w.wind_direction_deg;
    }
    if (ws.sensor[i].w.rain_ok){
      aprs_wx.rain_mm_total = ws.sensor[i].w.rain_mm;
    }
  }
}

// Periodic runner; call in loop()
static void aprs_periodic(){
  // store a sample for 1h rain calc at the same rate we send
  if (millis() - aprs_last_send_ms >= APRS_PERIOD_MS){
    aprs_rain_ring[aprs_rain_idx] = { aprs_wx.rain_mm_total, millis() };
    aprs_rain_idx = (aprs_rain_idx + 1) & 3;
    // Only send if we have at least some data
    if (!isnan(aprs_wx.temp_c) || !isnan(aprs_wx.wind_avg_ms) || !isnan(aprs_wx.wind_gust_ms)){
      aprs_send_now();
    } else {
      Serial.println(F("[APRS] Skipping send (no data yet)"));
    }
    aprs_last_send_ms = millis();
  }
}

// Wi-Fi + NTP init; call once in setup()
static void aprs_wifi_time_init(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(APRS_WIFI_SSID, APRS_WIFI_PASS);
  Serial.print(F("[WiFi] connecting"));
  for (int i=0; i<40 && WiFi.status()!=WL_CONNECTED; ++i){ delay(250); Serial.print('.'); }
  Serial.println();
  if (WiFi.status()==WL_CONNECTED){
    Serial.print(F("[WiFi] IP: ")); Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("[WiFi] not connected (will retry later)"));
  }
  configTime(0,0,"pool.ntp.org","time.nist.gov"); // UTC
}

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);

      aprs_wifi_time_init(); // WiFi+NTP for APRS
Serial.printf("Starting execution...\n");
    initBoard();
    ws.begin();
}


void loop() 
{   
    // This example uses only a single slot in the sensor data array
    int const i=0;

    // Clear all sensor data
    ws.clearSlots();

    // Tries to receive radio message (non-blocking) and to decode it.
    // Timeout occurs after a small multiple of expected time-on-air.
    int decode_status = ws.getMessage();

    if (decode_status == DECODE_OK) {
        char batt_ok[] = "OK ";
        char batt_low[] = "Low";
        char batt_inv[] = "---";
        char * batt;

        if ((ws.sensor[i].s_type == SENSOR_TYPE_WEATHER1) && !ws.sensor[i].w.temp_ok) {
            // Special handling for 6-in-1 decoder
            batt = batt_inv;
        } else if (ws.sensor[i].battery_ok) {
            batt = batt_ok;
        } else {
            batt = batt_low;
        }
        Serial.printf("Id: [%8X] Typ: [%X] Ch: [%d] St: [%d] Bat: [%-3s] RSSI: [%6.1fdBm] ",
            static_cast<int> (ws.sensor[i].sensor_id),
            ws.sensor[i].s_type,
            ws.sensor[i].chan,
            ws.sensor[i].startup,
            batt,
            ws.sensor[i].rssi);
           
        if (ws.sensor[i].s_type == SENSOR_TYPE_LIGHTNING) {
            // Lightning Sensor
            Serial.printf("Lightning Counter: [%4d] ", ws.sensor[i].lgt.strike_count);
            if (ws.sensor[i].lgt.distance_km != 0) {
                Serial.printf("Distance: [%2dkm] ", ws.sensor[i].lgt.distance_km);
            } else {
                Serial.printf("Distance: [----] ");
            }
            Serial.printf("unknown1: [0x%03X] ", ws.sensor[i].lgt.unknown1);
            Serial.printf("unknown2: [0x%04X]\n", ws.sensor[i].lgt.unknown2);

        }
        else if (ws.sensor[i].s_type == SENSOR_TYPE_LEAKAGE) {
            // Water Leakage Sensor
            Serial.printf("Leakage: [%-5s]\n", (ws.sensor[i].leak.alarm) ? "ALARM" : "OK");
      
        }
        else if (ws.sensor[i].s_type == SENSOR_TYPE_AIR_PM) {
            // Air Quality (Particular Matter) Sensor
            if (ws.sensor[i].pm.pm_1_0_init) {
                Serial.printf("PM1.0: [init] ");
            } else {
                Serial.printf("PM1.0: [%uµg/m³] ", ws.sensor[i].pm.pm_1_0);
            }
            if (ws.sensor[i].pm.pm_2_5_init) {
                Serial.printf("PM2.5: [init] ");
            } else {
                Serial.printf("PM2.5: [%uµg/m³] ", ws.sensor[i].pm.pm_2_5);
            }
            if (ws.sensor[i].pm.pm_10_init) {
                Serial.printf("PM10: [init]\n");
            } else {
                Serial.printf("PM10: [%uµg/m³]\n", ws.sensor[i].pm.pm_10);
            }
            
        }
        else if (ws.sensor[i].s_type == SENSOR_TYPE_CO2) {
            // CO2 Sensor
            if (ws.sensor[i].co2.co2_init) {
                Serial.printf("CO2: [init]\n");
            } else {
                Serial.printf("CO2: [%uppm]\n", ws.sensor[i].co2.co2_ppm);
            }

        }
        else if (ws.sensor[i].s_type == SENSOR_TYPE_HCHO_VOC) {
            // HCHO / VOC Sensor
            if (ws.sensor[i].voc.hcho_init) {
                Serial.printf("HCHO: [init] ");
            } else {
                Serial.printf("HCHO: [%uppb] ", ws.sensor[i].voc.hcho_ppb);
            }
            if (ws.sensor[i].voc.voc_init) {
                Serial.printf("VOC: [init]\n");
            } else {
                Serial.printf("VOC: [%u]\n", ws.sensor[i].voc.voc_level);
            }

        }
        else if (ws.sensor[i].s_type == SENSOR_TYPE_SOIL) {
            Serial.printf("Temp: [%5.1fC] ", ws.sensor[i].soil.temp_c);
            Serial.printf("Moisture: [%2d%%]\n", ws.sensor[i].soil.moisture);

        } else {
            // Any other (weather-like) sensor is very similar
            if (ws.sensor[i].w.temp_ok) {
                Serial.printf("Temp: [%5.1fC] ", ws.sensor[i].w.temp_c);
            } else {
                Serial.printf("Temp: [---.-C] ");
            }
            if (ws.sensor[i].w.humidity_ok) {
                Serial.printf("Hum: [%3d%%] ", ws.sensor[i].w.humidity);
            }
            else {
                Serial.printf("Hum: [---%%] ");
            }
            if (ws.sensor[i].w.wind_ok) {
                Serial.printf("Wmax: [%4.1fm/s] Wavg: [%4.1fm/s] Wdir: [%5.1fdeg] ",
                        ws.sensor[i].w.wind_gust_meter_sec,
                        ws.sensor[i].w.wind_avg_meter_sec,
                        ws.sensor[i].w.wind_direction_deg);
            } else {
                Serial.printf("Wmax: [--.-m/s] Wavg: [--.-m/s] Wdir: [---.-deg] ");
            }
            if (ws.sensor[i].w.rain_ok) {
                Serial.printf("Rain: [%7.1fmm] ",  
                    ws.sensor[i].w.rain_mm);
            } else {
                Serial.printf("Rain: [-----.-mm] "); 
            }
        
            #if defined BRESSER_6_IN_1 || defined BRESSER_7_IN_1
            if (ws.sensor[i].w.uv_ok) {
                Serial.printf("UVidx: [%2.1f] ",
                    ws.sensor[i].w.uv);
            }
            else {
                Serial.printf("UVidx: [--.-] ");
            }
            #endif
            #ifdef BRESSER_7_IN_1
            if (ws.sensor[i].w.light_ok) {
                Serial.printf("Light: [%2.1fklx] ",
                    ws.sensor[i].w.light_klx);
            }
            else {
                Serial.printf("Light: [--.-klx] ");
            }
            if (ws.sensor[i].s_type == SENSOR_TYPE_WEATHER2) {
                if (ws.sensor[i].w.tglobe_ok) {
                    Serial.printf("T_globe: [%3.1fC] ",
                    ws.sensor[i].w.tglobe_c);
                }
                else {
                    Serial.printf("T_globe: [--.-C] ");
                }
            }
            #endif
            Serial.printf("\n");

      }
    
    } // if (decode_status == DECODE_OK)
      aprs_feed_from_ws();
  aprs_periodic();
  delay(100);
} // loop()
