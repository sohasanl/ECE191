/* Both Hardware and software serial works great, however String library has a bug -> stack overflow*/

#include <Arduino.h>
#include <heltec.h>
#include <AESLib.h>
#include <CRC32.h>
#include <esp_adc_cal.h>
#include <driver/adc.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

AESLib aesLib;
CRC32 crc;
TinyGPSPlus gps;
SoftwareSerial GPS;

 //Battery
#define MAXBATT                 4200    // The default Lipo is 4200mv when the battery is fully charged.
#define LIGHT_SLEEP_VOLTAGE     3750    // Point where start light sleep
#define MINBATT                 3200    // The default Lipo is 3200mv when the battery is empty...this WILL be low on the 3.3v rail specs!!!

#define VOLTAGE_DIVIDER         3.20    // Lora has 220k/100k voltage divider so need to reverse that reduction via (220k+100k)/100k on vbat GPIO37 or ADC1_1 (early revs were GPIO13 or ADC2_4 but do NOT use with WiFi.begin())
#define DEFAULT_VREF            1100    // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE            500     // Battery sample rate in ms
#define VBATT_SMOOTH            50      // Number of averages in sample
#define ADC_READ_STABILIZE      5       // in ms (delay from GPIO control and ADC connections times)
#define LO_BATT_SLEEP_TIME      10*60*1000*1000     // How long when low batt to stay in sleep (us)
#define HELTEC_V2_1             1       // Set this to switch between GPIO13(V2.0) and GPIO37(V2.1) for VBatt ADC.
#define VBATT_GPIO              21      // Heltec GPIO to toggle VBatt read connection ... WARNING!!! This also connects VEXT to VCC=3.3v so be careful what is on header.  Also, take care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH
#define __DEBUG                 0       // DEBUG Serial output

esp_adc_cal_characteristics_t *adc_chars;

// CRC & AES
uint32_t checksum;
String byteBuffer = "test";
String encrypted;
char cleartext[256];
char ciphertext[512];

// AES Encryption Key
uint8_t aes_key[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };


// General initialization vector (you must use your own IV's in production for full security!!!)
uint8_t aes_iv[N_BLOCK] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t enc_iv[N_BLOCK] = { 0 }; // iv_block gets written to, provide own fresh copy...
uint8_t dec_iv[N_BLOCK] = { 0 };





// LORA
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;      // destination to send to

String outgoing;              // outgoing message
word msgCount = 0;            // count of outgoing messages


long lastTime = 0;        // last send time
int interval = 5000;          // interval between sends


long Longitude = 0;        // Longitude
long Latitude = 0;        // Latitude

// Generate IV (once)
void aes_init() {
  aesLib.gen_iv(aes_iv);
}

String encrypt(const char * msg, uint16_t msgLen, byte iv[]) {
  int cipherlength = aesLib.get_cipher64_length(msgLen);
  char encrypted[cipherlength]; // AHA! needs to be large, 2x is not enough
  aesLib.encrypt64(msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}

void sendPacket(String outgoing)
{
  sprintf(cleartext, "%s", outgoing.c_str()); // must not exceed 255 bytes; may contain a newline 
  uint16_t clen = outgoing.length();
  encrypted = encrypt(cleartext, clen, enc_iv);
  checksum = CRC32::calculate(encrypted.c_str(), encrypted.length());
  Serial.println(outgoing);
  Serial.println(encrypted);
  Serial.println(checksum);

 
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write((msgCount>>8) & 0xFF);     // add message ID
  LoRa.write(msgCount & 0xFF);          // add message ID 
  
  LoRa.write((checksum>>24) & 0xFF);           
  LoRa.write((checksum>>16) & 0xFF);
  LoRa.write((checksum>>8) & 0xFF); 
  LoRa.write(checksum & 0xFF);  
  LoRa.write(encrypted.length());        // add payload length
  LoRa.print(encrypted);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID 
}

String getGPSInfo()
{
  String gpsInfo = "";
  if (gps.location.isValid())
  {
    gpsInfo += String(gps.location.lat(),6);
    gpsInfo +=",";
    gpsInfo += String(gps.location.lng(),6);
    gpsInfo +=",";
  }
  else
  {
    gpsInfo +="INVALID";
  }
  
  if (gps.date.isValid())
  {
    gpsInfo += gps.date.month();
    gpsInfo += "/";
    gpsInfo += gps.date.day();
    gpsInfo += "/";
    gpsInfo += gps.date.year();
  }
  else
  {
    gpsInfo +="INVALID";
  }
  gpsInfo +=",";
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) gpsInfo += "0";
    gpsInfo += gps.time.hour();
    gpsInfo += ":";
    if (gps.time.minute() < 10) gpsInfo += "0";
    gpsInfo += gps.time.minute();
    gpsInfo += ":";
    if (gps.time.second() < 10) gpsInfo += "0";
    gpsInfo += gps.time.second();
  }
  else
  {
    gpsInfo +="INVALID";
  }

  return gpsInfo;
}




static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (GPS.available())
      gps.encode(GPS.read()); 
  } while (millis() - start < ms);

}


// Poll the proper ADC for VBatt on Heltec Lora 32 with GPIO21 toggled
uint16_t ReadVBatt() {
  uint16_t reading = 0;
  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize
  #if (defined(HELTEC_V2_1))
  pinMode(ADC1_CHANNEL_1, OPEN_DRAIN);        // ADC GPIO37
  reading = adc1_get_raw(ADC1_CHANNEL_1);
  pinMode(ADC1_CHANNEL_1, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider)
  #else
  pinMode(ADC2_CHANNEL_4, OPEN_DRAIN);        // ADC GPIO13
  adc2_get_raw(ADC2_CHANNEL_4,ADC_WIDTH_BIT_12,&reading);
  pinMode(ADC2_CHANNEL_4, INPUT);             // Disconnect ADC before GPIO goes back high so we protect ADC from direct connect to VBATT (i.e. no divider
  #endif

  uint16_t voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);  
  voltage*=VOLTAGE_DIVIDER;

  return voltage;
}

//  Use a buffer to average/sample ADC
uint16_t Sample() {
  static uint8_t i = 0;
  static uint16_t samp[VBATT_SMOOTH];
  static int32_t t = 0;
  static bool f = true;
  if(f){ for(uint8_t c=0;c<VBATT_SMOOTH;c++){ samp[c]=0; } f=false; }   // Initialize the sample array first time
  t -= samp[i];   // doing a rolling recording, so remove the old rolled around value out of total and get ready to put new one in.
  if (t<0) {t = 0;}

  // ADC read
  uint16_t voltage = ReadVBatt();

  samp[i]=voltage;
  #if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("ADC Raw Reading[%d]: %d", i, voltage);
  #endif
  t += samp[i];

  if(++i >= VBATT_SMOOTH) {i=0;}
  uint16_t s = round(((float)t / (float)VBATT_SMOOTH));
  #if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("   Smoothed of %d/%d = %d\n",t,VBATT_SMOOTH,s); 
  #endif

  return s;
}


String drawBattery(uint16_t voltage, bool sleep) {
  Heltec.display->setColor(BLACK);
  Heltec.display->fillRect(99,0,29,24);

  Heltec.display->setColor(WHITE);
  Heltec.display->drawRect(104,0,12,6);
  Heltec.display->fillRect(116,2,1,2);

  uint16_t v = voltage;
  if (v < MINBATT) {v = MINBATT;}
  if (v > MAXBATT) {v = MAXBATT;}
  double pct = map(v,MINBATT,MAXBATT,0,100);
  uint8_t bars = round(pct / 10.0);
  Heltec.display->fillRect(105,1,bars,4);
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
  // Draw small "z" when using sleep
  if (sleep > 0) {
    Heltec.display->drawHorizontalLine(121,0,4);
    Heltec.display->drawHorizontalLine(121,5,4);
    Heltec.display->setPixel(124,1);
    Heltec.display->setPixel(123,2);
    Heltec.display->setPixel(122,3);
    Heltec.display->setPixel(121,4);
  }
  Heltec.display->drawString(127,5,String((int)round(pct))+"%");
  Heltec.display->drawString(127,14,String(round(voltage/10.0)/100.0)+"V");

  Heltec.display->drawString(127, 25, "MsgId #: "+String(msgCount));
  //Heltec.display->drawString(127, 50, String(checksum));

  #if defined(__DEBUG) && __DEBUG > 0
  static uint8_t c = 0;
  if ((c++ % 10) == 0) {
    c = 1;
    Serial.printf("VBAT: %dmV [%4.1f%%] %d bars\n", voltage, pct, bars);
  }
  #endif

  return String((int)round(pct))+"%";
}


//=================================================================================================
//
//=================================================================================================

void setup() {

  aes_init();
  aesLib.set_paddingmode(paddingMode::Array);

//-------------------------------------------------------------------- 
  // Characterize ADC at particular atten
  #if (defined(HELTEC_V2_1))
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_DB_6);
  #else
  // Use this for older V2.0 with VBatt reading wired to GPIO13
  adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  adc2_config_channel_atten(ADC2_CHANNEL_4,ADC_ATTEN_DB_6);
  #endif

  #if defined(__DEBUG) && __DEBUG > 0
  Serial.printf("ADC Calibration: ");
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      Serial.printf("eFuse Vref\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      Serial.printf("Two Point\n");
  } else {
      Serial.printf("Default[%dmV]\n",DEFAULT_VREF);
  }
  #else
  if (val_type);    // Suppress warning
  #endif

  #if defined(__DEBUG) && __DEBUG >= 1
  Serial.printf("ADC Calibration: ");
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
      Serial.printf("eFuse Vref\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
      Serial.printf("Two Point\n");
  } else {
      Serial.printf("Default[%dmV]\n",DEFAULT_VREF);
  }
  #else
  if (val_type);    // Suppress warning
  #endif

  // Prime the Sample register
  for (uint8_t i = 0;i < VBATT_SMOOTH;i++) {
    Sample();
  }

  pinMode(VBATT_GPIO,OUTPUT);
  digitalWrite(VBATT_GPIO, LOW);              // ESP32 Lora v2.1 reads on GPIO37 when GPIO21 is low
  delay(ADC_READ_STABILIZE);                  // let GPIO stabilize

//--------------------------------------------------------------------- 
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();

//---------------------------------------------------------------------
  GPS.begin(9600, SWSERIAL_8N1, 23, 19, false, 1024);
}

//=================================================================================================
//   Main Loop
//=================================================================================================

void loop() {
  //smartDelay(2000);
  //sendPacket(getGPSInfo()+ ",90");
  Heltec.display->clear();
  uint16_t voltage = Sample();
  String battInfo = drawBattery(voltage, voltage < LIGHT_SLEEP_VOLTAGE);
  Heltec.display->display();

  smartDelay(2000);
  sendPacket(getGPSInfo()+ "," + battInfo);

  if (voltage < MINBATT) {                  // Low Voltage cut off shut down to protect battery as long as possible
    Heltec.display->setColor(WHITE);
    Heltec.display->setFont(ArialMT_Plain_10);
    Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
    Heltec.display->drawString(64,24,"Shutdown!!");
    Heltec.display->display();
    delay(2000);
    #if defined(__DEBUG) && __DEBUG > 0
    Serial.printf(" !! Shutting down...low battery volotage: %dmV.\n",voltage);
    delay(10);
    #endif
    esp_sleep_enable_timer_wakeup(LO_BATT_SLEEP_TIME);
    esp_deep_sleep_start();
  } else if (voltage < LIGHT_SLEEP_VOLTAGE) {     // Use light sleep once on battery
    uint64_t s = VBATT_SAMPLE;
    #if defined(__DEBUG) && __DEBUG > 0
    Serial.printf(" - Light Sleep (%dms)...battery volotage: %dmV.\n",(int)s,voltage);
    delay(20);
    #endif
    esp_sleep_enable_timer_wakeup(s*1000);     // Light Sleep does not flush buffer
    esp_light_sleep_start();
  }
  delay(ADC_READ_STABILIZE);

}
