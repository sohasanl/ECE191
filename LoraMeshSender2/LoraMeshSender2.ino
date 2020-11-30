/* Both Hardware and software serial works great, however String library has a bug -> stack overflow*/


#include <heltec.h>
#include <AESLib.h>
#include <CRC32.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6
#define Fbattery    3700  //The default battery is 3700mv when the battery is fully charged.

AESLib aesLib;
CRC32 crc;
TinyGPSPlus gps;

//HardwareSerial GPS(2);  // Uart2  RX -> 16  TX -> 17
SoftwareSerial GPS;

 //Battery
//float XS = 0.00225;      //The returned reading is multiplied by this XS to get the battery voltage.
//uint16_t MUL = 1000;
//uint16_t MMUL = 100;

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
byte msgCount = 0;            // count of outgoing messages


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
  LoRa.write(msgCount);                 // add message ID
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


void setup() {

  aes_init();
  aesLib.set_paddingmode(paddingMode::Array);

  //adcAttachPin(13);
  //analogSetClockDiv(255); // 1338mS
  
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  //GPS.begin(9600);
  GPS.begin(9600, SWSERIAL_8N1, 23, 19, false, 1024);
}

void loop() {

  smartDelay(2000);
  sendPacket(getGPSInfo()+ ",90");
  
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Msg Id #: ");
  Heltec.display->drawString(0, 10, String(msgCount));
  Heltec.display->drawString(0, 20, String(checksum));
  Heltec.display->display();

  //uint16_t c  =  analogRead(13)*XS*MUL;
  //Serial.print((String)c);Serial.println("(mV)");

  
 
}
