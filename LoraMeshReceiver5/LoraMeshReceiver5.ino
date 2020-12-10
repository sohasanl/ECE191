
// install the following
// https://github.com/me-no-dev/ESPAsyncWebServer/archive/master.zip
// https://github.com/me-no-dev/AsyncTCP/archive/master.zip
// https://github.com/me-no-dev/arduino-esp32fs-plugin/releases/


#include <heltec.h>
#include <AESLib.h>
#include <CRC32.h>

// Import Wi-Fi library
#include <WiFi.h>
#include "ESPAsyncWebServer.h"

// Libraries to get time from NTP Server
//#include <WiFiUdp.h>

#include <SPIFFS.h>


#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6
#define Fbattery    3700  //The default battery is 3700mv when the battery is fully charged.

String dataPacket;
String dataP;


AESLib aesLib;
CRC32 crc;
// Create AsyncWebServer object on port 80
AsyncWebServer server(80);  

bool locked = true;
word currentMsgId = 0;
word incomingMsgId = 0 ;
int corruptedPackets = 0;
float longitude;
float latitude;
int rssi;
int snr;

String encrypted;
String decrypted;
char cleartext[256];
char ciphertext[512];


//const char* ssid     = "the rumi";  
//const char* password = "ruminight";

const char* ssid     = "NETGEAR53";
const char* password = "helpfulbreeze882";


// AES Encryption Key
uint8_t aes_key[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// General initialization vector (you must use your own IV's in production for full security!!!)
uint8_t aes_iv[N_BLOCK] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t enc_iv[N_BLOCK] = { 0 }; // iv_block gets written to, provide own fresh copy...
uint8_t dec_iv[N_BLOCK] = { 0 };

void aes_init() {
  aesLib.gen_iv(aes_iv);
}


//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
String decrypt(char * msg, uint16_t msgLen, byte iv[]) {
  char decrypted[msgLen];
  aesLib.decrypt64(msg, msgLen, decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}


//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
void connectWiFi(){
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
     Serial.print(".");
     wait(500);
  }
  // Print local IP address and start web server

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Heltec.display->clear();
  Heltec.display->drawString(0, 1, WiFi.localIP().toString().c_str());
  Heltec.display->display();
}

//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------

String processor(const String& var){
  if(var == "PACKETDATA"){
    return getHttpData().c_str();
  }
  else if (var == "PACKETID"){
    return String(incomingMsgId);
  }
  else if (var == "RSSI"){
    return String(LoRa.packetRssi());
  }
  else if (var == "SNR"){
    return String(LoRa.packetSnr());
  }
  return String();
}

//--------------------------------------------------------------------------
/* non-blocking wait function */
//--------------------------------------------------------------------------
void wait(unsigned long milliseconds) {
  unsigned long timeout = millis() + milliseconds;
  while (millis() < timeout) {
    yield();
  }
}

//--------------------------------------------------------------------------
/* The receive callback function*/
//--------------------------------------------------------------------------
void onReceive(int packetSize)
{

  // received a packet
  dataPacket = "";
  byte recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address

  byte MsgId[2] = {0};
  MsgId[0] = LoRa.read();     // incoming msg ID LSB
  MsgId[1] = LoRa.read();     // incoming msg ID MSB
  incomingMsgId = (MsgId[0] << 8) | MsgId[1];
  
  
  byte cs[4]={0};
  
  cs[0] = LoRa.read();      // incoming msg checksum
  cs[1] = LoRa.read();      // incoming msg checksum
  cs[2] = LoRa.read();      // incoming msg checksum 
  cs[3] = LoRa.read();      // incoming msg checksum
  int checksum = (cs[0] << 24) | (cs[1] << 16) | (cs[2] << 8) | cs[3];
  byte dump = LoRa.read();    
  // read packet
 
  for (int i = 10; i <= packetSize; i++) { dataPacket += (char)LoRa.read();}

 if(checksum == CRC32::calculate(dataPacket.c_str(), dataPacket.length())){
    uint16_t dlen = dataPacket.length();
    sprintf(ciphertext, "%s", dataPacket.c_str());
    decrypted = decrypt( ciphertext, dlen, dec_iv);
    if( corruptedPackets = incomingMsgId - currentMsgId ) {corruptedPackets--;}
    currentMsgId = incomingMsgId; 
    Serial.println(corruptedPackets);
    Serial.println(getHttpData());
} else { Serial.println("CRC Failed"); } 

}
//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------


String getHttpData(){
  return String(incomingMsgId)+ "," + String(corruptedPackets)+ "," + LoRa.packetRssi()+ "," + LoRa.packetSnr()+ "," + decrypted;
  
}
//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------

void httpServerBegin(){
    if(!SPIFFS.begin()){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/packetdata", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", getHttpData().c_str());
  });

  server.on("/rssi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(LoRa.packetRssi()).c_str());
  });
  server.on("/snr", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(LoRa.packetSnr()).c_str());
  });
  server.on("/packetid", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(incomingMsgId).c_str());
  });
  server.on("/winter", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/Night.jpg", "image/jpg");
  });
  // Start server
  server.begin();
}

//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------

void onScreen(){
  Heltec.display->clear();
  Heltec.display->drawString(0, 1, WiFi.localIP().toString().c_str());
  Heltec.display->drawString(0, 10, String(incomingMsgId));
  Heltec.display->drawString(0, 20, String(LoRa.packetRssi()));
  Heltec.display->drawString(0, 30, String(LoRa.packetSnr()));
  Heltec.display->drawString(0, 40, "Corrupted : " + String(corruptedPackets));
  Heltec.display->display();
  
}



//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
void setup() {
  
  Heltec.begin(true /*DisplayEnable Enable*/, true /*LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();

  
  LoRa.onReceive(onReceive); 
  LoRa.receive(); // put the radio into receive mode
  
  connectWiFi();
  httpServerBegin();
  wait(1000);
}
//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
void loop() {
  // do nothing
  wait(100);
  onScreen();
 
}
