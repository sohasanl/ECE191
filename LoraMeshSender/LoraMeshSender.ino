#include <heltec.h>
#include <AESLib.h>
#include <CRC32.h>

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6

AESLib aesLib;
CRC32 crc;
// CRC & AES
uint32_t checksum;
//uint8_t byteBuffer[] = "1024,34.0570362,-118.30973,Hello World";
String byteBuffer = "1024,34.0570362,-118.30973,Hello World";
String encrypted;
String decrypted;
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
long lastSendTime = 0;        // last send time
int interval = 5000;          // interval between sends



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

String decrypt(char * msg, uint16_t msgLen, byte iv[]) {
  char decrypted[msgLen];
  aesLib.decrypt64(msg, msgLen, decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

void sendPacket(String outgoing)
{
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  checksum = CRC32::calculate(outgoing.c_str(), outgoing.length());
  LoRa.write((checksum>>24) & 0xFF);           
  LoRa.write((checksum>>16) & 0xFF);
  LoRa.write((checksum>>8) & 0xFF); 
  LoRa.write(checksum & 0xFF);  
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID 
}

//
//void onReceive(int packetSize)
//{
//  if (packetSize == 0) return;          // if there's no packet, return
//
//  // read packet header bytes:
//  int recipient = LoRa.read();          // recipient address
//  byte sender = LoRa.read();            // sender address
//  byte incomingMsgId = LoRa.read();     // incoming msg ID
//  
//  uint32_t checksumLsB = LoRa.read();      // incoming msg checksum
//  uint32_t checksumMsB = LoRa.read();      // incoming msg checksum
//  uint32_t checksum = (checksumMsB * 256) + checksumLsB ;
//  Serial.println(checksum);
//  Heltec.display->drawString(0, 0, String(checksum));
//  
//  byte incomingLength = LoRa.read();    // incoming msg length
//  String incoming = "";                 // payload of packet
//
//  while (LoRa.available())             // can't use readString() in callback
//  {
//    incoming += (char)LoRa.read();      // add bytes one by one
//  }
//
//  if (incomingLength != incoming.length())   // check length for error
//  {
//    Serial.println("error: message length does not match length");
//    return;                             // skip rest of function
//  }
//
////  // if the recipient isn't this device or broadcast,
////  if (recipient != localAddress && recipient != 0xFF)
////  {
////    Serial.println("This message is not for me.");
////    return;                             // skip rest of function
////  }
////
////  // if message is for this device, or broadcast, print details:
////  Serial.println("Received from: 0x" + String(sender, HEX));
////  Serial.println("Sent to: 0x" + String(recipient, HEX));
////  Serial.println("Message ID: " + String(incomingMsgId));
////  Serial.println("Message length: " + String(incomingLength));
////  Serial.println("Message: " + incoming);
////  Serial.println("RSSI: " + String(LoRa.packetRssi()));
////  Serial.println("Snr: " + String(LoRa.packetSnr()));
////  Serial.println();
//}


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600); 
  aes_init();
  aesLib.set_paddingmode(paddingMode::Array);
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Init success!");
  //Heltec.display->drawString(0, 10, "Wait for incoming data...");
  Heltec.display->display();
  delay(1000);

  
  //LoRa.onReceive(onReceive);
  //LoRa.receive();
}

void loop() {

  sprintf(cleartext, "%s", byteBuffer.c_str()); // must not exceed 255 bytes; may contain a newline 
  uint16_t clen = byteBuffer.length();
  encrypted = encrypt(cleartext, clen, enc_iv);
  Serial.println(encrypted);
  uint16_t dlen = encrypted.length();
  sprintf(ciphertext, "%s", encrypted.c_str());
  decrypted = decrypt( ciphertext, dlen, dec_iv);
  // put your main code here, to run repeatedly:
  Serial.println(decrypted);
  checksum = CRC32::calculate(decrypted.c_str(), sizeof(decrypted) - 1);
  Serial.println(checksum);

  delay(500);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Msg Id #: ");
  Heltec.display->drawString(0, 10, String(msgCount));
  Heltec.display->drawString(0, 20, String(checksum));
  Heltec.display->drawString(0, 30, encrypted);
  //Heltec.display->drawString(0, 40, byteBuffer);
  Heltec.display->display();
  Serial.println(encrypted.length());
  sendPacket(encrypted);
  
  delay(2000);
}
