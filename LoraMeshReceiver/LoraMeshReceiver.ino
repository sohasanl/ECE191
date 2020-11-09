#include <heltec.h>
#include <AESLib.h>
#include <CRC32.h>

#define BAND    915E6  //you can set band here directly,e.g. 868E6,915E6
AESLib aesLib;
CRC32 crc;

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

void setup() {
 
 //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

  // register the receive callback
  LoRa.onReceive(onReceive);

  // put the radio into receive mode
  LoRa.receive();
}

void loop() {
  // do nothing
}

void onReceive(int packetSize)
{
  // received a packet
  Serial.println("Received packet '");
  byte recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte cs[4]={0};
  
  cs[0] = LoRa.read();      // incoming msg checksum
  cs[1] = LoRa.read();      // incoming msg checksum
  cs[2] = LoRa.read();      // incoming msg checksum 
  cs[3] = LoRa.read();      // incoming msg checksum
  int checksum = (cs[0] << 24) | (cs[1] << 16) | (cs[2] << 8) | cs[3];
  byte dump =cs[3] = LoRa.read();    
  Serial.println(recipient, HEX);
  Serial.println(sender, HEX);
  Serial.println(incomingMsgId);
  Serial.println(checksum, DEC);
  
  // read packet
  String dataPacket;
  for (int i = 9; i <= packetSize; i++)
  {
    dataPacket += (char)LoRa.read();
  }
  
  Serial.println(dataPacket);
  Serial.println(dataPacket.length());
  Serial.println(CRC32::calculate(dataPacket.c_str(), dataPacket.length()));
  if(checksum == CRC32::calculate(dataPacket.c_str(), dataPacket.length())){
    
    Serial.println(F("TEST PASSED"));
    uint16_t dlen = dataPacket.length();
    sprintf(ciphertext, "%s", dataPacket.c_str());
    decrypted = decrypt( ciphertext, dlen, dec_iv);
    Serial.println(decrypted);
    
  } else{
    Serial.println(F("TEST FAILED"));
  }
  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}
