
#include "Adafruit_FONA.h"

// standard pins for the shield, adjust as necessary
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#define RF_TX 5
#define RF_RX 6
// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial RFSerial = SoftwareSerial(RF_TX, RF_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Have a FONA 3G? use this object type instead
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
 char latitude_buffer[20]; // Buffer big enough for 7-character float
 char longitude_buffer[20];
 char send_buffer[40];
 char checksum_buffer[10];
void setup() {
  
  while (! Serial);
  Serial.begin(115200);
  RFSerial.begin(38400);
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while(1);
  }
  Serial.println(F("FONA is OK"));
  // Try to enable GPRS
 
  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);
  // Setup blinky LED
  pinMode(13,OUTPUT);
  uint8_t led_state = true;
}

void loop() {
  delay(2000);
 // RFSerial.write("Start Sending ! \n");
  float latitude, longitude, speed_kph, heading, speed_mph, altitude;
  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  if (gps_success) {
    longitude = longitude -106.0;
    // Create sending package
    //*****************************
    //******************************
    dtostrf(latitude,2,6,latitude_buffer);
    dtostrf(longitude,1,6,longitude_buffer);
    uint8_t number_lat = 9;
    uint8_t number_long = 8;
    uint8_t k = 1, i=0;
    int16_t checksum = 0;
    send_buffer[0] = '*';
    for(i = 0; i < number_lat; i++){
        send_buffer[k] = latitude_buffer[i];
        k++;
      }
    send_buffer[k] = ',';
    k++;
    for(i = 0; i< number_long; i++){
        send_buffer[k] = longitude_buffer[i];
        k++;
      }
    // Create Checksum
    for (i =0; i<sizeof(send_buffer);i++){
        checksum += send_buffer[i];
      }
    send_buffer[k] = '#';
    for (i=0;i<=k; i++){
        RFSerial.write(send_buffer[i]);
      }
      k++;
      RFSerial.print(checksum);
      RFSerial.write('\n');
     // ******************
     //*****************************
     //*******************************
    digitalWrite(13, HIGH);
      
  } else {
    digitalWrite(13, LOW);
    Serial.println("none");  
  }
  
}

