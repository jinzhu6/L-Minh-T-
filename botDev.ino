
#include "Adafruit_FONA.h"
#include <Wire.h>
#include <DFRobot_QMC5883.h>
// standard pins for the shield, adjust as necessary
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
#define SUBSS_RX 5
#define SUBSS_TX 6                      
#define RP6_TX 7
#define RP6_RX 8

DFRobot_QMC5883 compass;
// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial SubSerial = SoftwareSerial(SUBSS_TX, SUBSS_RX);
SoftwareSerial RP6Serial = SoftwareSerial(RP6_TX, RP6_RX);
SoftwareSerial *fonaSerial = &fonaSS;
uint8_t begin_flag = true;
uint8_t success_begin = true;
int RP6_current_angle = 0;
int RP6_command_angle = 0;
boolean correct = false;
int Des_angle, checksum, RP6checksum;
uint8_t receiveBuffer[20];
uint8_t letter = 0;
uint8_t k;
double distance = 0;
uint8_t a = 0, b= 0, c = 0;
double lat_target, lon_target;
uint8_t lat_buffer[20], long_buffer[20], checksum_buffer[10];
char sample[50];
uint8_t i= 0;
boolean gps_success;
float heading_RP6,old_heading;
float declinationAngle;
 
// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

// Have a FONA 3G? use this object type instead
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);
float latitude, longitude, speed_kph, heading, speed_mph, altitude;
int getWaypointHeading(float targetLatitude, float targetLongitude) {

  // radians
  float heading = atan2(targetLatitude - latitude, targetLongitude + 106 - longitude);

  // convert to degrees 0 to 180/-180
  heading *= (180 / 3.14159);

  // convert to 0-360 compass degrees, North = 0
  heading = (450 - (int)heading) % 360;

  return heading;

}
void setup() {
  while (! Serial);
  RP6Serial.begin(38400);
  Serial.begin(38400);
  SubSerial.begin(38400);
  SubSerial.println("Waiting for Signal!");
  delay (2000);
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    SubSerial.println("None");
    while(1);
  }
  fona.enableGPS(true);
  RP6Serial.write("Hello BOT!");
  
  while (!compass.begin())
  {
    SubSerial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

    if(compass.isHMC()){
        SubSerial.println("Initialize HMC5883");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        SubSerial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }
  delay (1000);
  Serial.println("Start Sending Something");
  pinMode(13, OUTPUT);
}

void loop() {
 
  
  gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
  Vector norm = compass.readNormalize();
  // Calculate heading
  heading_RP6 = atan2(norm.YAxis, norm.XAxis);

  declinationAngle = (0 + (30.0 / 60.0)) / (180 / PI);
  heading_RP6 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading_RP6 < 0){
   heading_RP6 += 2 * PI;
  }

  if (heading_RP6 > 2 * PI){
     heading_RP6 -= 2 * PI;  
  }
  heading_RP6 = (heading_RP6)*57.295779;
  if (heading_RP6 < 180){
      heading_RP6 += 180;
    }else{
        heading_RP6 -=180;
      }
  // Receive a String send from RF
  letter = 0;
  if(Serial.available()>0){
    while (letter < 50){
      sample[letter] = Serial.read();
      if (sample[letter] == '\n') break;
      letter++;
    }
  }
   // Check for star heading string!
    //Process String!! 
    if (sample[0] == '*'){
       k = 1;
       a = 0;
       b = 0;
       c = 0;
       while(!(sample[k] == ',')){
          lat_buffer[a] = sample[k];
           a++;
            k++;
       }
       k++;
       lat_buffer[a++] = '\0';
       while(!(sample[k] == '#')){
         long_buffer[b] = sample[k];
          b++;
         k++;
        }
        k++;
       long_buffer[b++] = '\0';
       while(!(sample[k]== '\n')){
            checksum_buffer[c] = sample[k];
            c++;
            k++;
        }
        checksum_buffer[c++] = '\0';
        //******* Calculate Checksum***////
        i = 0;
        RP6checksum = 0;
        while (!(sample[i] == '#')){
            RP6checksum += sample[i];
            i++;
          }
          RP6checksum += '#';
        //******* Result ********///
       lat_target = atof(lat_buffer);
       lon_target = atof(long_buffer);
       checksum = atoi(checksum_buffer);
      // clear buffer
      i=0;
      while(!(sample[i]=='\0')){
          sample[i] = '\0';
          i++;
        }
        i=0;
      while(!(lat_buffer[i]=='\0')){
          lat_buffer[i] = '\0';
          i++;
        }
          i=0;
      while(!(long_buffer[i]=='\0')){
          long_buffer[i] = '\0';
          i++;
        }
        i=0;
      while(!(checksum_buffer[i]=='\0')){
          checksum_buffer[i] = '\0';
          i++;
        }
       if (checksum == RP6checksum){
               correct = true;
               SubSerial.println(sample);
       }else{
            correct = false;
          
        }
        
    }
   
    for (int i = 0; i< 30; i++){
        sample[i] = '\0';
      } 
   // Read Local GPS
     if (gps_success ) {
        
        /* SubSerial.print("lat: ");
         SubSerial.println(latitude, 6);
         SubSerial.print("long: ");
         SubSerial.println(longitude, 6);*/
         SubSerial.print("Heading fix: ");
         SubSerial.println(heading_RP6);  
         //Command for RP6
          if (correct){          
            Des_angle = getWaypointHeading(lat_target,lon_target);
            RP6_current_angle = heading_RP6;
            RP6_command_angle = Des_angle - RP6_current_angle;
            if(RP6_command_angle < 0) RP6_command_angle = RP6_command_angle + 360;
            SubSerial.println("Command!!!");
            SubSerial.print("Angle: ");
            SubSerial.println(Des_angle);
             for(i =0; i<7; i++){
                RP6Serial.print("RUN");
                RP6Serial.print(RP6_command_angle);
                RP6Serial.print("FWD");
                RP6Serial.println(2000);
               RP6Serial.flush();
                delay(1000);
              }
              RP6Serial.flush(); 
             correct = false;
             checksum = 200;
             RP6checksum = 100;
          }else{
              SubSerial.print("RP6 Checksum: ");
              SubSerial.print(RP6checksum);
              SubSerial.print(" Target Checksum: ");
              SubSerial.println(checksum);
              
              delay(2000);
            }
         digitalWrite(13, HIGH);
      } else {
         SubSerial.print("Heading: ");
         SubSerial.println(heading_RP6);
         digitalWrite(13, LOW);
         delay(2000);
  }
}


