#include <Arduino.h>

/*
Oficial Andromeda PACINI MKII main firmware code

Megazar21 software
*/

// Library definition
#include <Adafruit_BMP280.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>    //  Library that provides NeoPixel functions
#include <SoftwareSerial.h>  // Serial connection for GPS
#include <TinyGPS.h>         // GPS parser
#include <Servo.h>

#define E32_TTL_1W
#include <LoRa_E32.h>

#include <InterComm.cpp>


// Cansat configuration
#define COMM_MODE 1 // 0 for text-based, 1 for advanced codification with special softw on base station
#define PACKET_SPEED 2 // Packets per second

// Constants and pinouts
#define GPS_RX 5
#define GPS_TX 6
#define EBYTE_RX 9
#define EBYTE_TX 10
#define EBYTE_M1 12
#define EBYTE_M0 13
#define EBYTE_AUX 11
#define BUZZER 14
#define CAMERA_SRV 15 // 1st servo
#define RUBBER_CUTTER_PIN 16 // 2nd servo
#define GPS_POWER 17 // Transistor to power on/off the gps module
#define LIPO_VOLTAGE 18 // Pin connected to the lipo battery through a double-100k ohm resistor
#define SOLAR_PANELS_VOLTAGE 19 // Pin connected to the solar panels output through a double-100k ohm resistor
#define LEGS_MOTOR 25 // Transistor to power on/off the legs motor

#define _SS_MAX_RX_BUFF 256 // Buffer per el serial del GPS 
// Potser es necesari canviar l'arxiu .h de la llibreria SoftwareSerial amb el paràmetre anterior


// Library object's construction
Adafruit_BMP280 bmp280;     // temperautre, barometric pressure
Adafruit_LIS3MDL lis3mdl;   // magnetometer
Adafruit_LSM6DS33 lsm6ds33; // accelerometer, gyroscope
Adafruit_SHT31 sht30;       // humidity
Adafruit_NeoPixel onePixel = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800); // (RGB pixel) -- Create a NeoPixel object called onePixel that addresses 1 pixel in pin 8
TinyGPS gps;                // GPS parser
SoftwareSerial ss_GPS(GPS_RX, GPS_TX);  // GPS SoftwareSerial
SoftwareSerial ss_ebyte(EBYTE_RX, EBYTE_TX); // Ebyte SoftwareSerial
LoRa_E32 e32ttl(&ss_ebyte,11,EBYTE_M0,EBYTE_M1,UART_BPS_RATE_9600); // Ebyte manager
Servo rubber_cutter_servo;


// Definició de variables globals
float temperature, pressure, altitude;
float magnetic_x, magnetic_y, magnetic_z;
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float humidity;
float latitud, longitud;
int sat;
int GPS_altitude;
int rubber_servo_angle; // 100º cuts the rubber (to cut: 80º-120º)
int rubber_servo_target_angle;
float lipo_voltage = 3.0;
float solar_panels_voltage; 



// Helper functions declaration:

void setLed(int8_t r, int8_t g, int8_t b) { // Debugging LED
  onePixel.setPixelColor(0, r, g, b); 
  onePixel.show();   

}

void ERROR(int8_t identifier) { // A breaker error
  setLed(255,0,0); // RED for error
  Serial.print("ERROR_ID: ");
  Serial.println(identifier);
  Serial.println("Bad error, inminent shutdown.");
  ss_ebyte.println("BAD_ERROR");
  while (1) {delay(100);} // Code breaker
}

void configure_ebyte() { // Set the ebyte to the previously defined configuration
  Serial.println("Ebyte configuration started!");
  // Set the ebyte to sleep/command mode
  ss_ebyte.begin(9600); // For sleep mode, 9600bps is needed
  //pinMode(EBYTE_M0_M1, OUTPUT);
  //digitalWrite(EBYTE_M0_M1, HIGH);
  e32ttl.setMode(MODE_3_SLEEP);
  
  ResponseStructContainer c;
    c = e32ttl.getConfiguration();
    if (c.status.code != E32_SUCCESS) { // In case of an error (Probably a communication error)
      Serial.println(getResponseDescriptionByParams( c.status.code) );
      ERROR(1); // ERROR_NUM_1
    }
    Configuration configuration = *(Configuration*) c.data;
    //printParameters(configuration);
    // DEFINE EBYTE PARAMETERS HERE:
    configuration.OPTION.transmissionPower = POWER_21; //  0.2W
    configuration.SPED.uartBaudRate = UART_BPS_19200;

    //configuration.OPTION.
    ResponseStatus r = e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE); // Save this configuration in ebyte's flash
  
  if (r.code != E32_SUCCESS) { // In case of an error
        Serial.println(getResponseDescriptionByParams( c.status.code) );
        ERROR(2); // ERROR_NUM_2
      }

  Serial.println("Ebyte configuration finished succesfully");

  // Return the EBYTE to the normal state
  ss_ebyte.begin(19200); // normal comm speed
  //digitalWrite(EBYTE_M0_M1,LOW);
  e32ttl.setMode(MODE_0_NORMAL);

  Serial.println("Ebyte set to normal mode");
}

void update_GPS() {
  bool newData = false;
  //ss_GPS.listen();
  
    while (ss_GPS.available())
    {
      char c = ss_GPS.read();
      if (gps.encode(c)) // ¿Ha entrado una nueva sentencia válida?
      newData = true;
    }
  if (newData)
  {    
//latitud y longitud, numero de satelites disponibles    
    gps.f_get_position(&latitud, &longitud);
    sat = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
    latitud = latitud == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : latitud;
    longitud = longitud == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : longitud;
    GPS_altitude = gps.altitude() / 100.0;
  }
}
bool tempConfigSent = false;

void runtimeTasks(uint32_t time) { // To avoid the use of delay()
  int actual = millis();
  while ((actual + time) > millis()) {
    if (rubber_servo_angle != rubber_servo_target_angle) {
      rubber_servo_angle += max(min((rubber_servo_target_angle - rubber_servo_angle),1),-1);
      rubber_cutter_servo.write(rubber_servo_angle);
    }

    delay(20);
  }
}

void update_sensor_data() {

  temperature = bmp280.readTemperature();
  pressure = bmp280.readPressure();
  altitude = bmp280.readAltitude(1013.25);

  lis3mdl.read();
  magnetic_x = lis3mdl.x;
  magnetic_y = lis3mdl.y;
  magnetic_z = lis3mdl.z;

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);
  accel_x = accel.acceleration.x;
  accel_y = accel.acceleration.y;
  accel_z = accel.acceleration.z;
  gyro_x = gyro.gyro.x;
  gyro_y = gyro.gyro.y;
  gyro_z = gyro.gyro.z;

  lipo_voltage = analogRead(LIPO_VOLTAGE);
  lipo_voltage *= 2;    // we divided by 2, so multiply back
  lipo_voltage *= 3.6;  // Multiply by 3.6V, our reference voltage
  lipo_voltage /= 1024; // convert to voltage

  humidity = sht30.readHumidity();

  //TODO: Update battery voltage info

  update_GPS();
  

}

// Main Arduino functions:

void setup() {
  // començar les comunicacions
  Serial.begin(115200); // USB Serial
  //ss_GPS.begin(9600); // GPS
  e32ttl.begin(); // EBYTE manager

  ebyte = &ss_ebyte;
  //ss_GPS.listen();

  pinMode(BUZZER, OUTPUT);
  pinMode(GPS_POWER, OUTPUT);
  pinMode(LIPO_VOLTAGE, INPUT);
  pinMode(SOLAR_PANELS_VOLTAGE, INPUT);
  pinMode(LEGS_MOTOR, OUTPUT);
  
  
  onePixel.begin();  // Start the NeoPixel object for the LED
  setLed(255,255,0); // YELLOW for startup
  

  delay(2000); // Esperar a que la comunicació serial per debugging estigui oberta
  
  Serial.println("                   __                              __         \r\n  ____ _____  ____/ /________  ____ ___  ___  ____/ /___ _    \r\n / __ `/ __ \\/ __  / ___/ __ \\/ __ `__ \\/ _ \\/ __  / __ `/    \r\n/ /_/ / / / / /_/ / /  / /_/ / / / / / /  __/ /_/ / /_/ /     \r\n\\__,_/_/ /_/\\__,_/_/   \\____/_/ /_/ /_/\\___/\\__,_/\\__,_/      \r\n    ____  ___   ___________   ______   __  _____ __ ________  \r\n   / __ \\/   | / ____/  _/ | / /  _/  /  |/  / //_//  _/  _/  \r\n  / /_/ / /| |/ /    / //  |/ // /   / /|_/ / ,<   / / / /    \r\n / ____/ ___ / /____/ // /|  // /   / /  / / /| |_/ /_/ /     \r\n/_/   /_/  |_\\____/___/_/ |_/___/  /_/  /_/_/ |_/___/___/");
  
  //ss_ebyte.println("Andromeda PACINI MKII is alive!");

  /*for (int i = 0; i < 400; i++) {
    tone(BUZZER, HIGH);
    delay(1000/400);
    digitalWrite(BUZZER, LOW);
    delay(1000/400);
  }*/
  tone(BUZZER,4000,500);
  digitalWrite(GPS_POWER,HIGH); // encendre el GPS
  
  // Ebyte configurator
  configure_ebyte();


  // initialize the integrated I2C sensors (mic and proximity/light sensors are innecessary)
  bmp280.begin();       // Temperature and pressure
  lis3mdl.begin_I2C();  // Magnetometer
  lsm6ds33.begin_I2C(); // Accelerometer and gyroscope
  sht30.begin();        // Humidity
  
  rubber_cutter_servo.attach(RUBBER_CUTTER_PIN);
  rubber_servo_angle = 80;
  rubber_servo_target_angle = 80;
  rubber_cutter_servo.write(rubber_servo_angle);

  Serial.println("Setup finished, running loop...");
  setLed(0,0,255);
  Serial1.begin(115200);
  startInterComm();
    
}

typedef enum {
  PHASE_AIR,
  PHASE_STATIONED
} phaseNum;

phaseNum actualPhase = PHASE_STATIONED;

/*
------------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------PHASE AIR CODE------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------
*/

struct tx_packet { // The sructure of the Lancelot TX data packets
  unsigned int protocol = 0x78563412;
  unsigned int time;
  unsigned int temperature; 
  unsigned int pressure; 
  unsigned int humidity; 
  unsigned int latitude; 
  unsigned int longitude; 
  unsigned int GPS_altitude;
  unsigned int sattelites;
  unsigned int lipo_voltage; 
 };
 tx_packet state;

tx_packet create_packet() {
  tx_packet result;
  result.time = millis();
  result.temperature = temperature * 100 + 10000;
  result.pressure = pressure * 10;
  result.humidity = humidity * 100;
  result.latitude = latitud*(100000) + (1000000000);
  result.longitude = longitud*(100000) + (1000000000);
  result.GPS_altitude = GPS_altitude;
  result.sattelites = sat;
  result.lipo_voltage = lipo_voltage * 100;
  return result;
}

void phaseAirLoop() {
  setLed(0,255,0);
  update_sensor_data();
  ss_ebyte.listen();

  if (COMM_MODE == 0) {
    String packet = String("\r\nT=") + temperature;
    packet += String("\r\nP=") + pressure;
    packet += String("\r\nAt=") + altitude;
    packet += String("\r\nH=") + humidity;
    packet += String("\r\nG=") + String(latitud,5); packet += String(", ") + String(longitud,5);
    packet += String("\r\nGAt=") + GPS_altitude;
    packet += String("\r\nBAT=") + lipo_voltage;
    packet += String("\r\n");
    ss_ebyte.print(packet);
    Serial.print(packet);
    while (ss_ebyte.available()) {
      ss_ebyte.read();
      rubber_servo_target_angle = 120;
    }
    if (rubber_servo_angle == 120) {
      rubber_servo_target_angle = 80;
    }
    

  } // TODO: advanced comm mode
  else {

  byte *ptr;
  tx_packet actual_packet = create_packet();
  ptr = (byte*)&actual_packet;
  byte counter = 40;//sizeof(actual_packet);
  do
  {
    byte m = (byte)*ptr;
    ss_ebyte.write(m);
    //Serial.print(m, HEX);
    String actual = String(m,HEX);
    if (actual.length() == 1) {
      actual = "0" + actual;
    }
    //Serial.print(actual);
    ptr++;
    counter--;
  }
  while(counter != 0);

    if (ss_ebyte.available()) { // Receive "PHASE_STATIONED" to change the device operation mode
      String message = ss_ebyte.readString();
      if (message == "PHASE_STATIONED") {
        actualPhase = PHASE_STATIONED;
        ss_ebyte.println("#STATIONED MODE ACTIVATED");
      }
    }
    
  }
  
  setLed(0,0,255);
  digitalWrite(BUZZER,LOW);
  digitalWrite(LEGS_MOTOR, HIGH);
  runtimeTasks(1000 / PACKET_SPEED);
  digitalWrite(BUZZER,HIGH);
  digitalWrite(LEGS_MOTOR, LOW);
}

/*
------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------PHASE STATIONED CODE-------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------
*/





void phaseStationedLoop() {
  serialIteration();
  ebyteIteration();
  if (tempConfigSent == false && millis() > 4000) {
    tempConfigSent = true;

    printDebug("Sending config to camera");
    sendLine("#PING CAMERA");
    delay(200);

    sendCameraSettigns(true,30,10,100,SIZE_HD,SIZE_VGA,10,30);

  }
  delay(20);
}

void loop() {
  switch (actualPhase) {
    case PHASE_AIR:
      phaseAirLoop();
      break;
    case PHASE_STATIONED:
      phaseStationedLoop();
      break;
  }
}
