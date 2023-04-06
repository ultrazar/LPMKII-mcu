/*********
 * Megazar21 Software
 * Lancelot PACINI MKII open-source firmware 2023 ©
 * https://www.instagram.com/cansat_pacini/
 * 
 * Internal communications script (on both MCUs)
*********/
#include <Arduino.h>
#include <SoftwareSerial.h> 

#define EBYTE_AUX 11

#define ADAFRUIT
#ifdef ADAFRUIT

#define DEBUG_ENABLED // Send the prints to the base station

SoftwareSerial* ebyte;
Uart* serial = &Serial1;
Adafruit_USBD_CDC* serialDebug = &Serial;
#else
HardwareSerial* serial = &Serial;
HardwareSerial* serialDebug = &Serial;
#endif

// Constants
typedef enum {
    SIZE_96X96,    // 96x96
    SIZE_QQVGA,    // 160x120
    SIZE_QCIF,     // 176x144
    SIZE_HQVGA,    // 240x176
    SIZE_240X240,  // 240x240
    SIZE_QVGA,     // 320x240
    SIZE_CIF,      // 400x296
    SIZE_HVGA,     // 480x320
    SIZE_VGA,      // 640x480
    SIZE_SVGA,     // 800x600
    SIZE_XGA,      // 1024x768
    SIZE_HD,       // 1280x720
    SIZE_SXGA,     // 1280x1024
    SIZE_UXGA,     // 1600x1200
} imageSize;

void startInterComm()  {
    serial->setTimeout(10);

}

/*
TX section
*/
void sendLine(String text) {
    serial->println(text);
}

void printDebug(String text) {
    Serial.println("#" + text);

    #ifdef DEBUG_ENABLED
    ebyte->println("#" + text); // Send the print to the base station
    #endif
}


void sendCameraSettigns(    bool enable, 
                            unsigned short int interval_high, 
                            unsigned short int interval_low, 
                            unsigned int unix_time, 
                            imageSize camera_res_high,
                            imageSize camera_res_low,
                            int jpeg_quality_high,
                            int jpeg_quality_low) {

    String result = String(enable) + ";" + 
                    String(interval_high) + ";" + 
                    String(interval_low) + ";" + 
                    String(unix_time) + ";" +
                    String(camera_res_high) + ";" + 
                    String(camera_res_low) + ";" + 
                    String(jpeg_quality_high) + ";" +
                    String(jpeg_quality_low);
    sendLine("C" + result);
    
}

void sendListFilesRequest() {
    sendLine("L");
}


/*
RX section
*/

void waitTransceiver(){
    while (digitalRead(EBYTE_AUX) == LOW) {delay(1);};
}

byte fileRXBuffer[65536];

void newLine(String line)  {
    //Serial.println(line);
    switch (line.charAt(0)) {
        case '#': // Debug print
            printDebug("CAM: " + line.substring(1));
            break;
        case '!': // Error
            printDebug("CAM-ERROR: " + line.substring(1));
            break;
        case 'R': // Command response
            waitTransceiver();
            ebyte->println(line);
            waitTransceiver();
            break;
        case 'D': // FILE TRANSER STARTS
            // DW<FileSize>
            pinMode(EBYTE_AUX,INPUT);
            int fileSize = line.substring(2).toInt();

            int bytesLeft = fileSize;
            int position = 0;

            while (bytesLeft != 0) { // Wait until all the bytes are received
                while (serial->available() == 0) {delay(1);} // Wait to receive actual data
                while (serial->available() != 0) {
                    fileRXBuffer[position] = serial->read();
                    position++;
                    bytesLeft -= 1;
                }
            }
            // In here, all the data has been received to the mcu!
            delay(500);
            waitTransceiver(); // Send that we are going to send a file of x size
            ebyte->print(line + "\n"); // line is DW<FileSize>
            waitTransceiver();
            delay(500);
            
            int packetSize = 0; //Send 500 bytes and then wait (the max ebyte buffer is 512)
            position = 0;

            for (int i = 0;i < fileSize; i++){
                ebyte->write(fileRXBuffer[position]);
                position++;
                packetSize++;
                if (packetSize == 250) {
                    waitTransceiver();
                    packetSize = 0;
                    delay(100);
                }
            }
            waitTransceiver();
            delay(500); // Extra waiting
            // And just right here, the data has benn sent to the base station
            sendLine("#WAKE_UP"); // Wake up the camera that was waiting till the data is sent
            printDebug("File sent succesfully");

            break;

    }
}

String RXBuffer;

bool serialIteration() {
    unsigned int bytesAvailable = serial->available();
    //Serial.println("Available: " + String(bytesAvailable) );
    if (bytesAvailable != 0) {
        bool newLineReceived = false;
        String actual = serial->readString();
        for (unsigned int i = 0; i < actual.length(); i++) {
            char iChar = actual.charAt(i);
            //serialDebug->println((byte)iChar);
            if (iChar == '\n' || iChar == '\r') {
                newLineReceived = true;
                newLine(RXBuffer);
                RXBuffer = "";
            } else {
                RXBuffer += iChar;
            }
        }
        return newLineReceived;
    }
    return false;
}


/*
------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------PHASE STATIONED CODE-------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------
*/
/*
The stationed phase works in a different way. It actually does nothing, until a command is received from the base station.
This phase is controlled by the base station, so that the Godot programm of the base station will define the behavior of LPMKII.
During this phase, the transmission protocol between the base station and the device changes from structs to ASCII plain text.

The available commands that LPMKII can process in the stationed phase are:

  SETTER_COMMANDS:

  CAMERA      - SYNTAX: /C;<bool enable>;<interval_high>;<interval_low>;<unix_time>;<camera_res_high>;<camera_res_low>,<jpeg_quality_high>;<jpeg_quality_low> 
              - DESCRIPTION: Sets the camera module settings:

                bool enable, 
                unsigned short int interval_high, 
                unsigned short int interval_low, 
                unsigned int unix_time, 
                imageSize camera_res_high,
                imageSize camera_res_low,
                int jpeg_quality_high,
                int jpeg_quality_low

  GETTER_COMMANDS:

  LIST FILES  - SYNTAX: /L 
              - DESCRIPTION: Retrieve the available files in the microSD card (images and csv files)
              - RESPONSE: "R1:<File1Name>,<File1Size>;<File2Name>,<File2Size>;..."

  DOWNLOAD    - SYNTAX: /D;<FileName>
              - DESCRIPTION: Download a file from the microSD card of ESP32-CAM, the fileSize should be previously retrieved with /LS
              - RESPONSE: (Raw data of undetermined size)

  SENSORS     - SYNTAX: /S
              - DESCRIPTION: Ask the relevant sensors data 
              - RESPONSE: "R2:Temp;Humidity;Pressure;Battery;SolarPanels"
*/

/*
RX 
*/

void newBaseLine(String line) {
    switch (line.charAt(0)) {
        case '#': // Debug print
            printDebug("PONG: " + line.substring(1));
            break;
        case '/': // Error
            switch (line.charAt(1)) {
                case 'L':
                    sendListFilesRequest();
                    break;
                case 'D': // /D;<FileName>
                    printDebug(line.substring(3));
                    delay(100);
                    sendLine("D" + line.substring(3));
                    break;
            }
            break;
    }
}


String ebyteRXBuffer;

bool ebyteIteration() {
    unsigned int bytesAvailable = ebyte->available();
    if (bytesAvailable != 0) {
        bool newLineReceived = false;
        String actual = ebyte->readString();
        for (unsigned int i = 0; i < actual.length(); i++) {
            char iChar = actual.charAt(i);
            //serialDebug->println((byte)iChar);
            if (iChar == '\n' || iChar == '\r') {
                newLineReceived = true;
                newBaseLine(ebyteRXBuffer);
                ebyteRXBuffer = "";
            } else {
                ebyteRXBuffer += iChar;
            }
        }
        return newLineReceived;
    }
    return false;
}

