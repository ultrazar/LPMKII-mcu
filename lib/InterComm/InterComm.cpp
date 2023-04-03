/*********
 * Megazar21 Software
 * Lancelot PACINI MKII open-source firmware 2023 ©
 * https://www.instagram.com/cansat_pacini/
 * 
 * Internal communications script (on both MCUs)
*********/
#include <Arduino.h>


#define ADAFRUIT
#ifdef ADAFRUIT
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
    Serial.print("#" + text);
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



/*
RX section
*/

void newLine(String line)  {
    //Serial.println(line);
    char initChar = line.charAt(0);
    switch (initChar) {
        case '#': // Debug print
            Serial.println("CAM: " + line.substring(1));
            break;
        case '!': // Error
            Serial.println("CAM-ERROR: " + line.substring(1));
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