/*********
 * Megazar21 Software
 * Lancelot PACINI MKII open-source firmware 2023 ©
 * https://www.instagram.com/cansat_pacini/
 * 
 * Internal communications script (on both MCUs)
*********/
#include <Arduino.h>

typedef enum {
    FRAMESIZE_96X96,    // 96x96
    FRAMESIZE_QQVGA,    // 160x120
    FRAMESIZE_QCIF,     // 176x144
    FRAMESIZE_HQVGA,    // 240x176
    FRAMESIZE_240X240,  // 240x240
    FRAMESIZE_QVGA,     // 320x240
    FRAMESIZE_CIF,      // 400x296
    FRAMESIZE_HVGA,     // 480x320
    FRAMESIZE_VGA,      // 640x480
    FRAMESIZE_SVGA,     // 800x600
    FRAMESIZE_XGA,      // 1024x768
    FRAMESIZE_HD,       // 1280x720
    FRAMESIZE_SXGA,     // 1280x1024
    FRAMESIZE_UXGA,     // 1600x1200
} imageSize;



const unsigned int PACKET_WAIT_TIME = 100; // milliseconds

enum packetType {
    PACKET_FILE,
    PACKET_CONFIG,
    PACKET_DIRECTOR,
};

struct packetDirector {
    int ID = 0x1234abcd; // resync checkpoint, in case of packet loss or desync
    packetType dataType;
    unsigned short int dataSize;
};

void sendPacketDirector(packetType pt, int size) {
    packetDirector actual;
    actual.dataType = pt;
    actual.dataSize = size;
    Serial.write((const char*)&actual, sizeof(actual));
    delay(PACKET_WAIT_TIME); // Give a time to process the packet...
}

struct packetFile { // An actual file or plain debug text (stream data to send to the base station, not fixed size)
    char nameLength = 5;
    char name[64] = "DEBUG"; // Name of the file 
    unsigned short int dataSize; // MAX 65,535 bytes
};

struct packetConfig {
    bool enable_camera;
    unsigned short int interval_high = 30; // the high-quality photo takes priotity over low-quality
    unsigned short int interval_low = 7;
    unsigned int unix_time;
    framesize_t camera_res_high;
    framesize_t camera_res_low;
    int jpeg_quality_high; //0-63
    int jpeg_quality_low;
};


packetDirector lastPacketDirector;
packetConfig lastPacketConfig;

packetType lastPacket = PACKET_DIRECTOR;
packetType currentPacket = PACKET_DIRECTOR;
int currentPacketSize = sizeof(PACKET_DIRECTOR);


void sendFile(char* stream, String fileName, unsigned short int dataLength){
    packetFile message;
    message.dataSize = dataLength;

    strcpy(message.name,fileName.c_str());
    message.nameLength = fileName.length();

    sendPacketDirector(PACKET_FILE,sizeof(message)); // Send the packet director ("Going to send a file")

    Serial.write((const char*)&message, sizeof(message)); // Send the File info ("With 12 of lentgh and 'DEBUG' for name")

    delay(PACKET_WAIT_TIME);

    Serial.write((const char*)&stream, dataLength); // Send the actual stream of data ("Hello World!")

    delay(PACKET_WAIT_TIME);
}

void sendDebugText(String text){ // TX 
    char* t = "";
    strcpy(t,text.c_str());
    sendFile(t, "DEBUG", text.length());
    
}

bool serialIteration() {   // RX iteration

    if (Serial.available() == currentPacketSize) {
        lastPacket = currentPacket;
    if (currentPacket == PACKET_FILE){
        Serial.readBytes((char*)NULL, currentPacketSize);
        
        currentPacket = PACKET_DIRECTOR;
        currentPacketSize = sizeof(PACKET_DIRECTOR);

    } else if (currentPacket == PACKET_CONFIG) {
        Serial.readBytes((char*) &lastPacketConfig, currentPacketSize);
        
        currentPacket = PACKET_DIRECTOR;
        currentPacketSize = sizeof(PACKET_DIRECTOR);
        

    } else if (currentPacket == PACKET_DIRECTOR) {
        Serial.readBytes((char*) &lastPacketDirector,currentPacketSize);

        currentPacket = lastPacketDirector.dataType;
        currentPacketSize = lastPacketDirector.dataSize;
    }
    return true; // A packet has arrived!
    }
    return false; // Nop, still hasn't arrived any packet


}


/*
END InterCOMM
*/