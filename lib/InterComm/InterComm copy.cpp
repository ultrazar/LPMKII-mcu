/*********
 * Megazar21 Software
 * Lancelot PACINI MKII open-source firmware 2023 ©
 * https://www.instagram.com/cansat_pacini/
 * 
 * Internal communications script (on both MCUs)
*********/
#include <Arduino.h>

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

const unsigned int PACKET_WAIT_TIME = 100; // milliseconds

enum packetType {
    PACKET_FILE,
    PACKET_CONFIG,
    PACKET_DIRECTOR,
    PACKET_STREAM
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
    Serial.write((const char*)&actual, sizeof(packetDirector));
    delay(PACKET_WAIT_TIME); // Give a time to process the packet...
}

struct packetFile { // An actual file or plain debug text (stream data to send to the base station, not fixed size)
    char nameLength = 5;
    byte name[64]; // Name of the file 
    unsigned short int dataSize; // MAX 65,535 bytes
};

struct packetConfig {
    bool enable_camera;
    unsigned short int interval_high = 30; // the high-quality photo takes priotity over low-quality
    unsigned short int interval_low = 7;
    unsigned int unix_time;
    imageSize camera_res_high;
    imageSize camera_res_low;
    int jpeg_quality_high; //0-63
    int jpeg_quality_low;
};


packetDirector lastPacketDirector;
packetConfig lastPacketConfig;

packetType lastPacket = PACKET_DIRECTOR;
packetType currentPacket = PACKET_DIRECTOR;
int currentPacketSize = sizeof(packetDirector);


void sendFile(byte* stream, String fileName, unsigned short int dataLength){
    packetFile message;
    message.dataSize = dataLength;

    fileName.getBytes(message.name,fileName.length());
    message.nameLength = fileName.length();

    sendPacketDirector(PACKET_FILE,sizeof(message)); // Send the packet director ("Going to send a file")

    Serial.write((const char*)&message, sizeof(message)); // Send the File info ("With 12 of lentgh and 'DEBUG' for name")

    delay(PACKET_WAIT_TIME);

    Serial.write((const char*)&stream, dataLength); // Send the actual stream of data ("Hello World!")

    delay(PACKET_WAIT_TIME);
}

void sendDebugText(String text){ // TX 
    byte data[text.length()];
    text.getBytes(data,text.length());
    sendFile(data, "DEBUG", (unsigned short int)text.length());
    
}
bool tempp = false;
bool serialIteration() {   // RX iteration
    if (Serial.available() && !tempp) {
        tempp = true;
        //Serial.println("Yeah, I received: " + String(Serial.available()));
    }
    if (Serial.available() == currentPacketSize) {
        lastPacket = currentPacket;
    if (currentPacket == PACKET_FILE){
        Serial.readBytes((char*)NULL, currentPacketSize);
        
        currentPacket = PACKET_DIRECTOR;
        currentPacketSize = sizeof(packetDirector);

    } else if (currentPacket == PACKET_CONFIG) {
        Serial.readBytes((char*) &lastPacketConfig, currentPacketSize);
        
        currentPacket = PACKET_DIRECTOR;
        currentPacketSize = sizeof(packetDirector);
        

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