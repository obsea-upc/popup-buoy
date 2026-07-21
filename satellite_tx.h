#pragma once
#include <Arduino.h>

// KIM (Kineis/ARGOS) module configuration and satellite transmission of GPS + seabed data.

// Configure the KIM module (power per state, message format).
void configureKIM();

// Build the Kineis message (hex lat/long/epoch + battery byte or CRC) into kineisMessage.
void maskGPS(double &gpsLat, double &gpsLong, uint32_t &epochTime, char *kineisMessage, char *ADCreadHex);

// Transmit kineisMessage sendRepeat times, sleeping waitRepeat ms between sends.
bool sendGPSviaKIM(int sendRepeat, int waitRepeat);

// Build + transmit the current GPS position for timeSending seconds.
void SendGPSMessage(int timeSending);

// Transmit one seabed-data line (kineisdataMessage).
void SendDataMessage();

// --- Seabed-data file transmission + progress tracking (SD data/progress files) ---
void readSuccessFile();
void splitLineProgressFile(const String &line, int &row, int &nbrsent);
void splitLineDataFile(const String &line, int &index, char *data);
char *GetLineDataFile(int Row);
void SendFileKim(int time_to_send);
void SaveInProgressFile(int CurrentRow, int CurrentNbrSent);
void countLinesInDataFile();
