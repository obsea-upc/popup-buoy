#pragma once
#include <Arduino.h>

// KIM (Kineis/ARGOS) module configuration and satellite transmission of GPS + seabed data.

// Configure the KIM module (power per state, message format).
// Returns false when the module could not be brought up after
// SAT_MODULE_MAX_ATTEMPTS tries with a power cycle between them; the caller then
// skips the session instead of transmitting into a module that is not there.
bool configureKIM();

// Build the Kineis message (hex lat/long/epoch + battery byte or CRC) into kineisMessage.
void maskGPS(double &gpsLat, double &gpsLong, uint32_t &epochTime, char *kineisMessage, char *ADCreadHex);

// Transmit kineisMessage sendRepeat times, sleeping waitRepeat ms between sends.
bool sendGPSviaKIM(int sendRepeat, int waitRepeat);

// Build + transmit the current GPS position for timeSending seconds.
void SendGPSMessage(int timeSending);

// Transmit one seabed-data line (kineisdataMessage).
bool SendDataMessage();   // true when the module accepted it

// --- Seabed-data file transmission + progress tracking (SD data/progress files) ---
void readSuccessFile();
void splitLineProgressFile(const String &line, int &row, int &nbrsent);
void splitLineDataFile(const String &line, int &index, char *data);
char *GetLineDataFile(int Row);
void SendFileKim(int time_to_send);
void SaveInProgressFile(int CurrentRow, int CurrentNbrSent);
void countLinesInDataFile();

// --- Transmit slots (conf.h, "Transmit slots") ---
// Sets the UTC time base from the GPS fix: at this instant UTC is unixSec plus
// msIntoSecond milliseconds. Called by gps.cpp as soon as the fix is in.
void txClockSyncGps(uint32_t unixSec, uint32_t msIntoSecond);
// Which slot of the 30 s cycle this buoy owns: TX_SLOT from conf.txt (0-5), or -1
// to derive it from idBuoy.
void txSetSlotConfig(int slot);
// The window every emission of this wake must fall in, in unix seconds: the planned
// session at MinElev. 0, 0 lifts it (no coverage: the recovery messages).
void txSetSessionWindow(uint32_t startUnix, uint32_t endUnix);

// Seabed-data and send-progress file paths (defined in satellite_tx.cpp).
extern char *SD_data_filename;
extern const char *SD_progress_filename;

// KIM transmission parameters (defined in satellite_tx.cpp).
extern char PWR2[10];
extern char PWR3[10];
extern char AFMT[8];
extern const int delayKIM;

// Data-transmission progress counters (defined in satellite_tx.cpp).
extern int RowProgress;
extern int MaxRowDataFile;
extern int MaxNbrMsgSendingDataFile;
