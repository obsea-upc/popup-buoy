#pragma once
#include <Arduino.h>

// Open the FTP connection to the pop-up server (with LED feedback). Returns 0 ok, -1 error.
int connectToFTP();

// Download every file in this buoy's server folder (/PopUpBuoy_<idBuoy>) to the SD card,
// in batches, retrying/skipping per file. Returns 0 ok, -1 error.
int downloadAllFilesFTP();

// Helpers used by the download path.
int  getFileSize(const char* filename);
bool fileExistsInSD(const char* filename, uint32_t expectedSize);
int  tryDownloadFile(const char* source, int size, const char* dest, int tries);
int  DownloadFile(const char* source, int size, const char* dest);
