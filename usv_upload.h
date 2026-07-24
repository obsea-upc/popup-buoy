#pragma once
#include <Arduino.h>

// USV (BlueBoat) data-upload path — the surface counterpart to the seabed
// FTP *download* in ftp_download.*. When the buoy meets an unmanned surface
// vehicle, it offers the server a manifest of everything on the SD card, then
// FTP-STORs whatever the server says it still needs.
//
// Connects to the BlueBoat Wi-Fi AP, verifies the server identifies as
// "blueboat", asks permission, exchanges a file manifest and uploads the
// requested files over FTP (SECRET_FTP_UPLOAD_PORT), then confirms receipt.
//
// Returns 0 on a fully confirmed transfer, or a negative code on failure:
//   -1 no Wi-Fi   -2 server not blueboat   -3 permission denied
//   -4 manifest exchange failed   -5 server did not confirm   -6 incomplete upload
// Any negative return is safe to retry next cycle: the manifest step lets the
// server re-request only whatever is still missing.
int tryUploadDataToUSV();
