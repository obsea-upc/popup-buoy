# State 6 / BlueBoat Upload — Changes

Branch: `feature/state6-transfer`

## What changed and why

### conf.h — test override

```c
#define TEST_FORCE_STATE_6
#define SECRET_FTP_UPLOAD_PORT 2121
```

`TEST_FORCE_STATE_6` forces `currentState = 6` immediately after EEPROM init in `setup()`, bypassing the need to press physical buttons. **Remove before real deployment.**

`SECRET_FTP_UPLOAD_PORT 2121` is a separate FTP port used only for uploading to the BlueBoat. Port 2121 is non-privileged (no root on the Jetson), different from the lander's vsftpd on port 21.

### setup() — state override

```cpp
#ifdef TEST_FORCE_STATE_6
  currentState = 6;
  EEPROM.write(0, 6);
  EEPROM.commit();
#endif
```

Inserted right after `initializeEEPROM()`. Config loading from `conf.txt` still runs (it runs for all states), so `idBuoy`, `maxFRM`, etc. are correctly loaded.

### tryUploadDataToUSV() — fully implemented

Previously returned `0` immediately after checking server identity. Now runs the full 6-step protocol:

| Step | Function | Returns if fail |
|------|----------|-----------------|
| 1 | `connectToRaspWiFi()` | `-1` |
| 2 | `getServerID()` → verify `"blueboat"` | `-2` |
| 3 | `getUploadPermission()` | `-3` |
| 4 | `sendManifestAndGetWantedFiles()` | `-4` |
| 5 | `uploadFilesViaFTP()` | continues (partial upload) |
| 6 | `confirmTransfer()` | `-5` |

### Helper functions added

#### `getUploadPermission()`
`GET /uploadpermission/<id>` — reads `navigation.yaml` on the server. Returns `false` if denied, so the buoy can retry next wake-up.

#### `sendManifestAndGetWantedFiles()`
Builds a JSON manifest of files on the SD card (`GPS_track.csv`, `LogFile.txt`, `dataFile.txt`), PUTs it to `/filelist/<id>`, and parses the server's wanted-file list. The server only asks for files it doesn't already have, so reconnects after dropped WiFi don't re-upload completed files.

#### `uploadFilesViaFTP()`
Creates a dedicated `ESP32_FTPClient` on `SECRET_FTP_UPLOAD_PORT` (2121). For each file:
- `NewFile(basename)` → FTP STOR
- `WriteData(buf, n)` in 512-byte chunks
- `CloseFile()`

Uses the existing `ESP32_FTPClient` upload primitives. The separate FTP client object keeps upload logic isolated from the lander download path.

#### `confirmTransfer(nSent)`
`POST /transfercomplete/<id>` with `{"files_sent": N}`. Server verifies files on disk. Returns `true` on success → `tryUploadDataToUSV()` returns `0` → state 6 transitions to state 4.

---

## Files transferred

| SD path | Basename sent to server |
|---------|------------------------|
| `/GPS_track.csv` | `GPS_track.csv` |
| `/LogFile.txt` | `LogFile.txt` |
| `/PopUpBuoy_<id>/dataFile.txt` | `dataFile.txt` |

---

## Testing

1. Ensure `#define TEST_FORCE_STATE_6` is active in `conf.h`
2. Flash the ESP32
3. Start popup-server on Jetson in blueboat mode (see popup-server/CHANGES.md)
4. Start the WiFi hotspot on the Jetson (see blueboat-core scripts)
5. Power the ESP32 — it boots into state 6, connects to the hotspot, uploads
6. Check `~/popup_data/<id>/` on the Jetson for received files

**Remove `TEST_FORCE_STATE_6` before real deployment.**
