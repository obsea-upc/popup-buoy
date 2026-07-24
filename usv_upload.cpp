#include "usv_upload.h"
#include "conf.h"
#include "secrets.h"
#include "logging.h"
#include "wifi_http.h"
#include <WiFi.h>   // ESP32_FTPClient.h uses WiFiClient but doesn't include it
#include "ESP32_FTPClient.h"
#include <HTTPClient.h>
#include <SD.h>
#include <vector>
#include <algorithm>

// Globals owned by the main sketch (popup-buoy.ino).
extern int idBuoy;
extern int releaseFlag;
extern ReleaseMode releaseMode;
extern int sleeptime_h;
extern int sleeptime_m;

// Base URL for the pop-up server's HTTP endpoints (whoami/uploadpermission/filelist/...).
static String serverUrl(const String& path) {
  return String("http://") + SECRET_FTP_SERVER_IP + ":" + String(SECRET_FTP_SERVER_PORT) + path;
}

// ---- Server identity -----------------------------------------------------

// GET /whoami — returns the server's "id" field (e.g. "blueboat"), empty on error.
// Lets the buoy tell a BlueBoat USV apart from the lander before it starts uploading.
static String getServerID() {
  HTTPClient http;
  http.begin(serverUrl("/whoami"));
  int code = http.GET();
  if (code > 300) {
    http.end();
    return String("");
  }
  String json = http.getString();
  http.end();

  int keyPos = json.indexOf("\"id\"");
  if (keyPos == -1) {
    writeLogFile("Key 'id' not found in the string.");
    return String("");
  }
  int colonPos = json.indexOf(":", keyPos);
  if (colonPos == -1) return String("");
  int startQuote = json.indexOf("\"", colonPos);
  if (startQuote == -1) return String("");
  int endQuote = json.indexOf("\"", startQuote + 1);
  if (endQuote == -1) return String("");
  return json.substring(startQuote + 1, endQuote);
}

// ---- USV upload helpers --------------------------------------------------

// GET /uploadpermission/<id> — reads navigation.yaml on the server. Returns
// false when denied, so the buoy just retries on the next wake-up.
static bool getUploadPermission() {
  HTTPClient http;
  http.begin(serverUrl(String("/uploadpermission/") + String(idBuoy)));
  int code = http.GET();
  if (code != 200) {
    writeLogFile("uploadpermission HTTP error: " + String(code));
    http.end();
    return false;
  }
  String resp = http.getString();
  http.end();
  // Parse {"allow": true} or {"allow": false}
  return (resp.indexOf("true") >= 0);
}

// Minimal in-memory tree used only to pretty-print the FTP upload manifest.
struct TreeNode {
  String name;
  bool isFile;
  std::vector<TreeNode> children;
};

// Builds JSON manifest of SD files and sends PUT /filelist/<id>.
// Fills wantedOut with SD-relative paths the server wants; returns count or -1 on error.
// sdPathsOut is filled in parallel with the full SD path for each wanted file.
// No fixed cap on file count — candidates/wanted lists grow on the heap, bounded
// only by whatever's actually free at upload time (this board has no PSRAM).
static int sendManifestAndGetWantedFiles(std::vector<String>& wantedOut, std::vector<String>& sdPathsOut) {
  // Candidate SD files to offer: every regular file anywhere on the card,
  // found by walking the directory tree (not just SD root — images and the
  // lander-instrument data file live under subfolders like /PopUpBuoy_<id>/).
  std::vector<String> candidates;

  // Iterative walk (explicit stack, not recursion — ESP32 loopTask stack is
  // small and this file already avoids recursion for the same reason).
  // Directory *depth* is kept bounded (unlike file count) — real folder
  // nesting doesn't grow with how many files/images you have.
  const int MAX_DIR_DEPTH = 16;
  static String dirStack[MAX_DIR_DEPTH];
  int stackTop = 0;
  dirStack[stackTop++] = "/";

  while (stackTop > 0) {
    String dirPath = dirStack[--stackTop];
    String dirPrefix = dirPath.endsWith("/") ? dirPath : dirPath + "/";
    File dir = SD.open(dirPath.c_str());
    if (!dir) continue;
    File entry = dir.openNextFile();
    while (entry) {
      // entry.name() is only reliably a bare basename for nested entries —
      // at root it happens to double as a root-relative path too, which is
      // how this silently passed testing there. Always rebuild the path
      // from the parent we're actually iterating, don't trust entry.name()
      // to already be reopenable on its own.
      String rawName = String(entry.name());
      int lastSlash = rawName.lastIndexOf('/');
      String baseName = (lastSlash >= 0) ? rawName.substring(lastSlash + 1) : rawName;
      String entryPath = dirPrefix + baseName;
      if (entry.isDirectory()) {
        if (stackTop < MAX_DIR_DEPTH) {
          dirStack[stackTop++] = entryPath;
        } else {
          writeLogFile("SD scan: max directory depth reached, skipping " + entryPath);
        }
      } else {
        candidates.push_back(entryPath);
      }
      entry.close();
      entry = dir.openNextFile();
    }
    if (entry) entry.close();
    dir.close();
  }
  writeLogFile("SD scan: found " + String(candidates.size()) + " file(s)");

  // Build JSON body manually (no ArduinoJson dependency).
  String body = "{\"files\":[";
  bool first = true;
  for (size_t i = 0; i < candidates.size(); i++) {
    if (candidates[i].length() == 0) continue;
    File f = SD.open(candidates[i].c_str(), FILE_READ);
    if (!f) continue;
    size_t sz = f.size();
    f.close();
    // Offer the SD-relative PATH (leading '/' stripped), not the basename:
    // the server mirrors the tree and dedupes by path, so files with the
    // same name in different folders no longer collide/overwrite.
    String rel = candidates[i].startsWith("/") ? candidates[i].substring(1) : candidates[i];
    if (!first) body += ",";
    body += "{\"name\":\"" + rel + "\",\"size\":" + String(sz) + "}";
    first = false;
  }
  body += "]}";

  HTTPClient http;
  http.begin(serverUrl(String("/filelist/") + String(idBuoy)));
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(body);
  if (code != 200) {
    writeLogFile("filelist PUT HTTP error: " + String(code));
    http.end();
    return -1;
  }
  String resp = http.getString();
  http.end();

  // Parse {"tobesent": ["GPS_track.csv", "PopUpBuoy_1/img.png", ...]}
  // Entries are SD-relative paths (they echo back what the manifest offered).
  int arrayStart = resp.indexOf('[');
  int arrayEnd   = resp.indexOf(']');
  if (arrayStart < 0 || arrayEnd < 0) return 0;

  String arr = resp.substring(arrayStart + 1, arrayEnd);
  int pos = 0;
  while (pos < (int)arr.length()) {
    int q1 = arr.indexOf('"', pos);
    if (q1 < 0) break;
    int q2 = arr.indexOf('"', q1 + 1);
    if (q2 < 0) break;
    String rel = arr.substring(q1 + 1, q2);
    // The relative path prefixed with '/' IS the SD path — no basename
    // search needed (and no ambiguity when names repeat across folders).
    wantedOut.push_back(rel);
    sdPathsOut.push_back("/" + rel);
    pos = q2 + 1;
  }
  return (int)wantedOut.size();
}

// Depth here is bounded by SD folder nesting (shallow in practice), so plain
// recursion is fine, unlike the SD-card walk in sendManifestAndGetWantedFiles()
// which needed an explicit stack.
static void treeInsert(TreeNode& root, const String& path) {
  TreeNode* node = &root;
  int start = 0;
  while (start <= (int)path.length()) {
    int slash = path.indexOf('/', start);
    bool isLeaf = (slash < 0);
    String part = isLeaf ? path.substring(start) : path.substring(start, slash);
    TreeNode* child = nullptr;
    for (auto& c : node->children) {
      if (c.name == part && c.isFile == isLeaf) { child = &c; break; }
    }
    if (!child) {
      node->children.push_back(TreeNode{part, isLeaf, {}});
      child = &node->children.back();
    }
    node = child;
    if (isLeaf) break;
    start = slash + 1;
  }
}

static void treePrint(TreeNode& node, const String& prefix) {
  for (size_t i = 0; i < node.children.size(); i++) {
    bool last = (i == node.children.size() - 1);
    TreeNode& c = node.children[i];
    writeLogFile(prefix + (last ? "`-- " : "|-- ") + c.name + (c.isFile ? "" : "/"));
    if (!c.isFile) {
      treePrint(c, prefix + (last ? "    " : "|   "));
    }
  }
}

// Logs `basenames` as a directory tree instead of one line per file, so a
// manifest of dozens of files takes a handful of log lines instead of one
// per entry.
static void logUploadManifestTree(std::vector<String>& basenames) {
  writeLogFile("FTP upload: " + String(basenames.size()) + " file(s) to transfer:");
  std::vector<String> sorted(basenames);
  std::sort(sorted.begin(), sorted.end());
  TreeNode root{"", false, {}};
  for (auto& b : sorted) treeInsert(root, b);
  treePrint(root, "");
}

// FTP-uploads each file from SD to the BlueBoat server.
// `basenames` holds SD-relative paths (e.g. "PopUpBuoy_1/img.png"); STOR with
// a relative path lands in the matching subfolder under /<idBuoy> — the server
// pre-creates those directories at /filelist time, so no MKD is needed here.
// Returns number of files successfully uploaded.
static int uploadFilesViaFTP(std::vector<String>& sdPaths, std::vector<String>& basenames) {
  size_t nFiles = sdPaths.size();
  if (nFiles == 0) return 0;
  logUploadManifestTree(basenames);
  char uploadDir[16];
  snprintf(uploadDir, sizeof(uploadDir), "/%d", idBuoy);

  // Heap-allocate the FTP client: its internal buffers total ~5.7 KB (inBuffer[4096]
  // + clientBuf[1500] + outBuf[128]), which would overflow the loopTask stack.
  ESP32_FTPClient* ftpUp = new ESP32_FTPClient(
    (char*)SECRET_FTP_SERVER_IP,
    SECRET_FTP_UPLOAD_PORT,
    (char*)SECRET_FTP_SERVER_USER,
    (char*)SECRET_FTP_SERVER_PASS
  );

  if (ftpUp->OpenConnection() < 0) {
    writeLogFile("FTP upload: connection failed");
    delete ftpUp;
    return 0;
  }

  // The /filelist endpoint already calls os.makedirs on the server side, so the
  // directory always exists by the time we get here. Calling MakeDir would return
  // 550 File exists, which sets _isConnected=false in the library and breaks all
  // subsequent calls. Skip MakeDir and go straight to CWD.
  ftpUp->ChangeWorkDir(uploadDir);

  int uploaded = 0;
  int lastLoggedDecile = 0;
  static uint8_t buf[512];

  for (size_t i = 0; i < nFiles; i++) {
    // WriteData/NewFile/CloseFile return void — isConnected() is the only
    // signal this library gives for a dropped session. Without this check,
    // a connection lost partway through a long batch would silently count
    // every remaining file as "uploaded" without sending any of them.
    if (!ftpUp->isConnected()) {
      writeLogFile("FTP upload: connection lost after " + String(uploaded) + "/" +
                    String(nFiles) + " files — stopping, remainder will retry next cycle");
      break;
    }
    if (sdPaths[i].length() == 0) {
      writeLogFile("No SD path for " + basenames[i] + " — skipping");
      continue;
    }
    File f = SD.open(sdPaths[i].c_str(), FILE_READ);
    if (!f) {
      writeLogFile("Cannot open " + sdPaths[i] + " — skipping");
      continue;
    }
    ftpUp->InitFile("Type I");  // opens fresh PASV data connection for this STOR
    ftpUp->NewFile(basenames[i].c_str());
    bool readError = false;
    while (f.available()) {
      int n = f.read(buf, sizeof(buf));
      if (n <= 0) {
        // SD read error: the position doesn't advance, so available() stays
        // true and looping again would spin forever — loopTask has no
        // watchdog, the buoy would hang until the battery died.
        readError = true;
        break;
      }
      ftpUp->WriteData(buf, n);
    }
    ftpUp->CloseFile();
    f.close();
    if (readError) {
      writeLogFile("SD read error on " + basenames[i] + " — truncated, not counted");
    } else if (!ftpUp->isConnected()) {
      // STOR rejected, PASV setup failed, or connection dropped mid-file:
      // the library only surfaces any of these via isConnected(), so check
      // it AFTER CloseFile before counting. The loop head logs and stops
      // on the next iteration.
      writeLogFile("FTP session error on " + basenames[i] + " — not counted");
    } else {
      uploaded++;
    }

    // Report progress in 10% steps instead of one line per file.
    int decile = (int)(((i + 1) * 10) / nFiles);
    if (decile > lastLoggedDecile) {
      lastLoggedDecile = decile;
      writeLogFile(String(decile * 10) + "% of files transferred");
    }
  }

  ftpUp->CloseConnection();
  delete ftpUp;
  return uploaded;
}

// POST /transfercomplete/<id> with {"files_sent": N}. The server verifies the
// files on disk and answers {"success": true} — only then is the transfer done.
static bool confirmTransfer(int nSent) {
  HTTPClient http;
  http.begin(serverUrl(String("/transfercomplete/") + String(idBuoy)));
  http.addHeader("Content-Type", "application/json");
  String body = "{\"files_sent\":" + String(nSent) + "}";
  int code = http.POST(body);
  String resp = http.getString();
  http.end();
  bool ok = (code == 200) && (resp.indexOf("\"success\": true") >= 0 || resp.indexOf("\"success\":true") >= 0);
  writeLogFile("confirmTransfer: code=" + String(code) + " success=" + String(ok));
  return ok;
}

// ---- Main entry point ----------------------------------------------------

int tryUploadDataToUSV() {
  /* Connects the buoy to the BlueBoat USV and uploads SD data.
     Protocol:
       1. Connect to BlueBoat WiFi AP        → -1 if fail
       2. Sync RTC, verify server identity   → -2 if not blueboat
       3. Request upload permission          → -3 if denied
       4. PUT file manifest, get wanted list → -4 if fail
       5. FTP STOR each requested file        → -6 if incomplete
       6. POST transfer-complete              → -5 if unconfirmed, else 0
  */

  writeLogFile("tryUploadDataToUSV: starting");

  // Step 1: WiFi
  if (!connectToRaspWiFi()) {
    writeLogFile("tryUploadDataToUSV: no WiFi");
    return -1;
  }

  // Step 2: RTC sync + server identity
  if (!sendHttpGetRequest(idBuoy, GETTIME, releaseFlag, releaseMode, sleeptime_h, sleeptime_m)) {
    writeLogFile("tryUploadDataToUSV: RTC sync failed (non-fatal)");
  }
  String serverId = getServerID();
  if (serverId != "blueboat") {
    writeLogFile("tryUploadDataToUSV: server is not blueboat (got '" + serverId + "')");
    return -2;
  }
  writeLogFile("tryUploadDataToUSV: server identified as blueboat");

  // Step 3: Upload permission
  if (!getUploadPermission()) {
    writeLogFile("tryUploadDataToUSV: upload permission denied");
    return -3;
  }
  writeLogFile("tryUploadDataToUSV: permission granted");

  // Step 4: File manifest — no fixed cap, grows on the heap to fit whatever's on the card.
  std::vector<String> wantedBasenames;
  std::vector<String> wantedSDPaths;
  int nWanted = sendManifestAndGetWantedFiles(wantedBasenames, wantedSDPaths);
  if (nWanted < 0) {
    writeLogFile("tryUploadDataToUSV: manifest exchange failed");
    return -4;
  }
  writeLogFile("tryUploadDataToUSV: server wants " + String(nWanted) + " files");

  if (nWanted == 0) {
    writeLogFile("tryUploadDataToUSV: server already has all files — confirming");
    confirmTransfer(0);
    return 0;
  }

  // Step 5: FTP upload
  int uploaded = uploadFilesViaFTP(wantedSDPaths, wantedBasenames);
  writeLogFile("tryUploadDataToUSV: uploaded " + String(uploaded) + "/" + String(nWanted) + " files");

  if (uploaded < nWanted) {
    // Partial upload: do NOT confirm and do NOT return 0 — the transfer is
    // not complete, so the server must not be told it is. Fail instead: the
    // next cycle re-manifests and the server re-requests whatever it is
    // still missing.
    writeLogFile("tryUploadDataToUSV: incomplete upload — retrying next cycle");
    return -6;
  }

  // Step 6: Confirm
  if (!confirmTransfer(uploaded)) {
    writeLogFile("tryUploadDataToUSV: server did not confirm receipt");
    return -5;
  }

  writeLogFile("tryUploadDataToUSV: complete!");
  return 0;
}
