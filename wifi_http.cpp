#include "wifi_http.h"
#include "conf.h"
#include "secrets.h"
#include "logging.h"
#include <WiFi.h>
#include <HTTPClient.h>

// Globals owned by the main sketch (popup-buoy.ino).
extern int syncTime;
extern int maxWIFITimeout;
extern int year_lander, month_lander, day_lander, hour_lander, minute_lander, second_lander;

bool parseTimeResponse(const String &payload, int &year, int &month, int &day, int &hour, int &minute, int &second) {
    // Verificar si el campo "success" es true
    if (payload.indexOf("\"success\": true") == -1) {
        writeLogFile("Request failed: Time parsing failed due to missing success flag.");
        return false;
    }

    // Extraer el año
    int yearStart = payload.indexOf("\"year\":");
    if (yearStart != -1) {
        yearStart = payload.indexOf(":", yearStart) + 1;
        year = payload.substring(yearStart, payload.indexOf(",", yearStart)).toInt();
        //SerialPrintDebugln("Year is: " + String(year));
    } else {
        writeLogFile("Missing year in response");
        return false;
    }

    // Extraer el mes
    int monthStart = payload.indexOf("\"month\":");
    if (monthStart != -1) {
        monthStart = payload.indexOf(":", monthStart) + 1;
        month = payload.substring(monthStart, payload.indexOf(",", monthStart)).toInt();
        //SerialPrintDebugln("Month is: " + String(month));
    } else {
        writeLogFile("Missing month in response");
        return false;
    }

    // Extraer el día
    int dayStart = payload.indexOf("\"day\":");
    if (dayStart != -1) {
        dayStart = payload.indexOf(":", dayStart) + 1;
        day = payload.substring(dayStart, payload.indexOf(",", dayStart)).toInt();
        //SerialPrintDebugln("Day is: " + String(day));
    } else {
        writeLogFile("Missing day in response");
        return false;
    }

    // Extraer la hora
    int hourStart = payload.indexOf("\"hour\":");
    if (hourStart != -1) {
        hourStart = payload.indexOf(":", hourStart) + 1;
        hour = payload.substring(hourStart, payload.indexOf(",", hourStart)).toInt();
        //SerialPrintDebugln("Hour is: " + String(hour));
    } else {
        writeLogFile("Missing hour in response");
        return false;
    }

    // Extraer el minuto
    int minuteStart = payload.indexOf("\"minute\":");
    if (minuteStart != -1) {
        minuteStart = payload.indexOf(":", minuteStart) + 1;
        minute = payload.substring(minuteStart, payload.indexOf(",", minuteStart)).toInt();
        //SerialPrintDebugln("Minute is: " + String(minute));
    } else {
        writeLogFile("Missing minute in response");
        return false;
    }

    // Extraer el segundo
    int secondStart = payload.indexOf("\"second\":");
    if (secondStart != -1) {
        secondStart = payload.indexOf(":", secondStart) + 1;
        second = payload.substring(secondStart, payload.indexOf("}", secondStart)).toInt();
        //SerialPrintDebugln("Second is: " + String(second));
    } else {
        writeLogFile("Missing second in response");
        return false;
    }

    // Log de éxito
    String log = "Parsed time response successfully. ";
    log += "Year: " + String(year) + ", ";
    log += "Month: " + String(month) + ", ";
    log += "Day: " + String(day) + ", ";
    log += "Hour: " + String(hour) + ", ";
    log += "Minute: " + String(minute) + ", ";
    log += "Second: " + String(second);

    writeLogFile(log);
    return true;
}

bool parsePermissionResponse(const String &payload, int &releaseFlag, ReleaseMode &releaseMode, int &sleeptime_h, int &sleeptime_m) {
    // Verificar si el campo "success" es true
    if (payload.indexOf("\"success\": true") == -1) {
        // Extraer el mensaje de error si "success" no es true
        int messageStart = payload.indexOf("\"message\":\"");
        if (messageStart != -1) {
            messageStart += 10;  // Ajustar posición al valor del mensaje
            int messageEnd = payload.indexOf("\"", messageStart);
            String message = payload.substring(messageStart, messageEnd);
            String log = "Request failed: " + message;
            writeLogFile(log);
        } else {
            writeLogFile("Request failed: Unknown error");
        }
        return false;
    }

    // Extraer releaseFlag (manejar espacios antes del valor)
    int flagStart = payload.indexOf("\"releaseFlag\":");
    if (flagStart != -1) {
        flagStart = payload.indexOf(":", flagStart) + 1;  // Ajustar posición al valor
        releaseFlag = payload.substring(flagStart, payload.indexOf(",", flagStart)).toInt();
        //SerialPrintDebugln("releaseFlag is " + String(releaseFlag));
    } else {
        writeLogFile("Missing releaseFlag in response");
        return false;
    }

    // Extraer releaseMode
    int modeStart = payload.indexOf("\"releaseMode\": ");
    if (modeStart != -1) {
        modeStart = payload.indexOf(":", modeStart) + 2;  // Ajustar posición al valor, +2 para incluir el siguiente "
        int modeEnd = payload.indexOf(",", modeStart);
        String releaseModeStr = payload.substring(modeStart, modeEnd); // Sin trim()
        //SerialPrintDebugln("releaseModeStr is: " + releaseModeStr);
        if (releaseModeStr == "\"FRM\"") {
            releaseMode = FRM;
        } else if (releaseModeStr == "\"DM\"") {
            releaseMode = DM;
        } else {
            writeLogFile("Unknown releaseMode: " + releaseModeStr + ". Forcing releaseMode to DM.");
            releaseMode = DM;
        }
    } else {
        writeLogFile("Missing releaseMode in response");
        return false;
    }

    // Extraer sleeptime_h (manejar espacios antes del valor)
    int sleepHStart = payload.indexOf("\"sleeptime_h\":");
    if (sleepHStart != -1) {
        sleepHStart = payload.indexOf(":", sleepHStart) + 3;  // Ajustar posición al valor
        String sleeptime_h_str = payload.substring(sleepHStart, payload.indexOf(",", sleepHStart)-1);
        //SerialPrintDebugln("sleeptime_h is: " + sleeptime_h_str);
        sleeptime_h =  sleeptime_h_str.toInt();
    } else {
        writeLogFile("Missing sleeptime_h in response");
        return false;
    }

    // Extraer sleeptime_m (manejar espacios antes del valor)
    int sleepMStart = payload.indexOf("\"sleeptime_m\":");
    if (sleepMStart != -1) {
        sleepMStart = payload.indexOf(":", sleepMStart) + 3;  // Ajustar posición al valor
        String sleeptime_m_str = payload.substring(sleepMStart, payload.indexOf("}", sleepMStart)-1);
        //SerialPrintDebugln("sleeptime_m is: " + sleeptime_m_str);
        sleeptime_m = sleeptime_m_str.toInt();
    } else {
        writeLogFile("Missing sleeptime_m in response");
        return false;
    }

    // Log para indicar éxito con los valores extraídos
    String log = "Parsed permission response successfully. ";
    log += "releaseFlag: " + String(releaseFlag) + ", ";
    log += "releaseMode: " + String(releaseMode == FRM ? "FRM" : "DM") + ", ";
    log += "sleeptime_h: " + String(sleeptime_h) + ", ";
    log += "sleeptime_m: " + String(sleeptime_m);

    writeLogFile(log);
    return true;
}

bool parseSyncTimeResponse(const String& payload) {
    // Verificar si el campo "sync_time" está presente
    int syncTimeStart = payload.indexOf("\"sync_time\":");
    if (syncTimeStart != -1) {
        syncTimeStart += 12;  // Ajustar posición al valor
        syncTime = payload.substring(syncTimeStart, payload.indexOf("}", syncTimeStart)).toInt();
        SerialPrintDebugln("syncTime is " + String(syncTime));
        writeLogFile("Parsed syncTime successfully. syncTime: " + String(syncTime));
        return true;
    } else {
        writeLogFile("Missing syncTime in response");
        return false;
    }
}

bool sendHttpGetRequest(int idBoia, ActionType action, int &releaseFlag, ReleaseMode &releaseMode, int &sleeptime_h, int &sleeptime_m) {
  HTTPClient http;
  String response;
  String action_string;
  String url;

  switch (action) {
    case RELEASE:
      action_string = "/release/";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string + String(idBoia);
      break;
    case PERMISSION:
      action_string = "/permission/";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string + String(idBoia);
      break;
    case GETTIME:
      action_string = "/gettime";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string;
      break;
    case GETSYNCTIME:
      action_string = "/getsynctime";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string;
      break;
    default:
      action_string = "/unknown/";
      // Construir la URL con la dirección IP, puerto y el número de GPIO
      url = "http://" + String(SECRET_FTP_SERVER_IP) + ":" + String(SECRET_FTP_SERVER_PORT) + action_string + String(idBoia);
      break;
  }


  // Comenzar la conexión HTTP
  http.begin(url);

  // Realizar la solicitud GET
  int httpResponseCode = http.GET();

  switch (action){
    case RELEASE:
      // Verificar el código de respuesta
      if (httpResponseCode == 200) {
        response = "HTTP response successful: " + String(httpResponseCode)+ " " + http.getString();
        writeLogFile(response);
        http.end();
        return true;
      } else {
        response = "Error in the HTTP request: " + String(httpResponseCode)+ " " + http.getString();
        writeLogFile(response);
        http.end();
        return false;
      }
      break;
    case PERMISSION:
      if (httpResponseCode == 200) {
        String payload = http.getString();  // Obtén la respuesta como string

        // Llamar a la subfunción para parsear el payload
        bool parsepermissionSuccess = parsePermissionResponse(payload, releaseFlag, releaseMode, sleeptime_h, sleeptime_m);
        //writeLogFile("Message parsed. releaseFlag = " + String(releaseFlag) + ", releaseMode = " + String (releaseMode) + ", sleeptime_h = " + String(sleeptime_h) + " and sleeptime_m = " + String(sleeptime_m));

        http.end();  // Liberar recursos
        return parsepermissionSuccess;
      } else {
        // Si el código de respuesta no es 200
        response = "Error in the HTTP request: "+ String(httpResponseCode)+ " "  + http.getString();
        writeLogFile(response);
        http.end();  // Liberar recursos
        return false;
      }
      break;
    case GETTIME:
      if (httpResponseCode == 200) {
        String payload = http.getString();  // Obtén la respuesta como string
        int year, month, day, hour, minute, second;
        // Llamar a la subfunción para parsear el payload
        bool parsetimeSuccess = parseTimeResponse(payload, year, month, day, hour, minute, second);
        //{"success": true, "current_time": {"year": 2024, "month": 10, "day": 10, "hour": 10, "minute": 40, "second": 34}}
        year_lander=year;
        month_lander=month;
        day_lander=day;
        hour_lander=hour;
        minute_lander=minute;
        second_lander=second;
        http.end();  // Liberar recursos
        return parsetimeSuccess;
      } else {
        // Si el código de respuesta no es 200
        response = "Error in the HTTP request: "+ String(httpResponseCode)+ " "  + http.getString();
        writeLogFile(response);
        http.end();  // Liberar recursos
        return false;
      }
      break;
    case GETSYNCTIME:
      if (httpResponseCode == 200) {
        String payload = http.getString();
        if (!parseSyncTimeResponse(payload)) {
          writeLogFile("Failed to parse sync_time");
          return false;
        }
        return true;
      } else {
        writeLogFile("HTTP GET failed for GETSYNCTIME");
        return false;
      }
      break;
    default:
      response = "Unknown action requested: " + String(action) + " - HTTP response code: " + String(httpResponseCode);
      writeLogFile(response);
      return false;
      break;
  }

  // Liberar recursos
  http.end();
  return false;
}

bool connectToRaspWiFi() {
  WiFi.begin(WIFI_SSID2, WIFI_PASS2);
  unsigned long startAttemptTime = millis();

  SerialPrintDebug("Conectando a WiFi...");

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < maxWIFITimeout) {
    delay(500);
    SerialPrintDebug(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    SerialPrintDebugln("Connected!");
    writeLogFile("Connected to Pop-Up server WiFi");
    return true;
  } else {
    SerialPrintDebugln("Not connected!");
    String response = "ERROR: Impossible to connect to Pop-Up server WiFi. Reason: " + getWiFiFailureReason(WiFi.status());
    writeLogFile(response);
    return false;
  }
}

String getWiFiFailureReason(int status) {
  switch (status) {
    case WL_IDLE_STATUS: return "Idle status";
    case WL_NO_SSID_AVAIL: return "No SSID available";
    case WL_SCAN_COMPLETED: return "Scan completed";
    case WL_CONNECT_FAILED: return "Connection failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED: return "Disconnected";
    default: return "Unknown";
  }
}
