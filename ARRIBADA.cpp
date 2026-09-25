// -------------------------------------------------------------------------- //
// ARRIBADA.cpp
// Minimal Arduino interface for Arribada Argos SMD AT firmware
// -------------------------------------------------------------------------- //

#include "ARRIBADA.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

ARRIBADA::ARRIBADA(HardwareSerial* device)
    : arribadaSerial(device),
      baudRate(ARRIBADA_BAUD),
      rxPinUsed(-1),
      txPinUsed(-1),
      uartStarted(false),
      txConfirmed(false) {
  response[0] = '\0';
  command[0] = '\0';
}

void ARRIBADA::begin(uint32_t baud, int8_t rxPin, int8_t txPin) {
  baudRate = baud;
  rxPinUsed = rxPin;
  txPinUsed = txPin;

  if (arribadaSerial == nullptr) {
    uartStarted = false;
    return;
  }

  if (rxPin >= 0 && txPin >= 0) {
    arribadaSerial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
  } else {
    arribadaSerial->begin(baudRate);
  }

  arribadaSerial->setTimeout(ARRIBADA_TIMEOUT_MS);
  uartStarted = true;
  clearSerial();
}

void ARRIBADA::end() {
  if (arribadaSerial == nullptr || !uartStarted) {
    return;
  }

  arribadaSerial->end();
  uartStarted = false;

  if (rxPinUsed >= 0 && txPinUsed >= 0) {
    pinMode(rxPinUsed, INPUT);
    pinMode(txPinUsed, INPUT);
  }
}

void ARRIBADA::clearSerial() {
  if (arribadaSerial == nullptr) {
    return;
  }

  arribadaSerial->flush();

  while (arribadaSerial->available() > 0) {
    arribadaSerial->read();
  }
}

void ARRIBADA::trimLine(char* text) {
  if (text == nullptr) {
    return;
  }

  size_t len = strlen(text);

  while (len > 0 &&
         (text[len - 1] == '\r' ||
          text[len - 1] == '\n' ||
          isspace(static_cast<unsigned char>(text[len - 1])))) {
    text[--len] = '\0';
  }

  size_t start = 0;
  while (text[start] != '\0' &&
         isspace(static_cast<unsigned char>(text[start]))) {
    start++;
  }

  if (start > 0) {
    memmove(text, text + start, strlen(text + start) + 1);
  }
}

bool ARRIBADA::readLine(char* output,
                        size_t outputSize,
                        uint32_t timeoutMs) {
  if (arribadaSerial == nullptr || output == nullptr || outputSize == 0) {
    return false;
  }

  const uint32_t startTime = millis();
  size_t index = 0;
  output[0] = '\0';

  while ((millis() - startTime) < timeoutMs) {
    while (arribadaSerial->available() > 0) {
      const int value = arribadaSerial->read();

      if (value < 0) {
        continue;
      }

      const char received = static_cast<char>(value);

      if (received == '\n') {
        output[index] = '\0';
        trimLine(output);

        if (output[0] != '\0') {
          return true;
        }

        index = 0;
        output[0] = '\0';
        continue;
      }

      if (received != '\r' && index < outputSize - 1) {
        output[index++] = received;
        output[index] = '\0';
      }
    }

    delay(1);
  }

  output[index] = '\0';
  trimLine(output);
  return output[0] != '\0';
}

RetStatusARRIBADATypeDef ARRIBADA::send_ATCommand(
    const char* commandToSend,
    const char* expectedPrefix,
    uint32_t timeoutMs) {
  if (arribadaSerial == nullptr || commandToSend == nullptr) {
    return ERROR_ARRIBADA;
  }

  if (!uartStarted) {
    begin(baudRate, rxPinUsed, txPinUsed);
  }

  clearSerial();
  response[0] = '\0';

  // Arribada's Arduino sample uses println(), therefore commands end in CRLF.
  arribadaSerial->print(commandToSend);
  arribadaSerial->print("\r\n");

  const uint32_t startTime = millis();
  bool expectedResponseFound = (expectedPrefix == nullptr);

  while ((millis() - startTime) < timeoutMs) {
    char line[128];

    const uint32_t elapsed = millis() - startTime;
    const uint32_t remaining =
        (elapsed < timeoutMs) ? (timeoutMs - elapsed) : 0;

    if (!readLine(line, sizeof(line), remaining)) {
      break;
    }

    if (strncmp(line, "+ERROR=", 7) == 0) {
      strncpy(response, line, sizeof(response) - 1);
      response[sizeof(response) - 1] = '\0';
      return ERROR_ARRIBADA;
    }

    if (expectedPrefix != nullptr &&
        strncmp(line, expectedPrefix, strlen(expectedPrefix)) == 0) {
      strncpy(response, line, sizeof(response) - 1);
      response[sizeof(response) - 1] = '\0';
      expectedResponseFound = true;
      continue;
    }

    if (strcmp(line, "+OK") == 0) {
      if (expectedResponseFound) {
        // PING has no data line, so preserve +OK as its response.
        if (expectedPrefix == nullptr) {
          strncpy(response, line, sizeof(response) - 1);
          response[sizeof(response) - 1] = '\0';
        }
        return OK_ARRIBADA;
      }

      // A response was syntactically successful but the requested data
      // prefix was not received.
      return UNKNOWN_ERROR_ARRIBADA;
    }

    // AT+TX may later produce +TX=0,<payload>. send_data() intentionally
    // returns on the immediate +OK and does not wait for RF completion.
  }

  return TIMEOUT_ARRIBADA;
}

// Note on syntax: the wiki writes the query form as "AT+FW?", but the firmware
// measured here (5ad8cd5_Tx_gui_basic, Oct 2025) wants "AT+ID=?" and answers
// +ERROR=1203 to "AT+ID?" - the same "=?" form the KIM1 uses. The "?" variants
// are kept as a fallback in case another build flips it back.

bool ARRIBADA::check() {
  // AT+ID is used rather than AT+PING because it is confirmed present on this
  // firmware, while several documented commands (AT+RCONF, AT+LPM, AT+VERSION)
  // come back as +ERROR=1203 on it.
  if (send_ATCommand("AT+ID=?", "+ID=", 3000) == OK_ARRIBADA) {
    return true;
  }
  return send_ATCommand("AT+PING=?", nullptr, 3000) == OK_ARRIBADA;
}

char* ARRIBADA::get_ID() {
  if (send_ATCommand("AT+ID=?", "+ID=") != OK_ARRIBADA) {
    send_ATCommand("AT+ID?", "+ID=");
  }
  return response;
}

char* ARRIBADA::get_SN() {
  if (send_ATCommand("AT+SN=?", "+SN=") != OK_ARRIBADA) {
    send_ATCommand("AT+SN?", "+SN=");
  }
  return response;
}

char* ARRIBADA::get_RCONF() {
  if (send_ATCommand("AT+RCONF=?", "+RCONF=") != OK_ARRIBADA) {
    send_ATCommand("AT+RCONF?", "+RCONF=");
  }
  return response;
}

RetStatusARRIBADATypeDef ARRIBADA::set_KMAC(uint8_t profile) {
  snprintf(command, sizeof(command), "AT+KMAC=%u", (unsigned)profile);
  return send_ATCommand(command, nullptr, 5000);
}

int ARRIBADA::get_KMAC() {
  if (send_ATCommand("AT+KMAC=?", "+KMAC=") != OK_ARRIBADA) return -1;
  const char *p = strstr(response, "+KMAC=");
  if (p == nullptr || !isdigit((unsigned char)p[6])) return -1;
  return atoi(p + 6);
}

RetStatusARRIBADATypeDef ARRIBADA::send_data(
    const char data[],
    uint16_t len) {
  txConfirmed = false;

  if (data == nullptr || len == 0) {
    return ERROR_ARRIBADA;
  }

  // LDA2/VLD carry at most 24 bytes (48 hex characters); keep the command
  // buffer protected even if another firmware profile is used.
  if (len > sizeof(command) - strlen("AT+TX=") - 1) {
    return ERROR_ARRIBADA;
  }

  const int written = snprintf(
      command,
      sizeof(command),
      "AT+TX=%.*s",
      static_cast<int>(len),
      data);

  if (written < 0 || static_cast<size_t>(written) >= sizeof(command)) {
    return ERROR_ARRIBADA;
  }

  const RetStatusARRIBADATypeDef status =
      send_ATCommand(command, nullptr, ARRIBADA_TX_ACCEPT_TIMEOUT_MS);
  if (status != OK_ARRIBADA) {
    return status;
  }

  // The +OK above only says the module queued the command. The burst itself is
  // reported afterwards by a "+TX=0,<payload>" line. Consume it here for two
  // reasons: it tells the caller whether anything was actually radiated, and it
  // keeps the line out of the buffer, where the next command's clearSerial()
  // would swallow it.
  //
  // Not finding it is not treated as a failure - see the note in the header.
  const uint32_t startTime = millis();
  char line[128];

  while (true) {
    const uint32_t elapsed = millis() - startTime;
    if (elapsed >= ARRIBADA_TX_DONE_TIMEOUT_MS) {
      break;
    }

    if (!readLine(line, sizeof(line), ARRIBADA_TX_DONE_TIMEOUT_MS - elapsed)) {
      break;
    }

    if (strncmp(line, "+TX=", 4) == 0) {
      txConfirmed = true;
      strncpy(response, line, sizeof(response) - 1);
      response[sizeof(response) - 1] = '\0';
      break;
    }
  }

  return OK_ARRIBADA;
}

void ARRIBADA::uint2hexString(
    const uint8_t* input,
    uint16_t len,
    char* output) {
  if (input == nullptr || output == nullptr) {
    return;
  }

  for (uint16_t index = 0; index < len; index++) {
    sprintf(output + index * 2, "%02X", input[index]);
  }

  output[len * 2] = '\0';
}
