// -------------------------------------------------------------------------- //
// ARRIBADA.h
// Minimal Arduino interface for Arribada Argos SMD AT firmware
// Supported operations: PING, get ID, get SN and transmit data
//
// The Arribada wing and the KIM1 shield plug into the same socket and share the
// same UART, so only one of them is ever present. sat_module.* picks the driver
// at run time; nothing outside it should talk to this class directly.
// -------------------------------------------------------------------------- //

#ifndef ARRIBADA_H
#define ARRIBADA_H

#include <Arduino.h>

#ifndef ARRIBADA_BAUD
#define ARRIBADA_BAUD 9600
#endif

#ifndef ARRIBADA_TIMEOUT_MS
#define ARRIBADA_TIMEOUT_MS 30000
#endif

// MAC profile the buoy transmits with. 1 is the basic Kineis profile; the
// module powers up with 0, which refuses to transmit.
#ifndef ARRIBADA_KMAC_PROFILE
#define ARRIBADA_KMAC_PROFILE 1
#endif

typedef enum {
  ERROR_ARRIBADA         = 0x00,
  UNKNOWN_ERROR_ARRIBADA = 0x02,
  OK_ARRIBADA            = 0x01,
  TIMEOUT_ARRIBADA       = 0x04
} RetStatusARRIBADATypeDef;

class ARRIBADA {
 public:
  explicit ARRIBADA(HardwareSerial* device);

  // Initializes the UART. It is also called automatically before each command.
  // rxPin/txPin default to -1, meaning "use the core's default pins for this
  // UART". Always pass them on the buoy: UART2's default pins moved between
  // ESP32 core 2.x and 3.x, which is what silently broke KIM comms once.
  void begin(uint32_t baud = ARRIBADA_BAUD, int8_t rxPin = -1, int8_t txPin = -1);

  // Releases the UART and tri-states the pins, so the powered-down module is
  // not back-fed through its RX pin.
  void end();

  // Sends AT+PING and checks for +OK. The KIM1 firmware has no PING command,
  // which is what makes this a reliable way to tell the two modules apart.
  bool check();

  // Returns the complete value response:
  //   +ID=<decimal ID>
  //   +SN=<serial number>
  // The returned pointer remains valid until the next command.
  char* get_ID();
  char* get_SN();

  // Reads the radio configuration: "+RCONF=<min_freq>,<max_freq>,<rf_level>,<modulation>".
  //
  // Read only on purpose. There is no setter here and there should not be one:
  // AT+RCONF is not a RAM setting, it erases and reprograms the flash page that
  // also holds the device ID, address and secret key. On the module measured
  // here it fails outright with +ERROR=700 (flash error) and AT+SAVE_RCONF is a
  // do-nothing stub in the vendor source, so the configuration can only be
  // changed by reflashing the module's own firmware. The buoy checks it and
  // complains in the log; a human fixes it.
  char* get_RCONF();

  // Selects the Kineis MAC profile (AT+KMAC=<profile>).
  //
  // This is not optional and it is not persistent. The module boots with
  // +KMAC=0 and answers +ERROR=253 to every AT+TX in that state; measured on
  // this hardware, the setting is also lost whenever the module loses power.
  // Since the buoy drops the module's supply between messages, it has to be
  // re-sent before each transmission - satModuleSendData() does that.
  RetStatusARRIBADATypeDef set_KMAC(uint8_t profile = ARRIBADA_KMAC_PROFILE);

  // Sends AT+TX=<hex payload>.
  // Returns OK when the module accepts/queues the command (+OK).
  // It does not wait for the later +TX=0,<payload> RF completion message.
  //
  // The payload length must already be a multiple of 8 hex characters; the
  // firmware answers +ERROR=1100 otherwise. satModuleSendData() takes care of
  // that, so call through sat_module.* rather than here.
  RetStatusARRIBADATypeDef send_data(const char data[], uint16_t len);

  // Converts a byte array into an uppercase hexadecimal C string.
  static void uint2hexString(const uint8_t* input, uint16_t len, char* output);

 private:
  HardwareSerial* arribadaSerial;
  uint32_t baudRate;
  int8_t rxPinUsed;
  int8_t txPinUsed;
  bool uartStarted;

  char response[128];
  char command[160];

  RetStatusARRIBADATypeDef send_ATCommand(
      const char* commandToSend,
      const char* expectedPrefix,
      uint32_t timeoutMs = ARRIBADA_TIMEOUT_MS);

  void clearSerial();
  bool readLine(char* output, size_t outputSize, uint32_t timeoutMs);
  static void trimLine(char* text);
};

#endif
