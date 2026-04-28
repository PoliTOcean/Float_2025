#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>

// Define buffer size for debug messages
#define DEBUG_OUTPUT_LEN 248  // Size of the output buffer for debug messages

class DebugSerial {
private:
  bool* debug_mode_ptr;  // Pointer to debug mode flag
  uint8_t (*send_message_func)(const char*, uint32_t);  // Function pointer for sending messages

public:
  // Constructor
  DebugSerial();
  
  // Initialization method
  void begin(bool* debug_mode_flag, uint8_t (*send_func)(const char*, uint32_t));
  
  // Print methods
  void print(const char* str);
  void print(const String& str);
  void print(int value);
  void print(long value);
  void print(unsigned int value);
  void print(unsigned long value);
  void print(float value, int digits = 2);
  void print(double value, int digits = 2);
  
  // Println methods
  void println(const char* str);
  void println(const String& str);
  void println(int value);
  void println(long value);
  void println(unsigned int value);
  void println(unsigned long value);
  void println(float value, int digits = 2);
  void println(double value, int digits = 2);
  void println();
  
  // Printf method
  void printf(const char* format, ...);
  
  // Direct serial access for non-debug output
  void directPrint(const char* str);
  void directPrintln(const char* str);
};

// Global debug serial instance
extern DebugSerial Debug;

#endif // DEBUG_SERIAL_H
