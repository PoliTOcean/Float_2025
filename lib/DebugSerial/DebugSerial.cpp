#include "DebugSerial.h"
#include <stdarg.h>

// Global debug serial instance
DebugSerial Debug;

DebugSerial::DebugSerial() {
  debug_mode_ptr = nullptr;
  send_message_func = nullptr;
}

void DebugSerial::begin(bool* debug_mode_flag, uint8_t (*send_func)(const char*, uint32_t)) {
  debug_mode_ptr = debug_mode_flag;
  send_message_func = send_func;
}

void DebugSerial::print(const char* str) {
  if (debug_mode_ptr && *debug_mode_ptr && send_message_func) {
    send_message_func(str, 100);
  }
  Serial.print(str);
}

void DebugSerial::print(const String& str) {
  print(str.c_str());
}

void DebugSerial::print(int value) {
  char buffer[12];
  itoa(value, buffer, 10);
  print(buffer);
}

void DebugSerial::print(long value) {
  char buffer[20];
  ltoa(value, buffer, 10);
  print(buffer);
}

void DebugSerial::print(unsigned int value) {
  char buffer[12];
  utoa(value, buffer, 10);
  print(buffer);
}

void DebugSerial::print(unsigned long value) {
  char buffer[20];
  ultoa(value, buffer, 10);
  print(buffer);
}

void DebugSerial::print(float value, int digits) {
  char buffer[32];
  dtostrf(value, 0, digits, buffer);
  print(buffer);
}

void DebugSerial::print(double value, int digits) {
  print((float)value, digits);
}

void DebugSerial::println(const char* str) {
  if (debug_mode_ptr && *debug_mode_ptr && send_message_func) {
    char buffer[DEBUG_OUTPUT_LEN];
    snprintf(buffer, DEBUG_OUTPUT_LEN, "%s\n", str);
    send_message_func(buffer, 100);
  }
  Serial.println(str);
}

void DebugSerial::println(const String& str) {
  println(str.c_str());
}

void DebugSerial::println(int value) {
  char buffer[12];
  itoa(value, buffer, 10);
  println(buffer);
}

void DebugSerial::println(long value) {
  char buffer[20];
  ltoa(value, buffer, 10);
  println(buffer);
}

void DebugSerial::println(unsigned int value) {
  char buffer[12];
  utoa(value, buffer, 10);
  println(buffer);
}

void DebugSerial::println(unsigned long value) {
  char buffer[20];
  ultoa(value, buffer, 10);
  println(buffer);
}

void DebugSerial::println(float value, int digits) {
  char buffer[32];
  dtostrf(value, 0, digits, buffer);
  println(buffer);
}

void DebugSerial::println(double value, int digits) {
  println((float)value, digits);
}

void DebugSerial::println() {
  if (debug_mode_ptr && *debug_mode_ptr && send_message_func) {
    send_message_func("\n", 100);
  }
  Serial.println();
}

void DebugSerial::printf(const char* format, ...) {
  char buffer[DEBUG_OUTPUT_LEN];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, DEBUG_OUTPUT_LEN, format, args);
  va_end(args);
  print(buffer);
}

void DebugSerial::directPrint(const char* str) {
  Serial.print(str);
}

void DebugSerial::directPrintln(const char* str) {
  Serial.println(str);
}
