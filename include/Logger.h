#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <WiFiUdp.h>

#define LOG_BUFFER_SIZE 32
#define LOG_MESSAGE_MAX_LEN 128

enum LogLevel {
  LOG_INFO = 6,
  LOG_NOTICE = 5,
  LOG_WARNING = 4,
  LOG_ERR = 3
};

struct LogEntry {
  LogLevel level;
  char tag[16];
  char message[LOG_MESSAGE_MAX_LEN];
};

// Serial + optional syslog. Call only from the loop task.
class Logger {
public:
  Logger();

  // Syslog destination. ip 0.0.0.0 = syslog off (entries stay serial-only).
  void setTarget(IPAddress ip, uint16_t port);

  void log(LogLevel level, const char *tag, const char *fmt, ...);
  void info(const char *tag, const char *fmt, ...);
  void warn(const char *tag, const char *fmt, ...);
  void error(const char *tag, const char *fmt, ...);

  // Sends at most one queued entry per call
  void update();

private:
  void _add(LogLevel level, const char *tag, const char *msg);

  LogEntry _buffer[LOG_BUFFER_SIZE];
  int _head;
  int _tail;

  WiFiUDP _udp;
  IPAddress _target;
  uint16_t _port;
};

extern Logger logger;

#endif
