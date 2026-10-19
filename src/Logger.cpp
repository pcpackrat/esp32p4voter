#include "Logger.h"
#include <stdarg.h>

Logger logger;

Logger::Logger() {
  _head = 0;
  _tail = 0;
  _target = IPAddress(0, 0, 0, 0);
  _port = 514;
}

void Logger::setTarget(IPAddress ip, uint16_t port) {
  _target = ip;
  _port = port;
}

void Logger::_add(LogLevel level, const char *tag, const char *msg) {
  LogEntry &e = _buffer[_head];
  e.level = level;
  strncpy(e.tag, tag, sizeof(e.tag) - 1);
  e.tag[sizeof(e.tag) - 1] = '\0';
  strncpy(e.message, msg, sizeof(e.message) - 1);
  e.message[sizeof(e.message) - 1] = '\0';

  int nextHead = (_head + 1) % LOG_BUFFER_SIZE;
  if (nextHead == _tail) {
    _tail = (_tail + 1) % LOG_BUFFER_SIZE; // Full - drop oldest
  }
  _head = nextHead;

  const char *lvlStr = "INFO";
  if (level == LOG_WARNING)
    lvlStr = "WARN";
  if (level == LOG_ERR)
    lvlStr = "ERROR";
  Serial.printf("[LOG][%s][%s] %s\r\n", lvlStr, tag, msg);
}

void Logger::log(LogLevel level, const char *tag, const char *fmt, ...) {
  char buf[LOG_MESSAGE_MAX_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  _add(level, tag, buf);
}

void Logger::info(const char *tag, const char *fmt, ...) {
  char buf[LOG_MESSAGE_MAX_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  _add(LOG_INFO, tag, buf);
}

void Logger::warn(const char *tag, const char *fmt, ...) {
  char buf[LOG_MESSAGE_MAX_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  _add(LOG_WARNING, tag, buf);
}

void Logger::error(const char *tag, const char *fmt, ...) {
  char buf[LOG_MESSAGE_MAX_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  _add(LOG_ERR, tag, buf);
}

void Logger::update() {
  if (_target == IPAddress(0, 0, 0, 0))
    return;
  if (_tail == _head)
    return;

  const LogEntry &e = _buffer[_tail];
  char packet[192];
  int priority = (1 * 8) + (int)e.level; // facility: user
  int len = snprintf(packet, sizeof(packet), "<%d>%s: %s", priority, e.tag, e.message);
  if (len > (int)sizeof(packet) - 1)
    len = sizeof(packet) - 1;

  _udp.beginPacket(_target, _port);
  _udp.write((const uint8_t *)packet, len);
  _udp.endPacket();

  _tail = (_tail + 1) % LOG_BUFFER_SIZE;
}
