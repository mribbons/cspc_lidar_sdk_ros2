#ifndef NODE_SERIAL_H
#define NODE_SERIAL_H

#include "lidar_information.h"
#include <stdint.h>
#include <vector>

#include <string>
#include <atomic>
#include <limits>
#include <termios.h>

using namespace std;

class MillisecondTimer
{
public:
  explicit MillisecondTimer(const uint64_t millis);
  int64_t remaining();

private:
  static timespec timespec_now();
  timespec expiry;
};

struct Timeout
{
  static uint32_t max()
  {
    return std::numeric_limits<uint32_t>::max();
  }
  
  static Timeout simpleTimeout(uint32_t timeout_t)
  {
    return Timeout(max(), timeout_t, 0, timeout_t, 0);
  }

  uint32_t inter_byte_timeout;
  uint32_t read_timeout_constant;

  uint32_t read_timeout_multiplier;
  uint32_t write_timeout_constant;
 
  uint32_t write_timeout_multiplier;

  explicit Timeout(uint32_t inter_byte_timeout_ = 0,
                   uint32_t read_timeout_constant_ = 0,
                   uint32_t read_timeout_multiplier_ = 0,
                   uint32_t write_timeout_constant_ = 0,
                   uint32_t write_timeout_multiplier_ = 0)
      : inter_byte_timeout(inter_byte_timeout_),
        read_timeout_constant(read_timeout_constant_),
        read_timeout_multiplier(read_timeout_multiplier_),
        write_timeout_constant(write_timeout_constant_),
        write_timeout_multiplier(write_timeout_multiplier_)
  {
  }
};


typedef enum {
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

typedef enum {
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

typedef enum {
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

typedef enum {
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

class Serial_Port
{
private:
  string port_;
  
  unsigned long baudrate_;    // Baudrate
  int fd_;
  pid_t pid;
  bool is_open_ = false;
  uint32_t byte_time_ns_; // Nanoseconds to transmit/receive a single byte

  Timeout timeout_;           // Timeout for read operations

  parity_t parity_;           // Parity
  bytesize_t bytesize_;       // Size of the bytes
  stopbits_t stopbits_;       // Stop Bits
  flowcontrol_t flowcontrol_; // Flow Control

public:
  Serial_Port(const std::string &port = "",
                  uint32_t baudrate = 115200,
                  Timeout timeout = Timeout(),
                  bytesize_t bytesize = eightbits,
                  parity_t parity = parity_none,
                  stopbits_t stopbits = stopbits_one,
                  flowcontrol_t flowcontrol = flowcontrol_none);

  ~Serial_Port();
  bool open();
  void close();
  bool getTermios(termios *tio);

  /*设置数据位数*/
  void set_databits(termios *tio, bytesize_t databits);

  /*设置奇偶性*/
  void set_parity(termios *tio, parity_t parity);

  /*设置停止位*/
  void set_stopbits(termios *tio, stopbits_t stopbits);

  /*流量控制*/
  void set_flowcontrol(termios *tio,flowcontrol_t flowcontrol);

  void set_common_props(termios *tio);

  
  result_t read_data(uint8_t *buf, size_t size);
  size_t write_data(const uint8_t *data, size_t length);
  result_t waitForData(size_t data_count, uint64_t timeout, size_t *returned_size);
  size_t available();
  bool setDTR(bool level);

  /*获取串口传输一个byte时间*/
  uint32_t getByteTime();

  bool setBaudrate(unsigned long baudrate);
  bool setTermios(const termios *tio);
  bool setCustomBaudRate(unsigned long baudrate);
  bool waitReadable(uint32_t timeout_t);



};

#endif
