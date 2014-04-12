#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <string>
#include <cstring>
#include <exception>
#include <stdexcept>
#include "device_driver_base/driver_util.h"
#include <boost/shared_ptr.hpp>


namespace device_driver{

/**
 * @author Mitchell Wills
 * @brief An exception indicating that a function timed out while reading bytes from the device
 */
class TimeoutException : public Exception{
 public:
  TimeoutException(const std::string& msg):Exception(msg){}
};
/**
 * @author Mitchell Wills
 * @brief An exception indicating that corrupt data was recieved from the device
 */
class CorruptDataException : public Exception{
 public:
  CorruptDataException(const std::string msg):Exception(msg){}
};

#define BAUD(rate) case rate: return B##rate
/**
 * Converts an integer to the internal bit representation of a baud rate
 * @param baud an integer representation of a buad rate
 * @return the bit representation of the given baud rate
 */
static inline speed_t to_bit_speed(int baud){
  switch(baud){
    BAUD(50);
    BAUD(75);
    BAUD(110);
    BAUD(134);
    BAUD(150);
    BAUD(200);
    BAUD(300);
    BAUD(600);
    BAUD(1200);
    BAUD(1800);
    BAUD(2400);
    BAUD(4800);
    BAUD(9600);
    BAUD(19200);
    BAUD(38400);
    BAUD(57600);
    BAUD(115200);
    BAUD(230400);
    BAUD(460800);
    BAUD(500000);
    BAUD(576000);
    BAUD(921600);
    BAUD(1000000);
    BAUD(1152000);
    BAUD(1500000);
    BAUD(2000000);
    BAUD(2500000);
    BAUD(3000000);
    BAUD(3500000);
    BAUD(4000000);
  default:
    DRIVER_EXCEPT(IllegalArgumentException, "Invalid buad rate: %d", baud);
  }
}



/**
 * the type expected for the char size of a serial connection
 */
typedef uint8_t char_size_t;
/**
 * An enumeration of the possible parity configurations for a serial port
 */
typedef enum {
  serial_parity_odd,
  serial_parity_even,
  serial_parity_none
} serial_parity_t;

/**
 * A class which defines common methods for interacting with a serial port for creating ros drivers
 */
class DriverSerialPort{
 public:
  /**
   * Creates a new serial port object
   * You must call open before you can use the serial port
   */
  DriverSerialPort():fd_(-1){}

  /**
   * @brief Opens the serial port
   * @param port the serial port to open (such as /dev/ttyUSB0)
   * @param baudrate the baud rate to use with the serial port (this is the bit representation, not an integer). Use the to_bit_speed function to convert from an integer.
   * @param char_size the character size to use with the serial port (5, 6, 7, 8)
   * @param parity the parity to use with the serial port
   */
  void open(const std::string& port, speed_t baudrate, char_size_t char_size, serial_parity_t parity);
  /**
   * Close the serial port
   */
  void close();
  /**
   * @return true if the serial port is open
   */
  bool is_open(){return fd_!=-1;}


  /**
   * @brief Read a number of bytes from the serial port into a buffer
   * @param buf [out] the buffer which the read values will be stored into
   * @param size [in] the number of bytes to read
   * @param timeout [in] the number of ms to wait before timing out on the read
   * @return the numbre of bytes read
   */
  int read(void* buf, size_t size, unsigned int timeout);
  /**
   * @brief Read a number of bytes from the serial port into a buffer
   * @param buf [out] the buffer which the read values will be stored into
   * @param size [in] the number of bytes to read
   * @param end_time [in] the time when the read should timeout
   * @return the number of bytes read
   */
  int read(void* buf, size_t size, struct timeval* end_time);
  /**
   * @brief Read bytes from the serial port until a specific byte is reached.
   * A null terminator will replace the end_char
   * @param buf [out] the buffer which values are read into
   * @param size [in] the size of the buffer, if this number of bytes is read an exception will be raised
   * @param end_char [in] the character to read until (this character will be read and then replaced with a null terminator
   * @param timeout [in] the number of ms to wait before timing out on the read
   * @return the number of bytes read (this will not include the end character)
   */
  int read_until(char* buf, size_t size, char end_char, unsigned int timeout);
  /**
   * @brief Read bytes from the serial port until reaching a given set of header bytes at which those bytes and more will be read into the buffer
   * The bytes in the header will be placed in the beginning bytes of the buffer
   * @param header [in] the bytes the should be looked for as a header of  message
   * @param header_size [in] the length of the header in bytes
   * @param buf [out] the buffer which values are read into
   * @param total_size [in] the size of the buffer including header size
   * @param timeout [in] the number of ms to wait before timing out on the read
   * @return the number of bytes read
   */
  int read_from_header(const uint8_t* header, size_t header_size, void* buf, size_t total_size, unsigned int timeout);

  /**
   * @brief Write bytes from a buffer to the serial port
   * @param buf [in] the buffer from which to write bytes
   * @param size [in] the number of bytes to read from the buffer
   * @param timeout [in] the number of ms to wait before timing out on the write
   * @return the number of bytes written
   */
  int write(const void* buf, size_t size, unsigned int timeout);
  /**
   * @brief Write bytes from a c string to the serial port
   * @param c_str [in] a c string to write to the serial port
   * @param timeout [in] the number of ms to wait before timing out on the write
   * @return the number of bytes written
   */
  int write(const char* c_str, unsigned int timeout);
  /**
   * @brief Write bytes from a string to the serial port
   * @param str [in] a string to write to the serial port
   * @param timeout [in] the number of ms to wait before timing out on the write
   * @return the number of bytes written
   */
  int write(const std::string str, unsigned int timeout);
  /**
   * @brief Write bytes to the serial port like printf
   * @param timeout [in] the number of ms to wait before timing out on the write
   * @param format [in] a printf style format for the vararg arguments
   * @return the number of bytes written
   */
  int writef(unsigned int timeout, const char* format, ...);


  /*
   * Private methods
   */
 private:
  int select_read(struct timeval* end_time);
  int select_write(struct timeval* end_time);
  int _select_end(fd_set* read_fds, fd_set* write_fds, fd_set* error_fds, struct timeval* end_time);
  int _read(void* buf, size_t size);
  int _write(const void* buf, size_t size);

  /*
   * Private fields
   */
 private:
  int fd_;

};
typedef boost::shared_ptr<DriverSerialPort> DriverSerialPortPtr;

}

#endif//SERIAL_PORT_H_

