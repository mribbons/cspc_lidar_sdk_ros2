#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include <sys/select.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include "serial_port.h"
#include "node_lidar.h"

using namespace std;


#define SNCCS 19
#define BOTHER 0010000
struct termios2 {
  tcflag_t c_iflag;       /* input mode flags */
  tcflag_t c_oflag;       /* output mode flags */
  tcflag_t c_cflag;       /* control mode flags */
  tcflag_t c_lflag;       /* local mode flags */
  cc_t c_line;            /* line discipline */
  cc_t c_cc[SNCCS];       /* control characters */
  speed_t c_ispeed;       /* input speed */
  speed_t c_ospeed;       /* output speed */
};

Serial_Port::Serial_Port(const string &port, uint32_t baudrate, Timeout timeout,
               bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
               flowcontrol_t flowcontrol)
	:port_(port),baudrate_(baudrate),timeout_(timeout),bytesize_(bytesize),parity_(parity),stopbits_(stopbits),flowcontrol_(flowcontrol){

}

Serial_Port::~Serial_Port(){
  
}

MillisecondTimer::MillisecondTimer(const uint64_t millis) : expiry(timespec_now())
{
	int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);

	if (tv_nsec >= 1e9)
	{
		int64_t sec_diff = tv_nsec / static_cast<int>(1e9);
		expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
		expiry.tv_sec += sec_diff;
	}
	else
	{
		expiry.tv_nsec = tv_nsec;
	}
}

int64_t MillisecondTimer::remaining()
{
	timespec now(timespec_now());
	int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
	millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
	return millis;
}

timespec MillisecondTimer::timespec_now()
{
	timespec time;
	clock_gettime(CLOCK_MONOTONIC, &time);
	return time;
}

timespec timespec_from_ms(const uint32_t millis)
{
	timespec time;
	time.tv_sec = millis / 1e3;
	time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
	return time;
}

result_t Serial_Port::waitForData(size_t data_count, uint64_t timeout, size_t *returned_size)
{
	if(!is_open_)
	{
		return RESULT_FAIL;
	}

	size_t length = 0;

	if (returned_size == NULL)
	{
		returned_size = (size_t *)&length;
	}

	*returned_size = 0;

	if (is_open_)
	{
    	/*得到缓冲区里有多少字节*/
		if (ioctl(fd_, FIONREAD, returned_size) == -1)
		{
			return RESULT_FAIL;
		}
		if (*returned_size >= data_count)
		{
			return RESULT_OK;
		}
	}
	
	fd_set readfds;
	/*将set清零使集合中不含任何fd*/
	FD_ZERO(&readfds);
	/*将fd_加入set集合*/
	FD_SET(fd_, &readfds);

	MillisecondTimer total_timeout(timeout);

	while (is_open_)
	{
		int64_t timeout_remaining_ms = total_timeout.remaining();

		if ((timeout_remaining_ms <= 0))
		{
			return RESULT_TIMEOUT;
		}
		
		timespec timeout_val(timespec_from_ms(timeout_remaining_ms));

		/*检查文件描述符是否就绪*/
		int n = pselect(fd_+1, &readfds, NULL, NULL, &timeout_val,NULL);

		if (n < 0)
		{
			if (errno == EINTR)
			{
				return RESULT_TIMEOUT;
			}
			return RESULT_FAIL;
		}
		else if (n == 0)
		{
			return RESULT_TIMEOUT;
		}
		else
		{
			/*调用select后，检查fd_是否在set集合中*/
			if(FD_ISSET(fd_, &readfds)){
				if(ioctl(fd_, FIONREAD, returned_size)<0){
					return RESULT_FAIL;
				}
			
				if (*returned_size >= data_count)
				{
					return RESULT_OK;
				}else{
					int remain_timeout = timeout_val.tv_sec * 1000000 + timeout_val.tv_nsec /1000;
					int expect_remain_time = (data_count - *returned_size) * 1000000 * 8 / baudrate_;
					if (remain_timeout > expect_remain_time)
					{
						usleep(expect_remain_time);
					}
				}
			}else{
				usleep(30);
			}
		}
	}
	return RESULT_FAIL;
}

bool Serial_Port::waitReadable(uint32_t timeout_t)
{
	// Setup a select call to block for serial data or a timeout
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(fd_, &readfds);
	timespec timeout_ts(timespec_from_ms(timeout_t));
	int r = pselect(fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

	if (r < 0)
	{
		// Select was interrupted
		if (errno == EINTR)
		{
			return false;
		}

		// Otherwise there was some error
		return false;
	}

	// Timeout occurred
	if (r == 0)
	{
		return false;
	}

	// This shouldn't happen, if r > 0 our fd has to be in the list!
	if (!FD_ISSET(fd_, &readfds))
	{
		return false;
	}

	// Data available to read.
	return true;
}

size_t Serial_Port::available()
{
	if (!is_open_)
	{
		return 0;
	}

	int count = 0;

	if (-1 == ioctl(fd_, TIOCINQ, &count))
	{
		return 0;
	}else{
		return static_cast<size_t>(count);
	}
}

uint32_t byte_time_ns_;

void waitByteTimes(size_t count)
{
	timespec wait_time = {0, static_cast<long>(byte_time_ns_ * count)};
	pselect(0, NULL, NULL, NULL, &wait_time, NULL);
}

result_t Serial_Port::read_data(uint8_t *buf, size_t size)
{
	// If the port is not open, throw
	if (!is_open_)
	{
		return 0;
	}

	Timeout timeout_;

	size_t bytes_read = 0;

	// Calculate total timeout in milliseconds t_c + (t_m * N)
	long total_timeout_ms = timeout_.read_timeout_constant;
	total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long>(size);
	MillisecondTimer total_timeout(total_timeout_ms);

	// Pre-fill buffer with available bytes
	{
		ssize_t bytes_read_now = ::read(fd_, buf, size);

		if (bytes_read_now > 0)
		{
			bytes_read = bytes_read_now;
		}
	}

	while (bytes_read < size)
	{
		int64_t timeout_remaining_ms = total_timeout.remaining();

		if (timeout_remaining_ms <= 0)
		{
			// Timed out
			break;
		}

		// Timeout for the next select is whichever is less of the remaining
		// total read timeout and the inter-byte timeout.
		uint32_t timeout_t = std::min(static_cast<uint32_t>(timeout_remaining_ms),
									  timeout_.inter_byte_timeout);

		// Wait for the device to be readable, and then attempt to read.
		if (waitReadable(timeout_t))
		{
			// If it's a fixed-length multi-byte read, insert a wait here so that
			// we can attempt to grab the whole thing in a single IO call. Skip
			// this wait if a non-max inter_byte_timeout is specified.
			if (size > 1 && timeout_.inter_byte_timeout == Timeout::max())
			{
				size_t bytes_available = available();

				if (bytes_available + bytes_read < size)
				{
					waitByteTimes(size - (bytes_available + bytes_read));
				}
			}

			// This should be non-blocking returning only what is available now
			//  Then returning so that select can block again.
			ssize_t bytes_read_now = ::read(fd_, buf + bytes_read, size - bytes_read);

			// read should always return some data as select reported it was
			// ready to read when we get to this point.
			if (bytes_read_now < 1)
			{
				// Disconnected devices, at least on Linux, show the
				// behavior that they are always ready to read immediately
				// but reading returns nothing.
				continue;
			}

			// Update bytes_read
			bytes_read += static_cast<size_t>(bytes_read_now);

			// If bytes_read == size then we have read everything we need
			if (bytes_read == size)
			{
				break;
			}

			// If bytes_read < size then we have more to read
			if (bytes_read < size)
			{
				continue;
			}

			// If bytes_read > size then we have over read, which shouldn't happen
			if (bytes_read > size)
			{
				break;
			}
		}
	}

	return bytes_read;
}

size_t Serial_Port::write_data(const uint8_t *data, size_t length) {
  if (is_open_ == false) {
    return 0;
  }

  Timeout timeout_;
  fd_set writefds;
  size_t bytes_written = 0;

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.write_timeout_constant;
  total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long>
                      (length);
  MillisecondTimer total_timeout(total_timeout_ms);

  bool first_iteration = true;

  while (bytes_written < length) {
    int64_t timeout_remaining_ms = total_timeout.remaining();

    // Only consider the timeout if it's not the first iteration of the loop
    // otherwise a timeout of 0 won't be allowed through
    if (!first_iteration && (timeout_remaining_ms <= 0)) {
      // Timed out
      break;
    }

    first_iteration = false;

    timespec timeout(timespec_from_ms(timeout_remaining_ms));

    FD_ZERO(&writefds);
    FD_SET(fd_, &writefds);

    // Do the select
    int r = pselect(fd_ + 1, NULL, &writefds, NULL, &timeout, NULL);

    // Figure out what happened by looking at select's response 'r'
    /** Error **/
    if (r < 0) {
      // Select was interrupted, try again
      if (errno == EINTR) {
        continue;
      }

      // Otherwise there was some error
      continue;
    }

    /** Timeout **/
    if (r == 0) {
      break;
    }

    /** Port ready to write **/
    if (r > 0) {
      // Make sure our file descriptor is in the ready to write list
      if (FD_ISSET(fd_, &writefds)) {
        // This will write some
        ssize_t bytes_written_now = ::write(fd_, data + bytes_written,
                                            length - bytes_written);

        // write should always return some data as select reported it was
        // ready to write when we get to this point.
        if (bytes_written_now < 1) {
          // Disconnected devices, at least on Linux, show the
          // behavior that they are always ready to write immediately
          // but writing returns nothing.
          continue;
        }

        // Update bytes_written
        bytes_written += static_cast<size_t>(bytes_written_now);

        // If bytes_written == size then we have written everything we need to
        if (bytes_written == length) {
          break;
        }

        // If bytes_written < size then we have more to write
        if (bytes_written < length) {
          continue;
        }

        // If bytes_written > size then we have over written, which shouldn't happen
        if (bytes_written > length) {
          break;
        }
      }

      // This shouldn't happen, if r > 0 our fd has to be in the list!
      break;
      //THROW (IOException, "select reports ready to write, but our fd isn't in the list, this shouldn't happen!");
    }
  }

  return bytes_written;
}


uint32_t Serial_Port::getByteTime() {
  return byte_time_ns_;
}

bool Serial_Port::getTermios(termios *tio) {
  ::memset(tio, 0, sizeof(termios));

  if (::tcgetattr(fd_, tio) == -1) {
    return false;
  }

  return true;
}


void Serial_Port::set_databits(termios *tio, bytesize_t databits) {
  tio->c_cflag &= ~CSIZE;

  switch (databits) {
    case fivebits:
      tio->c_cflag |= CS5;
      break;

    case sixbits:
      tio->c_cflag |= CS6;
      break;

    case sevenbits:
      tio->c_cflag |= CS7;
      break;

    case eightbits:
      tio->c_cflag |= CS8;
      break;

    default:
      tio->c_cflag |= CS8;
      break;
  }
}

void Serial_Port::set_parity(termios *tio,parity_t parity) {
  tio->c_iflag &= ~(PARMRK | INPCK);
  tio->c_iflag |= IGNPAR;

  switch (parity) {

#ifdef CMSPAR

    case parity_space:
      tio->c_cflag &= ~PARODD;
      tio->c_cflag |= PARENB | CMSPAR;
      break;

    case parity_mark:
      tio->c_cflag |= PARENB | CMSPAR | PARODD;
      break;
#endif

    case parity_none:
      tio->c_cflag &= ~PARENB;
      break;

    case parity_even:
      tio->c_cflag &= ~PARODD;
      tio->c_cflag |= PARENB;
      break;

    case parity_odd:
      tio->c_cflag |= PARENB | PARODD;
      break;

    default:
      tio->c_cflag |= PARENB;
      tio->c_iflag |= PARMRK | INPCK;
      tio->c_iflag &= ~IGNPAR;
      break;
  }
}

void Serial_Port::set_stopbits(termios *tio, stopbits_t stopbits) {
  switch (stopbits) {
    case stopbits_one:
      tio->c_cflag &= ~CSTOPB;
      break;

    case stopbits_two:
      tio->c_cflag |= CSTOPB;
      break;

    default:
      tio->c_cflag &= ~CSTOPB;
      break;
  }
}

void Serial_Port::set_flowcontrol(termios *tio,flowcontrol_t flowcontrol) {
  switch (flowcontrol) {
    case flowcontrol_none:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;

    case flowcontrol_hardware:
      tio->c_cflag |= CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;

    case flowcontrol_software:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag |= IXON | IXOFF | IXANY;
      break;

    default:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;
  }
}

bool Serial_Port::setCustomBaudRate(unsigned long baudrate) {
  struct termios2 tio2;

  if (::ioctl(fd_, TCGETS2, &tio2) != -1) {
    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;

    tio2.c_ispeed = baudrate;
    tio2.c_ospeed = baudrate;

    tcflush(fd_, TCIFLUSH);

    if (fcntl(fd_, F_SETFL, FNDELAY)) {
      return false;
    }

    if (::ioctl(fd_, TCSETS2, &tio2) != -1 && ::ioctl(fd_, TCGETS2, &tio2) != -1) {
      return true;
    }
  }
}

bool Serial_Port::setBaudrate(unsigned long baudrate) {

  if (fd_ == -1) {
    return false;
  }
  speed_t new_baud = static_cast<speed_t>(baudrate);

  return setCustomBaudRate(baudrate);
}
bool Serial_Port::setTermios(const termios *tio) {

  tcflush(fd_, TCIFLUSH);

  if (fcntl(fd_, F_SETFL, FNDELAY)) {
    return false;
  }

  if (::tcsetattr(fd_, TCSANOW, tio) == -1) {
    return false;
  }

  return true;
}

void Serial_Port::set_common_props(termios *tio) {
#ifdef OS_SOLARIS
  tio->c_iflag &= ~(IMAXBEL | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                    ICRNL | IXON);
  tio->c_oflag &= ~OPOST;
  tio->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tio->c_cflag &= ~(CSIZE | PARENB);
  tio->c_cflag |= CS8;
#else
  ::cfmakeraw(tio);
#endif
  tio->c_cflag |= CLOCAL | CREAD;
  tio->c_cc[VTIME] = 0;
  tio->c_cc[VMIN] = 0;
}

bool Serial_Port::open() {
  
	if (port_.empty()) {
		return false;
	}

	if (is_open_ == true) {
		return true;
	}
	/*
	pid = -1;
	pid = getpid();
	*/
	fd_ = ::open(port_.c_str(),
			O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND | O_NDELAY);

	if (fd_ == -1) {
		return false;
	}


	termios tio;
	if(!getTermios(&tio)){
		close();
		return false;
	}

	set_common_props(&tio);
	set_databits(&tio, bytesize_);
	set_parity(&tio, parity_);
	set_stopbits(&tio, stopbits_);
	set_flowcontrol(&tio, flowcontrol_);

	if (!setTermios(&tio)) {
    	close();
    	return false;
  	}
	if (!setBaudrate(baudrate_)) {
		return false;
	}

	// Update byte_time_ based on the new settings.
	uint32_t bit_time_ns = 1e9 / baudrate_;
	byte_time_ns_ = bit_time_ns * (1 + eightbits + parity_none + stopbits_one);


	is_open_ = true;
	return true;
}

bool Serial_Port::setDTR(bool level) {
  if (is_open_ == false) {
    return false;
  }
  int command = TIOCM_DTR;
  if (level) {
    if (-1 == ioctl(fd_, TIOCMBIS, &command)) {
      return false;
    }
  } else {
    if (-1 == ioctl(fd_, TIOCMBIC, &command)) {
      return false;
    }
  }
  return true;
}

/*关闭串口*/
void Serial_Port::close() 
{
  if (is_open_ == true) 
  {
    if (fd_ != -1) 
    {
      ::close(fd_);
    }
    fd_ = -1;
    pid = -1;
    is_open_ = false;
  }
}



