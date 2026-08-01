#include "UARTDriver.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>

namespace {

speed_t baud_to_termios(uint32_t baud)
{
    switch (baud) {
    case 9600U:
        return B9600;
    case 38400U:
        return B38400;
    case 57600U:
        return B57600;
    case 115200U:
        return B115200;
#ifdef B230400
    case 230400U:
        return B230400;
#endif
#ifdef B460800
    case 460800U:
        return B460800;
#endif
#ifdef B921600
    case 921600U:
        return B921600;
#endif
    default:
        return 0;
    }
}

} // namespace

Spresense::UARTDriver::UARTDriver(const char *device_path) :
    _device_path(device_path),
    _baud(0U)
{
}

bool Spresense::UARTDriver::is_initialized()
{
    return _device.is_open();
}

bool Spresense::UARTDriver::tx_pending()
{
    return false;
}

uint32_t Spresense::UARTDriver::txspace()
{
    return _device.is_open() ? 64U : 0U;
}

uint32_t Spresense::UARTDriver::get_baud_rate() const
{
    return _baud;
}

void Spresense::UARTDriver::_begin(uint32_t baud, uint16_t rx_space, uint16_t tx_space)
{
    (void)rx_space;
    (void)tx_space;
    if (_device.is_open()) {
        _end();
    }
    if (!_device.open_device(_device_path, O_RDWR | O_NONBLOCK)) {
        return;
    }
    if (!configure(baud)) {
        _device.close();
        return;
    }
    _baud = baud;
}

bool Spresense::UARTDriver::configure(uint32_t baud)
{
    const speed_t speed = baud_to_termios(baud);
    if (speed == 0) {
        return false;
    }

    struct termios config {};
    if (tcgetattr(_device.native_fd(), &config) != 0) {
        return false;
    }
    config.c_iflag = 0;
    config.c_oflag = 0;
    config.c_lflag = 0;
    config.c_cflag = CS8 | CREAD | CLOCAL;
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 0;
    if (cfsetispeed(&config, speed) != 0 || cfsetospeed(&config, speed) != 0) {
        return false;
    }
    return tcsetattr(_device.native_fd(), TCSANOW, &config) == 0;
}

size_t Spresense::UARTDriver::_write(const uint8_t *buffer, size_t size)
{
    return _device.write_all(buffer, size) ? size : 0U;
}

ssize_t Spresense::UARTDriver::_read(uint8_t *buffer, uint16_t size)
{
    return _device.read_some(buffer, size);
}

void Spresense::UARTDriver::_end()
{
    _device.close();
    _baud = 0U;
}

void Spresense::UARTDriver::_flush()
{
    if (_device.is_open()) {
        (void)tcdrain(_device.native_fd());
    }
}

uint32_t Spresense::UARTDriver::_available()
{
    if (!_device.is_open()) {
        return 0U;
    }
    int available = 0;
    if (ioctl(_device.native_fd(), FIONREAD, &available) != 0 || available < 0) {
        return 0U;
    }
    return static_cast<uint32_t>(available);
}

bool Spresense::UARTDriver::_discard_input()
{
    return _device.is_open() && tcflush(_device.native_fd(), TCIFLUSH) == 0;
}
