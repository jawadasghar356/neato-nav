#include "neato/serial_port.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#endif

namespace neato {

SerialPort::~SerialPort() { close(); }

#ifdef _WIN32  // ---------------------------------------------------------- Windows

bool SerialPort::open(const std::string& name, int baud) {
    close();
    const std::string path = "\\\\.\\" + name;  // needed for COM10 and above
    HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                           OPEN_EXISTING, 0, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        last_error_ = "Cannot open " + name + " (is PuTTY still connected?)";
        return false;
    }
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = static_cast<DWORD>(baud);  // USB CDC ignores it, but set it anyway
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!SetCommState(h, &dcb)) {
        CloseHandle(h);
        last_error_ = "SetCommState failed";
        return false;
    }
    handle_ = h;
    return true;
}

void SerialPort::close() {
    if (handle_) {
        CloseHandle(static_cast<HANDLE>(handle_));
        handle_ = nullptr;
    }
}

bool SerialPort::isOpen() const { return handle_ != nullptr; }

bool SerialPort::write(const std::string& data) {
    if (!handle_) return false;
    DWORD written = 0;
    BOOL ok = WriteFile(static_cast<HANDLE>(handle_), data.data(),
                        static_cast<DWORD>(data.size()), &written, nullptr);
    return ok && written == data.size();
}

int SerialPort::readSome(char* buffer, int max_len, int timeout_ms) {
    if (!handle_) return -1;
    // Return as soon as at least one byte is there, or after timeout_ms
    COMMTIMEOUTS to{};
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = static_cast<DWORD>(timeout_ms);
    SetCommTimeouts(static_cast<HANDLE>(handle_), &to);
    DWORD got = 0;
    if (!ReadFile(static_cast<HANDLE>(handle_), buffer, static_cast<DWORD>(max_len), &got, nullptr))
        return -1;
    return static_cast<int>(got);
}

void SerialPort::flushInput() {
    if (handle_) PurgeComm(static_cast<HANDLE>(handle_), PURGE_RXCLEAR);
}

#else  // ------------------------------------------------------------- Linux / macOS

bool SerialPort::open(const std::string& name, int /*baud*/) {
    close();
    fd_ = ::open(name.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        last_error_ = "Cannot open " + name + ": " + std::strerror(errno);
        return false;
    }
    termios tio{};
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);  // no line editing, no echo, 8N1
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd_, TCSANOW, &tio);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const { return fd_ >= 0; }

bool SerialPort::write(const std::string& data) {
    if (fd_ < 0) return false;
    return ::write(fd_, data.data(), data.size()) == static_cast<ssize_t>(data.size());
}

int SerialPort::readSome(char* buffer, int max_len, int timeout_ms) {
    if (fd_ < 0) return -1;
    pollfd p{fd_, POLLIN, 0};
    const int r = ::poll(&p, 1, timeout_ms);
    if (r <= 0) return r;  // 0 = timeout, -1 = error
    return static_cast<int>(::read(fd_, buffer, static_cast<size_t>(max_len)));
}

void SerialPort::flushInput() {
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

#endif

}  // namespace neato
