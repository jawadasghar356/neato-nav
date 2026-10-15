// Minimal cross-platform serial port (Windows COMx and Linux /dev/ttyACMx).
// No external libraries: Win32 API on Windows, termios on Linux/macOS.
#pragma once
#include <string>

namespace neato {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // name: "COM9" on Windows, "/dev/ttyACM0" on Linux
    bool open(const std::string& name, int baud = 115200);
    void close();
    bool isOpen() const;

    bool write(const std::string& data);

    // Read whatever is available, waiting at most timeout_ms for the first byte.
    // Returns the number of bytes read (0 on timeout, -1 on error).
    int readSome(char* buffer, int max_len, int timeout_ms);

    // Discard anything the robot sent that we have not read yet
    void flushInput();

    const std::string& lastError() const { return last_error_; }

private:
#ifdef _WIN32
    void* handle_{nullptr};  // HANDLE, kept as void* so <windows.h> stays out of headers
#else
    int fd_{-1};
#endif
    std::string last_error_;
};

}  // namespace neato
