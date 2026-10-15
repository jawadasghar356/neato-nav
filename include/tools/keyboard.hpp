// Non-blocking single-key input from the terminal.
// POSIX: switches the terminal to non-canonical mode without echo and restores it on destruction.
// Ctrl+C still raises SIGINT. Windows: uses the console functions from <conio.h>.
#pragma once

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace tools {

class RawKeyboard {
public:
    RawKeyboard() {
#ifndef _WIN32
        if (tcgetattr(STDIN_FILENO, &saved_) == 0) {
            termios raw = saved_;
            raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;   // read() returns immediately
            raw.c_cc[VTIME] = 0;
            active_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
        }
#endif
    }

    ~RawKeyboard() {
#ifndef _WIN32
        if (active_) tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
#endif
    }

    RawKeyboard(const RawKeyboard&) = delete;
    RawKeyboard& operator=(const RawKeyboard&) = delete;

    // Returns the next pressed key, or -1 if none is waiting.
    int poll() {
#ifdef _WIN32
        return _kbhit() ? _getch() : -1;
#else
        unsigned char c = 0;
        return read(STDIN_FILENO, &c, 1) == 1 ? c : -1;
#endif
    }

private:
#ifndef _WIN32
    termios saved_{};
    bool active_ = false;
#endif
};

}  // namespace tools
