# Utility functions for SO101 module
import sys
import termios
import tty


def enter_pressed():
    """Check if Enter key was pressed (non-blocking)."""
    if sys.stdin.isatty():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            if sys.stdin.read(1) == '\n':
                return True
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return False


def move_cursor_up(n_lines: int):
    """Move terminal cursor up n lines."""
    for _ in range(n_lines):
        sys.stdout.write('\033[1A')  # Move up one line
        sys.stdout.write('\033[K')   # Clear line
    sys.stdout.flush()


class DeviceNotConnectedError(ConnectionError):
    """Exception raised when the device is not connected."""
    pass


class DeviceAlreadyConnectedError(ConnectionError):
    """Exception raised when the device is already connected."""
    pass

