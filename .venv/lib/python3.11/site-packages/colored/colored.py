#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import annotations

import os
import platform
import sys
from typing import Any, Optional

from .hexadecimal import Hex
from .library import Library
from .utilities import Utilities

# TTY_AWARE = True
# IS_TTY = sys.stdout.isatty() and sys.stderr.isatty()
# _win_vterm_mode = None

# pylint: disable=invalid-name


class Config:  # pylint: disable=[R0903]

    """ Config some global variables.

    Attributes:
        TTY_AWARE (bool): Description
    """
    TTY_AWARE: bool = True
    WIN_VTERM_MODE: Optional[bool] = None

    @staticmethod
    def is_tty() -> bool:
        """ Check if both sys.stdout and sys.stderr are connected to a TTY (terminal).

        This function determines whether the script is running in an interactive
        terminal or if the output has been redirected (e.g., to a file or pipe).

        Returns:
            bool: True if both stdout and stderr are attached to a TTY, False otherwise.

        Example:
            >>> is_tty()
            True  # When running in a terminal

            >>> python script.py > output.txt
            False  # When stdout is redirected to a file
        """
        return sys.stdout.isatty() and sys.stderr.isatty()


class Colored:  # pylint: disable=[R0902]

    """ Colored class, it's the main class of managing
    foreground, background and style colors.
    """

    def __init__(self, name: str | int):
        """name can be str or int instead.

        Args:
            name (Union[str, int]): name of color or number of color
        """
        self._name: str = str(name).lower()
        self._hex_color: str = ''
        self._hex = Hex()
        self._utils = Utilities()

        self._ESC: str = Library.ESC
        self._END: str = Library.END

        self._STYLES: dict[str, str] = Library.STYLES
        self._FOREGROUND_256: str = Library.FOREGROUND_256
        self._BACKGROUND_256: str = Library.BACKGROUND_256

        self._COLORS: dict[str, str] = Library.COLORS
        self._HEX_COLORS: dict[str, str] = Library.HEX_COLORS
        self._UNDERLINE_COLOR: str = Library.UNDERLINE_COLOR

        if self._name.startswith('#'):
            self._hex_color = self._hex.find(self._name)

        self.enable_windows_terminal_mode()

    def attribute(self, line_color: str | int = '') -> str:
        """ Returns stylize text.

        Args:
            line_color: Sets color of the underline.

        Returns:
            str: Style code.
        """
        formatting: str = self._name
        if not self.enabled():
            return ''

        if self._name:
            self._utils.is_style_exist(self._name)

            if self._name in ('underline', '4') and line_color:
                line_color = str(line_color).lower()
                self._utils.is_color_exist(line_color)
                if not line_color.isdigit():
                    line_color = self._COLORS[line_color]
                return f'{self._UNDERLINE_COLOR}{line_color}{self._END}'

            if not self._name.isdigit():
                formatting = self._STYLES[self._name]

        return f'{self._ESC}{formatting}{self._END}'

    def foreground(self) -> str:
        """ Returns a foreground 256 color code. """
        color: str = self._name
        if not self.enabled():
            return ''

        if self._name:
            self._utils.is_color_exist(self._name)

            if self._name.startswith('#'):
                color = self._hex_color
            elif not self._name.isdigit():
                color = self._COLORS[self._name]

        return f'{self._FOREGROUND_256}{color}{self._END}'

    def background(self) -> str:
        """ Returns a background 256 color code. """
        color: str = self._name
        if not self.enabled():
            return ''

        if self._name:
            self._utils.is_color_exist(self._name)

            if self._name.startswith('#'):
                color = self._hex_color
            elif not self._name.isdigit():
                color = self._COLORS[self._name]

        return f'{self._BACKGROUND_256}{color}{self._END}'

    @staticmethod
    def enable_windows_terminal_mode() -> Any:
        """Contribution by: Andreas Fredrik Klasson, Magnus Heskestad,
        Dimitris Papadopoulos.

        Enable virtual terminal processing in Windows terminal. Does
        nothing if not on Windows. This is based on the rejected
        enhancement <https://bugs.python.org/issue29059>.

        Returns:
            Any: Description
        """
        # global _win_vterm_mode
        if Config.WIN_VTERM_MODE is not None:
            return Config.WIN_VTERM_MODE

        # Note: Cygwin should return something like 'CYGWIN_NT...'
        Config.WIN_VTERM_MODE = platform.system().lower() == 'windows'
        if Config.WIN_VTERM_MODE is False:
            return

        from ctypes import windll, wintypes, byref, c_void_p
        ENABLE_VIRTUAL_TERMINAL_PROCESSING = 0x0004
        INVALID_HANDLE_VALUE = c_void_p(-1).value
        STD_OUTPUT_HANDLE = wintypes.DWORD(-11)

        hStdout = windll.kernel32.GetStdHandle(STD_OUTPUT_HANDLE)
        if hStdout == INVALID_HANDLE_VALUE:
            Config.WIN_VTERM_MODE = False
            return

        mode = wintypes.DWORD(0)
        ok = windll.kernel32.GetConsoleMode(wintypes.HANDLE(hStdout), byref(mode))
        if not ok:
            Config.WIN_VTERM_MODE = False
            return

        mode = wintypes.DWORD(mode.value | ENABLE_VIRTUAL_TERMINAL_PROCESSING)
        ok = windll.kernel32.SetConsoleMode(wintypes.HANDLE(hStdout), mode)
        if not ok:
            # Something went wrong, probably a version too old
            # to support the VT100 mode.
            # To be more certain we could check kernel32.GetLastError
            # for STATUS_INVALID_PARAMETER, but since we only enable
            # one flag we can be certain enough.
            Config.WIN_VTERM_MODE = False
            return

    @staticmethod
    def enabled() -> bool:
        """ Contribution by Andreas Motl.
        https://github.com/chalk/supports-color#info
        Use the environment variable FORCE_COLOR=1 (level 1), FORCE_COLOR=2
        (level 2), or FORCE_COLOR=3 (level 3) to forcefully enable color, or
        FORCE_COLOR=0 to forcefully disable. The use of FORCE_COLOR overrides
        all other color support checks.
        """
        if 'FORCE_COLOR' in os.environ:
            if int(os.environ['FORCE_COLOR']) == 0:
                return False
            return True

        # https://no-color.org/
        # Check for the presence of a NO_COLOR environment variable that, when
        # present (regardless of its value), prevents the addition of ANSI
        # color.
        if 'NO_COLOR' in os.environ:
            return False

        # Also disable coloring when not printing to a TTY.
        if Config.TTY_AWARE and not Config.is_tty():
            return False

        # In all other cases, enable coloring.
        return True


def style(name: int | str, color: str | int = '') -> str:
    """ Alias for Colored(name).attribute()

    Args:
        name: Sets the name of the color.
        color: Sets the underline color.

    Returns:
        str: Style code.
    """
    return Colored(name).attribute(color)


def fore(name: int | str) -> str:
    """ Combination with text returns color text.

    Args:
        name: Sets the name of the color.

    Returns:
        str: Foreground code.
    """
    return Colored(name).foreground()


def back(name: int | str) -> str:
    """ Combination with text returns color background with text.

    Args:
        name: Sets the name of the color.

    Returns:
        str: Background code.
    """
    return Colored(name).background()


def fore_rgb(r: int | str, g: int | str, b: int | str) -> str:
    """ Combination with text returns color text.

    Args:
        r: Red color.
        g: Green color.
        b: Blue color.

    Returns:
        str: Foreground RGB code.
    """
    utils = Utilities()
    r, g, b = utils.is_percentage((r, g, b))
    return f'{Library.FOREGROUND_RGB}{r};{g};{b}{Library.END}'


def back_rgb(r: int | str, g: int | str, b: int | str) -> str:
    """ Combination with text returns color background with text.

    Args:
        r: Red color.
        g: Green color.
        b: Blue color.

    Returns:
        str: Background RGB code.
    """
    utils = Utilities()
    r, g, b = utils.is_percentage((r, g, b))
    return f'{Library.BACKGROUND_RGB}{r};{g};{b}{Library.END}'


def attr(name: int | str) -> str:
    """ This will be deprecated in the future, do not use with version >= 2.0.0,
    instead please use style() function (See issue #28).

    Args:
        name: Sets the name of the color.

    Returns:
        str: Style code.
    """
    return Colored(name).attribute()


def fg(name: int | str) -> str:
    """ This will be deprecated in the future, do not use with version >= 2.0.0,
    instead please use style() function (See issue #28).

    Args:
        name: Sets the name of the color.

    Returns:
        str: Foreground code.
    """
    return Colored(name).foreground()


def bg(name: int | str) -> str:
    """ This will be deprecated in the future, do not use with version >= 2.0.0,
    instead please use style() function (See issue #28).

    Args:
        name: Sets the name of the color.

    Returns:
        str: Background code.
    """
    return Colored(name).background()


def stylize(text: str, formatting: int | str, reset: bool = True) -> str:
    """ Conveniently styles your text as and resets ANSI codes at its end.

    Args:
        text: String type text.
        formatting: Sets the formatting (color or style) of the text.
        reset: Reset the formatting style at its end, default is True.

    Returns:
        str: Formatting string text.
    """
    terminator: str = style('reset') if reset else ''
    return f'{"".join(str(formatting))}{text}{terminator}'


def _c0wrap(formatting: str) -> str:
    """Contribution by brrzap.
    Wrap a set of ANSI styles in C0 control codes for readline safety.

    Args:
        formatting (str): Sets the formatting (color or style) of the text.

    Returns:
        str: Code formatting.
    """
    C0_SOH: str = '\x01'  # mark the beginning of nonprinting characters
    C0_STX: str = '\x02'  # mark the end of nonprinting characters
    return f'{C0_SOH}{"".join(formatting)}{C0_STX}'


def stylize_interactive(text: str, formatting: str, reset: bool = True) -> str:
    """Contribution by: Jay Deiman, brrzap
    stylize() variant that adds C0 control codes (SOH/STX) for readline
    safety.

    Args:
        text (str): String type text.
        formatting (str): Sets the formatting (color or style) of the text.
        reset (bool, optional): Reset the formatting style at its end, default is True.

    Returns:
        str: Formatting string text.
    """
    # problem: readline includes bare ANSI codes in width calculations.
    # solution: wrap nonprinting codes in SOH/STX when necessary.
    # see: https://gitlab.com/dslackw/colored/issues/5
    terminator: str = _c0wrap(style('reset')) if reset else ''
    return f'{_c0wrap(formatting)}{text}{terminator}'


def set_tty_aware(awareness: bool = True) -> None:
    """ Contribution by: Andreas Motl, Jay Deiman

    Makes all interactions here tty aware. This means that if either
    stdout or stderr are directed to something other than a tty,
    colorization will not be added.

    Args:
        awareness: Default is True.
    """
    # global TTY_AWARE
    # TTY_AWARE = awareness
    Config.TTY_AWARE = awareness
