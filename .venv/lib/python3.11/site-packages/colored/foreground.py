#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import annotations

from .exceptions import InvalidColor
from .library import Library
from .utilities import Utilities


class MetaFore(type):
    """Metaclass to customize attribute access."""
    def __getattr__(cls, color: str):
        """Override __getattr__ to raise InvalidColor when an invalid color is accessed."""
        raise InvalidColor(f"InvalidColor: {color}")


class Fore(metaclass=MetaFore):

    """ Fore class contains predefined foreground colors for
    the command line (terminal).
    """

    _utils = Utilities()
    _END: str = Library.END
    _COLORS: dict[str, str] = Library.COLORS
    _FOREGROUND_256: str = Library.FOREGROUND_256
    _FOREGROUND_RGB: str = Library.FOREGROUND_RGB

    for _color, _code in _COLORS.items():
        vars()[_color] = f'{_FOREGROUND_256}{_code}{_END}'
        vars()[_color.upper()] = f'{_FOREGROUND_256}{_code}{_END}'

    @classmethod
    def rgb(cls: type[Fore], r: int | str, g: int | str, b: int | str) -> str:
        """ Combination with text returns color text.

        Args:
            r: Sets the Red color.
            g: Sets the Green color.
            b: Sets the Blue color.

        Returns:
            str: Foreground RGB code.
        """
        r, g, b = cls._utils.is_percentage((r, g, b))
        return f'{cls._FOREGROUND_RGB}{r};{g};{b}{cls._END}'

    @classmethod
    def RGB(cls: type[Fore], r: int | str, g: int | str, b: int | str) -> str:  # pylint: disable=[C0103]
        """ Combination with text returns color text.

        Args:
            r: Sets the Red color.
            g: Sets the Green color.
            b: Sets the Blue color.

        Returns:
            str: Foreground RGB code.
        """
        return cls.rgb(r, g, b)


class fore(Fore):  # pylint: disable=[C0103]
    """ This will be deprecated in the future, do not use this for version >= 2.0.0,
        instead please use Fore class (See issue #28). """
