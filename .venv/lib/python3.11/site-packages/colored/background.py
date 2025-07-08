#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import annotations

from .exceptions import InvalidColor
from .library import Library
from .utilities import Utilities


class MetaBack(type):
    """Metaclass to customize attribute access."""
    def __getattr__(cls, color: str):
        """Override __getattr__ to raise InvalidColor when an invalid color is accessed."""
        raise InvalidColor(f'InvalidColor: {color}')


class Back(metaclass=MetaBack):

    """ Back class contains predefined background colors for
    the command line (terminal).
    """

    _utils = Utilities()
    _END: str = Library.END
    _COLORS: dict[str, str] = Library.COLORS
    _BACKGROUND_256: str = Library.BACKGROUND_256
    _BACKGROUND_RGB: str = Library.BACKGROUND_RGB

    for _color, _code in _COLORS.items():
        vars()[_color] = f'{_BACKGROUND_256}{_code}{_END}'
        vars()[_color.upper()] = f'{_BACKGROUND_256}{_code}{_END}'

    @classmethod
    def rgb(cls: type[Back], r: int | str, g: int | str, b: int | str) -> str:
        """Combination with text returns color background with text.

        Args:
            r (int | str): Sets the Red color.
            g (int | str): Sets the Green color.
            b (int | str): Sets the Blue color.

        Returns:
            str: Background RGB code.
        """
        r, g, b = cls._utils.is_percentage((r, g, b))
        return f'{cls._BACKGROUND_RGB}{r};{g};{b}{cls._END}'

    @classmethod
    def RGB(cls: type[Back], r: int | str, g: int | str, b: int | str) -> str:  # pylint: disable=[C0103]
        """Combination with text returns color background with text.

        Args:
            r (int | str): Sets the Red color.
            g (int | str): Sets the Green color.
            b (int | str): Sets the Blue color.

        Returns:
            str: Background RGB code.
        """
        return cls.rgb(r, g, b)


class back(Back):  # pylint: disable=[C0103]
    """ This will be deprecated in the future, do not use this for version >= 2.0.0,
        instead please use Back class (See issue #28). """
