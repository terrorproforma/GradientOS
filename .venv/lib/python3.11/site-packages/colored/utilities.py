#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import annotations

import os

from .exceptions import (InvalidColor, InvalidControl, InvalidHexColor,
                         InvalidStyle)
from .library import Library

# pylint: disable=invalid-name


class Utilities:

    """ Class Utilities that managing some behaviors.
    """

    def __init__(self) -> None:
        self._COLORS: dict[str, str] = Library.COLORS
        self._HEX_COLORS: dict[str, str] = Library.HEX_COLORS
        self._STYLES: dict[str, str] = Library.STYLES
        self._CONTROLS: dict[str, str] = Library.CONTROLS
        self._COLORTERM: dict[str, int] = Library.COLORTERM
        self.RGB_MAXIMUM_COLOR: int = 255
        self.colorterm: str = 'truecolor'
        self.set_colorterm()

    def is_color_exist(self, name: str) -> bool:
        """Checks for valid color by name or by hex style name or number,
            and raise a InvalidColor or InvalidHexColor exception if it doesn't exist.

        Args:
            name (str): Sets the name of the style or the number.

        Returns:
            bool: True if existed.

        Raises:
            InvalidColor: Invalid color message.
        """
        if name.startswith('#'):
            if name not in self._HEX_COLORS.values():
                raise InvalidHexColor(f'InvalidHexColor: {name}')

        elif name not in self._COLORS and name not in self._COLORS.values():
            raise InvalidColor(f'InvalidColor: {name}')

        return True

    def is_style_exist(self, name: str) -> bool:
        """Checks for valid style and raise a InvalidStyle exception
            if it doesn't exist.

        Args:
            name (str): Sets the name of the style or the number.

        Returns:
            bool: True if exists.

        Raises:
            InvalidStyle: Invalid style message.
        """
        if name not in self._STYLES and name not in self._STYLES.values():
            raise InvalidStyle(f'InvalidStyle: {name}')

        return True

    def is_control_exist(self, name: str) -> bool:
        """Checks for valid control and raise a InvalidControl exception
            if it doesn't exist.

        Args:
            name (str): Sets the name of the control.

        Returns:
            bool: True if exists.

        Raises:
            InvalidControl: Invalid control message.
        """
        if name.lower() not in self._CONTROLS:
            raise InvalidControl(f'InvalidControl: {name}')

        return True

    def convert_percentages(self, percent: str) -> int:
        """Convert percentages to color number with range 0-255:

        Args:
            percent (str | int): Sets the percent number.

        Returns:
            int | str
        """
        try:  # Sets maximum range for RGB colors.
            self.RGB_MAXIMUM_COLOR = self._COLORTERM[self.colorterm]
        except KeyError:
            pass

        percentages: dict[str, int] = {}
        for number in range(self.RGB_MAXIMUM_COLOR + 1):
            percentage = int((number / self.RGB_MAXIMUM_COLOR) * 100)
            percentages[f'{percentage}%'] = number
        try:
            color = percentages[percent]
        except KeyError:
            color = 0

        return color

    def set_colorterm(self, colorterm: str = '') -> None:
        """Set the  environment variable if it's necessary.
         used to indicate whether a terminal emulator
        could use colors, or whether it supports 8-bit color or more.

        Args:
            colorterm (str, optional): Sets the  environment variable.
        """
        if not colorterm:
            self.colorterm = os.getenv('COLORTERM') or ''
        else:
            self.colorterm = colorterm
            os.environ['COLORTERM'] = colorterm

    def is_percentage(self, numbers: tuple[int | str, int | str, int | str]) -> list:
        """Checks a tuple of RGB numbers and convert them to integers.

        Args:
            numbers (tuple): Sets percentages of numbers.

        Returns:
            list: List with RGB numbers.
        """
        rgb_list: list[str | int] = []
        for num in numbers:
            if isinstance(num, str) and num.endswith('%'):
                rgb_list.append(self.convert_percentages(str(num)))
            else:
                rgb_list.append(num)
        return rgb_list
