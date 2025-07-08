#!/usr/bin/env python
# -*- coding: utf-8 -*-

from typing import Union

from .colored import fore
from .library import Library


class Convert:

    """Using colored libraries to match HEX code to ANSI code and opposite.
    """

    def hex_to_ansi(self, hex_code: str) -> Union[str, None]:
        """Convert HEX code to ANSI code string.

        Args:
            hex_code (str): HEX code string.

        Returns:
            Union[str, None]: ANSI code number.
        """
        if hex_code in Library.HEX_COLORS.values():
            key: str = self._get_key_from_value(hex_code)
            return fore(key)
        return None

    def ansi_to_hex(self, ansi_code: int) -> Union[str, None]:
        """Convert ANSI code to HEX code string.

        Args:
            ansi_code (str): ANSI number code.

        Returns:
            Union[str, None]: HEX code string.
        """
        if str(ansi_code) in Library.HEX_COLORS:
            return Library.HEX_COLORS[str(ansi_code)]
        return None

    def _get_key_from_value(self, value: str) -> str:
        """Return the key of the dictionary with value.

        Args:
            value (str): Dictionary value.

        Returns:
            (str): The key of the dictionary.
        """
        for key, val in Library.HEX_COLORS.items():
            if val == value:
                return key
        return ''
