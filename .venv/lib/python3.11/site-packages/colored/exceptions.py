#!/usr/bin/env python
# -*- coding: utf-8 -*-


class InvalidColor(Exception):
    """Custom Exception for invalid colors."""
    def __init__(self, message: str):
        super().__init__(message)


class InvalidHexColor(Exception):
    """ Custom Exception for invalid hex colors. """
    def __init__(self, message: str):
        super().__init__(message)


class InvalidStyle(Exception):
    """ Custom Exception for invalid style. """
    def __init__(self, message: str):
        super().__init__(message)


class InvalidControl(Exception):
    """ Custom Exception for invalid navigation. """
    def __init__(self, message: str):
        super().__init__(message)
