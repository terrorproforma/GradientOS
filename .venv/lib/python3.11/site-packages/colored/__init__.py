#!/usr/bin/env python
# -*- coding: utf-8 -*-

from .attributes import Style
from .background import Back
from .colored import (Colored, attr, back, back_rgb, bg, fg, fore, fore_rgb,
                      set_tty_aware, style, stylize, stylize_interactive)
from .controls import Controls
from .convert import Convert
from .cprint import cprint
from .foreground import Fore

__version__: str = '2.3.0'

__all__ = [
    'Colored', 'fore', 'back', 'style', 'fore_rgb', 'back_rgb',
    'fg', 'bg', 'attr', 'stylize', 'stylize_interactive', 'set_tty_aware',
    'cprint', 'Fore', 'Back', 'Style', 'Controls', 'Convert', '__version__'
]
