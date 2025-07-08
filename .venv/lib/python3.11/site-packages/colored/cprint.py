#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import Any, Optional

from .colored import Colored
from .colored import back_rgb as background_rgb
from .colored import fore_rgb as foreground_rgb


def cprint(text: str,  # pylint: disable=[R0913,R0917]
           fore_256: Optional[int | str] = None,
           back_256: Optional[int | str] = None,
           fore_rgb: Optional[tuple] = None,
           back_rgb: Optional[tuple] = None,
           formatting: int | str = '',
           line_color: int | str = '',
           reset: bool = True,
           **kwargs: Any) -> None:  # noqa: ANN401
    """Looks like a patch to a built-in python print() function that allows
    to pass colored text and style, to this function.

    Args:
        text (str): String type text.
        fore_256 (Optional[int | str], optional): Sets the foreground color.
        back_256 (Optional[int | str], optional): Sets the background color.
        fore_rgb (Optional[tuple], optional): Sets the foreground color.
        back_rgb (Optional[tuple], optional): Sets the background color.
        formatting (int | str, optional): Sets the style of the text.
        line_color (int | str, optional): Sets the underline color.
        reset (bool, optional): Reset the formatting style after print, default is True.
        **kwargs (dict[str, Any]): Description
    """
    styling = Colored(formatting).attribute(line_color)
    terminator = Colored('reset').attribute() if reset else ''

    fore_color = resolve_foreground(fore_256, fore_rgb)
    back_color = resolve_background(back_256, back_rgb)

    print(f'{styling}{back_color}{fore_color}{text}{terminator}', **kwargs)


def resolve_foreground(fore_256: Optional[int | str], fore_rgb: Optional[tuple]) -> str:
    """Returns the correct foreground color string.

    Args:
        fore_256 (Optional[int | str]): Description
        fore_rgb (Optional[tuple]): Description

    Returns:
        str: Foreground color code.
    """
    fore_color: str = ''
    if fore_256:
        fore_color = Colored(fore_256).foreground()
    elif fore_rgb:
        fr, fg, fb = fore_rgb
        fore_color = foreground_rgb(fr, fg, fb)

    return fore_color


def resolve_background(back_256: Optional[int | str], back_rgb: Optional[tuple]) -> str:
    """Returns the correct background color string.

    Args:
        back_256 (Optional[int | str]): Description
        back_rgb (Optional[tuple]): Description

    Returns:
        str: Background color code.
    """
    back_color: str = ''
    if back_256:
        back_color = Colored(back_256).background()
    elif back_rgb:
        br, bg, bb = back_rgb
        back_color = background_rgb(br, bg, bb)

    return back_color
