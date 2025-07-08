#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 18 21:43:18 2020

@author: corkep
"""
import sys

try:
    from colored import fore, back, style

    _colored = True
    # print('using colored output')
except ImportError:
    # print('colored not found')
    _colored = False

# _colored use color ANSI escape sequences via colored package
# _unicode use box characters for table edges and separators
#   from ansitable.table import _unicode
#    _unicode = False

_unicode = True


def options(unicode, color=None):
    """
    Control ANSI/Unicode generation

    :param unicode: enable generation of Unicode characters
    :type unicode: bool
    :param color: enable generation of ANSI color control sequences, defaults to None
    :type color: bool, optional

    ANSItable by default uses Unicode characters to create nice table outlines
    and the colored package to allow colored text and fields.  For some
    applications it is useful to turn this off globally, rather than on a
    table by table basis.

    Unicode and ANSI color can be controlled individually.  If only one parameter
    is given then:

    * ``ansitable.options(True)`` enable Unicode, and ANSI characters if colored
      package is installed.
    * ``ansitable.options(False)`` disable Unicode, and ANSI characters

    If ``unicode=False`` and a border is specified it is set to ``"ascii"``.

    """
    global _colored, _unicode

    _unicode = unicode
    if color is None:
        color = False

    _colored = color


# ------------------------------------------------------------------------- #


class Cell:

    def __init__(self, text, fgcolor=None, bgcolor=None, style=None):
        """Override the color and style of a cell

        :param text: cell text
        :type text: str
        :param fgcolor: foreground color, defaults to None
        :type fgcolor: str, optional
        :param bgcolor: background color, defaults to None
        :type bgcolor: str, optional
        :param style: text style, defaults to None
        :type style: str, optional

        This class is used to override the color and style of a cell in a table, for example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row(Cell("ccccccc", bgcolor="red"), 8.8, -9)
            table.print()

        Will print a table with the first cell in the last row having a red background.  The colors and style override those specified when the column was created
        or specified for a row.
        """
        self.text = text
        self.fgcolor = fgcolor
        self.bgcolor = bgcolor
        self.style = style
        self.column = None
        self.row = None

    def __str__(self):
        return self.text


class Column:
    def __init__(
        self,
        name,
        fmt="{}",
        width=None,
        colcolor=None,
        colbgcolor=None,
        colstyle=None,
        colalign=">",
        headcolor=None,
        headbgcolor=None,
        headstyle=None,
        headalign=">",
    ):
        """
        Create a table column

        :param name: Name of column, also the column heading
        :type name: str
        :param fmt: Python format string, defaults to "{}"
        :type fmt: str, optional
        :param width: Column width, defaults to auto-fit
        :type width: int, optional
        :param colcolor: Color of column text, defaults to None
        :type colcolor: str, optional
        :param colbgcolor: Color of column background, defaults to None
        :type colbgcolor: str, optional
        :param colstyle: Column text style, see table below, defaults to None
        :type colstyle: str, optional
        :param colalign: Column data alignement, see table below, defaults to ">"
        :type colalign: str, optional
        :param headcolor: Color of heading text, defaults to None
        :type headcolor: str, optional
        :param headbgcolor: Color of heading background, defaults to None
        :type headbgcolor: str, optional
        :param headstyle: Heading text style, see table below, defaults to None
        :type headstyle: str, optional
        :param headalign: Heading text alignement, see table below, defaults to ">"
        :type headalign: str, optional

        ===========   =======================
        Alignment     Description
        ===========   =======================
        "<"           Left
        "^"           Centre
        ">"           Right (default)
        ===========   =======================


        ===========   =============================================
        Style         Description
        ===========   =============================================
        bold          bold font
        dim           low brightness font
        underlined    text is underlined
        blink         text is blinking
        reverse       text and background colors are reversed
        ===========   =============================================

        Implementation of these options depends heavily on the terminal emulator
        used.

        """

        self.name = name
        self.fmt = fmt
        self.formatted = []
        self.fgcolor = []
        self.bgcolor = []
        self.style = []
        self.table = None

        if headalign is None:
            self.headalign = colalign
        self.colcolor = colcolor
        self.colbgcolor = colbgcolor
        self.colstyle = colstyle
        self.colalign = colalign

        self.headcolor = headcolor
        self.headbgcolor = headbgcolor
        self.headstyle = headstyle
        self.headalign = headalign

        self.width = width
        self.maxwidth = len(name)

    def _setstyle(self, header):
        if header:
            color = self.headcolor
            bgcolor = self.headbgcolor
            style = self.headstyle
        else:
            color = self.colcolor
            bgcolor = self.colbgcolor
            style = self.colstyle

        text = ""
        if color is not None:
            text += self.table._FG(color)
        if bgcolor is not None:
            text += self.table._BG(bgcolor)
        if style is not None:
            text += self.table._ATTR(styledict[style])
        return text

    def _formatcolumn(
        self,
        text,
        header,
        plain=False,
        fgcolor=None,
        bgcolor=None,
        style=None,
    ):
        """
        Format text in a column

        :param text: text to be aligned
        :type text: str
        :param header: header row of column
        :type header: bool
        :return: aligned string
        :rtype: str
        """
        if header:
            fgcolor = fgcolor or self.headcolor
            bgcolor = bgcolor or self.headbgcolor
            style = style or self.headstyle
            align = self.headalign
        else:
            fgcolor = fgcolor or self.colcolor
            bgcolor = bgcolor or self.colbgcolor
            style = style or self.colstyle
            align = self.colalign

        if plain:
            # used for LaTeX and markdown tables
            fgcolor = None
            bgcolor = None

        gap = self.width - len(text)  # amount of padding required
        if align == "<":
            # left justified, spaces after text
            gap1 = _spaces(self.table.colsep)
            gap2 = _spaces(gap + self.table.colsep)
        elif align == ">":
            # right justified, spaces before text
            gap1 = _spaces(gap + self.table.colsep)
            gap2 = _spaces(self.table.colsep)
        elif align == "^":
            # centred, split the gap
            g1 = gap // 2  # left side gap
            g2 = gap - g1  # right side gap
            gap1 = _spaces(g1 + self.table.colsep)
            gap2 = _spaces(g2 + self.table.colsep)

        if fgcolor:
            text = self.table._FG(fgcolor) + text + self.table._ATTR(0)

        if style == "underlined":
            text = self.table._ATTR(styledict[style]) + text + self.table._ATTR(0)

        text = gap1 + text + gap2

        if bgcolor:
            text = self.table._BG(bgcolor) + text + self.table._ATTR(0)

        if style and style != "underlined":
            text = self.table._ATTR(styledict[style]) + text + self.table._ATTR(0)
        return text


def _spaces(n):
    """
    return n spaces

    :param n: number of spaes
    :type n: int
    :return: string containing spaces
    :rtype: str
    """
    return " " * n


"""
gap  FG text OFF gap
BG gap FG text BG gap OFF
STYLE gap FG text gap OFF
STYLE BG gap FG text BG gap OFF
"""

# unicode box drawing characters, see https://en.wikipedia.org/wiki/Box-drawing_character
#       ascii, thin, thin+round, thick, double
_tl = [ord("+"), 0x250C, 0x256D, 0x250F, 0x2554]  # top left
_tr = [ord("+"), 0x2510, 0x256E, 0x2513, 0x2557]  # top right
_bl = [ord("+"), 0x2514, 0x2570, 0x2517, 0x255A]  # bottom left
_br = [ord("+"), 0x2518, 0x256F, 0x251B, 0x255C]  # bottom right
_lj = [ord("+"), 0x251C, 0x251C, 0x2523, 0x2560]  # left join
_rj = [ord("+"), 0x2524, 0x2524, 0x252B, 0x2563]  # right join
_tj = [ord("+"), 0x252C, 0x252C, 0x2533, 0x2566]  # top join
_bj = [ord("+"), 0x2534, 0x2534, 0x253B, 0x2569]  # bottom join
_xj = [ord("+"), 0x253C, 0x253C, 0x254B, 0x256C]  # allway join
_hl = [ord("-"), 0x2500, 0x2500, 0x2501, 0x2550]  # horizontal line
_vl = [ord("|"), 0x2502, 0x2502, 0x2503, 0x2551]  # vertical line

borderdict = {
    "ascii": 0,
    "thin": 1,
    "round": 2,
    "thick": 3,
    "double": 4,
    "thick-thin": 5,
    "double-thin": 6,
}
styledict = {"bold": 1, "dim": 2, "underlined": 4, "blink": 5, "reverse": 7}

# ------------------------------------------------------------------------- #


class ANSIMatrix:

    def __init__(self, style="thin", fmt="{:< 10.3g}", squish=100):
        """
        Create a matrix formatter

        :param style: Bracket format, defaults to "thin"
        :type style: str, optional
        :param fmt: number format string, defaults to "{:< 10.3g}"
        :type fmt: str, optional
        :param squish: elements smaller than this times eps are displayed as
                       zero, defaults to 100
        :type squish: int, optional

        Creates a formatter, or template, for formatting matrices.

        .. code::

            from ansitable import ANSIMatrix
            import numpy as np

            formatter = ANSIMatrix(style='ascii')
            m = np.random.rand(4,4) - 0.5
            formatter.print(m)

            +                                           +
            | 0.23      -0.00542   -0.282      0.229    |
            | 0.433      0.229      0.489     -0.414    |
            | 0.0901    -0.351     -0.413     -0.418    |
            | 0.433      0.233      0.0495     0.000281 |
            +                                           +
        """

        # TODO: add colored fields, specify by tuples (rowstart, rowend, colstart, colend, color)

        import numpy as np  # only import if matrix is used

        if not _unicode:
            style = "ascii"
        self.style = borderdict[style]
        self.fmt = fmt
        self.width = len(fmt.format(1))
        if squish == 0:
            self.squish = None
        else:
            self.squish = squish * np.finfo(float).eps

    def str(self, matrix, suffix_super="", suffix_sub=""):
        """
        Output the table as a string

        :param matrix: NumPy matrix to format
        :type matrix: 1d or 2d ndarray
        :param suffix_super: Right superscript, defaults to ''
        :type suffix_super: str, optional
        :param suffix_sub: Right subscript, defaults to ''
        :type suffix_sub: str, optional
        :raises ValueError: [description]
        :return: ANSI string
        :rtype: str

        ``suffix_super`` is where the transpose, inverse, pseudo-inverse, etc. symbol
        is shown.  ``suffix_sub`` is where the matrix name is typically shown.
        """

        import numpy as np  # only import if matrix is used

        if len(matrix.shape) == 1:
            ncols = matrix.shape[0]
            matrix = matrix.reshape((1, -1))
        elif len(matrix.shape) == 2:
            ncols = matrix.shape[1]
        else:
            raise ValueError("Only 1D and 2D arrays supported")

        mwidth = ncols * self.width + (ncols - 1)
        if self.squish is None:
            m2 = matrix
        else:
            m2 = np.where(abs(matrix) < self.squish, 0, matrix)
        b = self.style

        s = chr(_tl[b]) + " " * mwidth + chr(_tr[b]) + suffix_super + "\n"
        for row in m2:
            s += (
                chr(_vl[b])
                + " ".join([self.fmt.format(x) for x in row])
                + chr(_vl[b])
                + "\n"
            )
        s += chr(_bl[b]) + " " * mwidth + chr(_br[b]) + suffix_sub
        return s

    def print(self, matrix, *pos, file=sys.stdout, **kwargs):
        """
        Print the matrix

        :param file: Print the matrix to this file, defaults to stdout
        :type file: writeable object, optional

        .. note:: Accepts the same arguments as ``str``.

        """

        print(self.str(matrix, *pos, **kwargs), file=file)


# ------------------------------------------------------------------------- #


class ANSITable:
    _color = _colored

    def __init__(
        self,
        *pos,
        colsep=1,
        offset=0,
        border=None,
        bordercolor=None,
        ellipsis=True,
        columns=None,
        header=True,
        color=True,
    ):
        """
        Create a table object

        :param colsep: Blank padding on each side of column separator, defaults to 1
        :type colsep: int, optional
        :param offset: Horizontal offset of the whole table, defaults to 0
        :type offset: int, optional
        :param border: Type of border, defaults to None
        :type border: str, optional
        :param bordercolor: Name of color to draw border in, defaults to None
        :type bordercolor: str, optional
        :param ellipsis: truncated lines are shown with an ellipsis, defaults to True
        :type ellipsis: bool, optional
        :param columns: specify detailed column format options, defaults to None
        :type columns: list of :class:`Column` objects, optional
        :param header: Show table header, defaults to True
        :type header: bool, optional
        :param color: [description], defaults to True
        :type color: bool, optional
        :raises TypeError: [description]

        A table can be created in several different ways::

            table = ANSITable("col1", "column 2 has a big header", "column 3")

            table = ANSITable(
                Column("col1"),
                Column("column 2 has a big header", "{:.3g}"),
                Column("column 3", "{:-10.4f}")
            )

            table = ANSITable()
            table.addcolumnColumn("col1"),
            table.addcolumnColumn("column 2 has a big header", "{:.3g}"),
            table.addcolumnColumn("column 3", "{:-10.4f}")

        The first option is quick and easy but does not allow any control of
        formatting or alignment.

        ===========   ==========================================================
        Border        Description
        ===========   ==========================================================
        ascii         Use ASCII +-| characters
        thin          Use ANSI thin box-drawing characters
        thin+round    Use ANSI thin box-drawing characters with rounded corners
        thick         Use ANSI thick box-drawing characters
        double        Use ANSI double-line box-drawing characters
        ===========   ==========================================================

        """
        global _unicode

        self.colsep = colsep
        self.offset = offset
        self.ellipsis = ellipsis
        self.rows = []
        self.bordercolor = bordercolor
        self.header = header
        self.color = color and self._color

        if border is not None and not _unicode:
            border = "ascii"
        self.border = border

        self.nrows = 0
        self.columns = []
        for c in pos:
            if isinstance(c, str):
                c = Column(c)
                c.table = self
                self.columns.append(c)
            elif isinstance(c, Column):
                c.table = self
                self.columns.append(c)
            else:
                raise TypeError("expecting a lists of Column objects")
        if len(self.columns) == 0 and columns is not None:
            for _ in range(columns):
                self.columns.append("")

    def addcolumn(self, name, **kwargs):
        """
        Add a column to the table

        :param name: column heading
        :type name: str

        An alternative way to create a table, column at a time.

        Example::

            table = ANSITable()
            table.addcolumnColumn("col1"),
            table.addcolumnColumn("column 2 has a big header", "{:.3g}"),
            table.addcolumnColumn("column 3", "{:-10.4f}")

        .. note:: Additional arguments are passed directly to ``Column``.

        """
        self.columns.append(Column(name, **kwargs))

    def row(self, *values, fgcolor=None, bgcolor=None, style=None):
        """
        Add a row of data

        :param: values: data items for the row.  These can be of any type that can be converted to a string for display, eg. numbers, strings or objects with a ``__str__`` method. The :class:`Cell` encapsulates a value and allows the color or style defaults of the row or column to be overriden.
        :param fgcolor: foreground color override for all columns in the row, defaults to None
        :type fgcolor: str, optional
        :param bgcolor: background color override for all columns in the row, defaults to None
        :type bgcolor: str, optional
        :param style: style override for all columns in the row, defaults to None
        :type style: str, optional
        :raises ValueError: invalid format string for the data provided

        ``table.row(d1, d2, ... dN)`` add data items that comprise a row of the
        table.  ``N`` is the number of columns.

        The data items can be any type, but the format string specified at
        table creation must be compatible.

        The column data is formatted with the color and style given when the ``Column``
        was created, but it can be overridden for a specific row by specifying the
        options ``fgcolor``, ``bgcolor``, or ``style``.

        :class:`Cell` overrides the color and style of a cell specified for a column and a row.
        """
        assert len(values) == len(self.columns), "wrong number of data items added"

        for value, c in zip(values, self.columns):

            if isinstance(value, Cell):
                # cell object, use its attributes
                _fgcolor = value.fgcolor or fgcolor
                _bgcolor = value.bgcolor or bgcolor
                _style = value.style or style
                value = value.text
            else:
                _fgcolor = fgcolor
                _bgcolor = bgcolor
                _style = style

            if c.fmt is None:
                s = value
            elif isinstance(c.fmt, str):
                s = c.fmt.format(value)
            elif callable(c.fmt):
                s = c.fmt(value)
            else:
                raise ValueError("fmt must be valid format string or callable")

            # handle a FG color specifier
            if s.startswith("<<"):
                # color specifier is given
                end = s.find(">>")
                _fgcolor = s[2:end]
                s = s[end + 2 :]
            else:
                color = None

            if c.width is not None and len(s) > c.width:
                if self.ellipsis:
                    s = s[: c.width - 1] + "\u2026"
                else:
                    s = s[: c.width]
            c.maxwidth = max(c.maxwidth, len(s))

            c.formatted.append(s)
            c.fgcolor.append(_fgcolor)
            c.bgcolor.append(_bgcolor)
            c.style.append(_style)
        self.nrows += 1

    def __len__(self):
        """Length of table

        :return: Number of rows in the table
        :rtype: int
        """
        return self.nrows

    def rule(self):
        """
        Add a horizontal rule to the table

        This is a horizontal line across all columns, used to delineate parts
        of the table.
        """
        for c in self.columns:
            c.formatted.append(None)
            c.fgcolor.append(None)
            c.bgcolor.append(None)
            c.style.append(None)
        self.nrows += 1

    def _topline(self):
        """
        Create the top line of the table

        :return: ansi code for the top line
        :rtype: str
        """
        return self._line(_tl, _tj, _tr)

    def _midline(self):
        """
        Create a middle line of the table

        :return: ansi code for the middle line
        :rtype: str
        """
        return self._line(_lj, _xj, _rj)

    def _bottomline(self):
        """
        Create the bottom line of the table

        :return: ansi code for the bottom line
        :rtype: str
        """
        return self._line(_bl, _bj, _br)

    def _line(self, left, mid, right):
        """
        Create a general horizontal line in the table

        :param left: left hand side character
        :type left: list of str
        :param mid: column join characters
        :type mid: list of str
        :param right: right hand side character
        :type right: list of str
        :return: ansi code for line
        :rtype: str

        The line's color is specified by the ```bordercolor``` property.
        """
        if self.borderdict is None:
            return ""
        else:
            b = self.borderdict
            text = _spaces(self.offset - 1)
            if self.bordercolor is not None:
                text += self._FG(self.bordercolor)
            text += chr(left[b])
            for c in self.columns:
                text += chr(_hl[b]) * (c.width + 2 * self.colsep)
                if not c.last:
                    text += chr(mid[b])
            text += chr(right[b])
            if self.bordercolor is not None:
                text += self._ATTR(0)
            return text + "\n"

    def _vline(self):
        """
        Vertical line string

        :return: ansi code for the vertical line
        :rtype: str

        A vertical line occurs on the left- and right-hand side of every
        cell in the table.  Its color is specified by the ``bordercolor``
        property.
        """
        if self.borderdict is not None:
            # set color
            if self.bordercolor is not None:
                text = self._FG(self.bordercolor)
            else:
                text = ""

            # get vertical line from dict
            b = self.borderdict
            text += chr(_vl[b])

            # turn off color
            if self.bordercolor is not None:
                text += self._ATTR(0)
            return text

    def _row(self, row=None):
        """
        Format a row of the table

        :param row: row number starting at 0, defaults to None
        :type row: int, optional
        :return: unicode string for the row
        :rtype: str

        The row can be the header row (```row=None```) or a data row
        """

        if row is not None and self.columns[0].formatted[row] is None:
            # it's a horizontal rule
            return self._midline()

        if self.borderdict is not None:
            # has border

            # add left-hand edge of table
            text = _spaces(self.offset - 1) + self._vline()

            # add each column
            for c in self.columns:

                if row is None:
                    # header
                    text += c._formatcolumn(
                        c.name,
                        header=True,
                    )
                else:
                    # table row proper
                    text += c._formatcolumn(
                        c.formatted[row],
                        header=False,
                        fgcolor=c.fgcolor[row],
                        bgcolor=c.bgcolor[row],
                        style=c.style[row],
                    )

                text += self._vline()

                # if c.last:
                #     # last cell on row
                #     text += _spaces(c2) + self._vline()
                # else:
                #     text += _spaces(c2) + self._vline() + _spaces(c1)
        else:
            # no border
            text = _spaces(self.offset)

            for c in self.columns:
                # ansi = c._setstyle(row is None)
                # text += ansi
                if row is None:
                    # header
                    text += c._formatcolumn(
                        c.name,
                        header=True,
                    )
                else:
                    # table row proper
                    text += c._formatcolumn(
                        c.formatted[row],
                        header=False,
                        fgcolor=c.fgcolor[row],
                        bgcolor=c.bgcolor[row],
                        style=c.style[row],
                    )

                # if len(ansi) > 0:
                #     text += self._ATTR(0)
                text += _spaces(self.colsep)

        return text + "\n"

    def print(self, file=None):
        """
        Print the table

        :param file: Print the table to this file, defaults to stdout
        :type file: writeable object, optional

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            table.print()

            +--------------+---------------------------+----------+
            |         col1 | column 2 has a big header | column 3 |
            +--------------+---------------------------+----------+
            |    aaaaaaaaa |                       2.2 |        3 |
            |bbbbbbbbbbbbb |                      -5.5 |        6 |
            |      ccccccc |                       8.8 |       -9 |
            +--------------+---------------------------+----------+

        """

        try:
            print(str(self), file=file)
        except UnicodeEncodeError:
            self.border = "ascii"
            print(str(self), file=file)

    def _findwidths(self):
        # find the width of each column and flag if it is
        # the last column in the table
        for i, c in enumerate(self.columns):
            c.width = c.width or c.maxwidth
            c.last = i == len(self.columns) - 1

    def __str__(self):
        """
        Output the table as a string

        :return: ANSI string
        :rtype: str
        """

        if self.border is not None:
            # printing borders, adjust some other parameters
            if self.offset == 0:
                self.offset = 1
            # self.colsep |= 1  # make it odd

            self.borderdict = borderdict[self.border]
        else:
            self.borderdict = None

        self._findwidths()

        text = ""

        # heading
        text += self._topline()
        if self.header:
            text += self._row()  # add the headers
            text += self._midline()  # add a separator line

        # rows
        for i in range(self.nrows):
            text += self._row(i)  # add a row

        # footer
        text += self._bottomline()

        return text

    def markdown(self):
        """
        Output the table in MarkDown markup format

        :return: ASCII markdown text
        :rtype: str

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            table.markdown()

            |          col1 | column 2 has a big header | column 3 |
            | ------------: | ------------------------: | -------: |
            |     aaaaaaaaa |                       2.2 |        3 |
            | bbbbbbbbbbbbb |                      -5.5 |        6 |
            |       ccccccc |                       8.8 |       -9 |

        .. note::
            - supports column alignment
            - does not support header alignment, same as column
        """

        self._findwidths()

        # markdown table setup
        s = ""

        # column headers
        for c in self.columns:
            s += "| " + c._formatcolumn(c.name, header=True, plain=True) + " "
        s += "|\n"

        for c in self.columns:
            if c.headalign == "<":
                bar = ":" + "-" * (c.width - 1)
            elif c.headalign == "^":
                bar = ":" + "-" * (c.width - 2) + ":"
            elif c.headalign == ">":
                bar = "-" * (c.width - 1) + ":"
            s += "| " + bar + " "
        s += "|\n"

        # rows
        for i in range(self.nrows):
            for c in self.columns:
                s += (
                    "| "
                    + c._formatcolumn(c.formatted[i], header=False, plain=True)
                    + " "
                )
            s += "|\n"

        return s

    def rest(self):
        """
        Output the table in ReST "simple table" markup format

        :return: ASCII text for a ReST "simple table"
        :rtype: str

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            table.rest()

            =============  =========================  ========
                     col1  column 2 has a big header  column 3
            =============  =========================  ========
                aaaaaaaaa                        2.2         3
            bbbbbbbbbbbbb                       -5.5         6
                  ccccccc                        8.8        -9
            =============  =========================  ========

        .. note::
            - does not support header or column alignment
        """

        self._findwidths()
        colsep = self.colsep
        self.colsep = 0

        # markdown table setup
        s = ""

        # column headers
        divider = ""
        for c in self.columns:
            divider += "=" * c.width + "  "

        s += divider + "\n"

        for c in self.columns:
            s += c._formatcolumn(c.name, header=True, plain=True) + "  "
        s += "\n"
        s += divider + "\n"

        # rows
        for i in range(self.nrows):
            for c in self.columns:
                s += c._formatcolumn(c.formatted[i], header=False, plain=True) + "  "
            s += "\n"
        s += divider + "\n"
        self.colsep = colsep
        return s

    def wikitable(self):
        """
        Output the table in wikitable markup format

        This is the markup format for tables in Wikipedia.

        :return: ASCII markdown text
        :rtype: str

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            table.wikitable()

            {| class="wikitable" col1right col2right col3right
            |-
            !           col1  !!  column 2 has a big header  !!  column 3
            |-
            |      aaaaaaaaa  ||                        2.2  ||         3
            |-
            |  bbbbbbbbbbbbb  ||                       -5.5  ||         6
            |-
            |        ccccccc  ||                        8.8  ||        -9
            |}

        .. note::
            - supports column alignment
            - does not support header alignment, always centred for ``wikitable`` class
        """

        self._findwidths()

        # Wikipedia table setup
        s = '{| class="wikitable"'

        for i, c in enumerate(self.columns):
            if c.headalign == "<":
                s += " col%dleft" % (i + 1)
            elif c.headalign == "^":
                s += " col%dcenter" % (i + 1)
            elif c.headalign == ">":
                s += " col%dright" % (i + 1)
        s += "\n"

        # column headers
        s += "|-\n"
        first = True
        for c in self.columns:
            if first:
                s += "! " + c._formatcolumn(c.name, header=True, plain=True) + " "
                first = False
            else:
                s += "!! " + c._formatcolumn(c.name, header=True, plain=True) + " "

        s += "\n"
        # rows
        for i in range(self.nrows):
            s += "|-\n"
            first = True
            for c in self.columns:
                if first:
                    s += (
                        "| "
                        + c._formatcolumn(c.formatted[i], header=False, plain=True)
                        + " "
                    )
                    first = False
                else:
                    s += (
                        "|| "
                        + c._formatcolumn(c.formatted[i], header=False, plain=True)
                        + " "
                    )
            s += "\n"
        s += "|}\n"

        return s

    def html(self, td="", th="", trd="", trh="", table=""):
        r"""
        Output the table in HTML format

        :param td: CSS style for table data cells, defaults to ""
        :type td: str, optional
        :param th: CSSstyle for table header cells, defaults to ""
        :type th: str, optional
        :param trd: CSS style for table data rows, defaults to ""
        :type trd: str, optional
        :param trh: CSS style for table header rows, defaults to ""
        :type trh: str, optional
        :param table: CSS style for the table, defaults to ""
        :type table: str, optional
        :return: table rendered in HTML
        :rtype: str

        The table is rendered as a table between `<table>` and `</table>` tags.
        Table color and style options are supported.

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            table.html()

            <table>
            <tr>
                <th style='text-align:right;'>col1</th>
                <th style='text-align:right;'>column 2 has a big header</th>
                <th style='text-align:right;'>column 3</th>
            </tr>
            <tr>
                <td style='text-align:right;'>aaaaaaaaa</td>
                <td style='text-align:right;'>2.2</td>
                <td style='text-align:right;'>3</td>
            </tr>
            <tr>
                <td style='text-align:right;'>bbbbbbbbbbbbb</td>
                <td style='text-align:right;'>-5.5</td>
                <td style='text-align:right;'>6</td>
            </tr>
            <tr>
                <td style='text-align:right;'>ccccccc</td>
                <td style='text-align:right;'>8.8</td>
                <td style='text-align:right;'>-9</td>
            </tr>
            </table>

        CSS options defined in the document will apply.  These can be overridden for
        various table elements by passed arguments strings. For example, to create a
        simple gridded table::

            table.html(
                td="border: 1px solid;",
                th="border: 1px solid;",
                table="border-collapse: collapse;",
            )

        The CSS style strings must end with a semi-colon.

        .. note::
            - supports column alignment
            - supports header alignment
            - support color and style options in the table

        """
        self._findwidths()

        # HTML table setup
        if self.bordercolor is not None:
            style = "border-color:" + self.bordercolor + ";"
        else:
            style = ""
        s = "<table style='" + style + table + "'>\n"

        align = {"<": "left;", "^": "center;", ">": "right;"}

        # column headers
        s += "  <tr style='" + trh + "'>\n"
        for c in self.columns:
            style = "text-align:" + align[c.headalign]
            if c.headcolor is not None:
                style += "color:" + c.headcolor + ";"
            if c.headbgcolor is not None:
                style += "background-color:" + c.headbgcolor + ";"
            s += "    <th style='" + style + th + "'>" + c.name + "</th>\n"
        s += "  </tr>\n"

        # rows
        for i in range(self.nrows):
            s += "  <tr style='" + trd + "'>\n"
            for c in self.columns:
                style = "text-align:" + align[c.colalign]
                if c.fgcolor[i] is not None:
                    style += "color:" + c.fgcolor[i] + ";"
                if c.bgcolor[i] is not None:
                    style += "background-color:" + c.bgcolor[i] + ";"
                s += (
                    "    <td style='"
                    + style
                    + td
                    + "'>"
                    + c.formatted[i].replace(
                        "\u2026", "&hellip;"
                    )  # replace unicode ellipsis with HTML entity
                    + "</td>\n"
                )
            s += "  </tr>\n"
        s += "</table>\n"
        return s

    def latex(self):
        r"""
        Output the table in LaTeX format

        :return: ASCII markdown text
        :rtype: str

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            table.latex()

            \begin{tabular}{ |r|r|r| }\hline
            \multicolumn{1}{|r|}{col1} & \multicolumn{1}{|r|}{column 2 has a big header} & \multicolumn{1}{|r|}{column 3}\\\hline\hline
            aaaaaaaaa & 2.2 & 3 \\
            bbbbbbbbbbbbb & -5.5 & 6 \\
            ccccccc & 8.8 & -9 \\
            \hline
            \end{tabular}

        .. note::
            - supports column alignment
            - supports header alignment
        """
        self._findwidths()

        # LaTeX tabular setup
        s = "\\begin{tabular}{ |"
        for c in self.columns:
            if c.colalign == "<":
                s += "l|"
            elif c.colalign == "^":
                s += "c|"
            elif c.colalign == ">":
                s += "r|"
        s += " }\\hline\n"

        # column headers
        first = True
        for c in self.columns:
            if first:
                first = False
            else:
                s += " & "

            s += "\\multicolumn{1}{"
            if c.headalign == "<":
                s += "|l|"
            elif c.headalign == "^":
                s += "|c|"
            elif c.headalign == ">":
                s += "|r|"
            s += "}{" + c.name + "}"

        s += "\\\\\\hline\\hline\n"
        # rows
        for i in range(self.nrows):
            first = True
            for c in self.columns:
                if first:
                    first = False
                else:
                    s += " & "
                s += c.formatted[i]
            s += " \\\\\n"
        s += "\\hline\n"
        s += "\\end{tabular}\n"
        return s

    def csv(self, delimiter=","):
        r"""
        Output the table in comma separated column (CSV) format

        :param delimiter: column delimiter, defaults to ","
        :type delimiter: str, optional
        :return: ASCII CSV text
        :rtype: str

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            table.csv()

            col1,column 2 has a big header,column 3
            aaaaaaaaa,2.2,3
            bbbbbbbbbbbbb,-5.5,6
            ccccccc,8.8,-9

        .. note::
            - does not support header or column alignment
        """

        self._findwidths()

        # column headers

        first = True
        s = ""
        for c in self.columns:
            if first:
                first = False
            else:
                s += delimiter
            s += c.name
        s += "\n"

        # rows
        for i in range(self.nrows):
            first = True
            for c in self.columns:
                if first:
                    first = False
                else:
                    s += delimiter
                s += c.formatted[i]
            s += "\n"

        return s

    def pandas(self, underscores=True):
        r"""
        Convert the table to a Pandas dataframe

        :param underscores: replace spaces in column names with underscores, defaults to True
        :type underscores: bool, optional
        :return: Pandas dataframe
        :rtype: `DataFrame` object

        Example::

            table = ANSITable("col1", "column 2 has a big header", "column 3")
            table.row("aaaaaaaaa", 2.2, 3)
            table.row("bbbbbbbbbbbbb", -5.5, 6)
            table.row("ccccccc", 8.8, -9)
            df = table.pandas()
            print(df)

                        col1 column_2_has_a_big_header column_3
            0      aaaaaaaaa                       2.2        3
            1  bbbbbbbbbbbbb                      -5.5        6
            2        ccccccc                       8.8       -9

        .. note::
            - does not support header or column alignment
            - ANSItable column headings can contain spaces, but Pandas column names
              with spaces cannot be used as attributes. By default spaces are replaced
              with underscores, but this can be disabled by passing ``underscores=False``.
        """

        try:
            import pandas as pd
        except ImportError:
            raise ImportError("pandas is not installed: pip install pandas")

        data = {}
        for c in self.columns:
            if underscores:
                colname = c.name.replace(" ", "_")
            else:
                colname = c.name
            data[colname] = c.formatted
        return pd.DataFrame(data)

    @staticmethod
    def Pandas(df, **kwargs):
        """
        Create an ANSITable from a Pandas dataframe

        :param df: Pandas dataframe
        :type df: :class:`pandas.DataFrame`
        :param kwargs: additional arguments to pass to the ANSITable constructor
        :return: an ansitable object
        :rtype: :class:`ANSITable`

        Example::

            import pandas as pd
            from ansitable import ANSITable

            df = pd.DataFrame({"calories": [420, 380, 390], "duration": [50, 40, 45]})
            table = ANSITable.Pandas(df, border="thin")
            table.print()

            ┌──────────┬──────────┐
            │ calories │ duration │
            ├──────────┼──────────┤
            │      420 │       50 │
            │      380 │       40 │
            │      390 │       45 │
            └──────────┴──────────┘

        .. note::
            - options for header and column alignment and format are not supported
            -
        """
        try:
            import pandas as pd
        except ImportError:
            raise ImportError("pandas is not installed: pip install pandas")

        table = ANSITable(*list(df.columns), **kwargs)
        for i in range(len(df)):
            table.row(*df.iloc[i])
        return table

    def _FG(self, c):
        if _unicode and self.color and c is not None:
            return fore(c)
        else:
            return ""

    def _BG(self, c):
        if _unicode and self.color and c is not None:
            return back(c)
        else:
            return ""

    def _ATTR(self, c):
        if _unicode and self.color and c is not None:
            return style(c)
        else:
            return ""


if __name__ == "__main__":

    import numpy as np

    ANSITable._unicode = False

    # -------------------------------- test ANSIMatrix
    m = np.arange(16).reshape((4, 4)) / 10 - 0.8
    m[0, 0] = 1.23456e-14
    print(m)
    print(np.array2string(m))

    formatter = ANSIMatrix(style="thick", squish=True)

    formatter.print(m)

    formatter.print(m, suffix_super="T", suffix_sub="3")

    m = np.arange(4) / 4 - 0.5
    formatter.print(m, "T")

    # -------------------------------- test ANSITable

    table = ANSITable("col1", "column 2 has a big header", "column 3", color=False)
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable("col1", "column 2 has a big header", "column 3")
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("<<red>>bbbbbbbbbbbbb", 5.5, 6)
    table.row("<<blue>>ccccccc", 8.8, -9)
    table.print()

    table = ANSITable("col1", "column 2 has a big header", "column 3")
    table.row("aaaaaaaaa", 2.2, 3, bgcolor="green")
    table.row(Cell("bbbbbbbbbbbbb", bgcolor="red"), 5.5, 6)
    table.row(Cell("ccccccc", fgcolor="blue"), 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1"), Column("column 2 has a big header"), Column("column 3")
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1"),
        Column("column 2 has a big header", "{:.3g}"),
        Column("column 3", "{:-10.4f}"),
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1", width=10),
        Column("column 2 has a big header", "{:.3g}"),
        Column("column 3", "{:-10.4f}"),
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1"),
        Column("column 2 has a big header"),
        Column("column 3"),
        border="ascii",
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1"),
        Column("column 2 has a big header"),
        Column("column 3"),
        border="thick",
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.rule()
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1"),
        Column("column 2 has a big header", colalign="^"),
        Column("column 3"),
        border="thick",
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1", headalign="<"),
        Column("column 2 has a big header", colalign="^"),
        Column("column 3", colalign="<"),
        border="thick",
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1", headalign="<"),
        Column("column 2 has a big header", colalign="^", colstyle="reverse"),
        Column("column 3", colalign="<"),
        border="thick",
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1", headalign="<", colcolor="red", headstyle="underlined"),
        Column("column 2 has a big header", colalign="^", colstyle="reverse"),
        Column("column 3", colalign="<", colbgcolor="green", fmt="{: d}"),
        border="thick",
        bordercolor="blue",
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.rule()
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1", headalign="^", colcolor="red", headstyle="underlined"),
        Column("column 2 has a big header", colalign="^"),
        Column("column 3", colalign="<", colbgcolor="green", fmt="{: d}"),
        border="thick",
        bordercolor="blue",
        colsep=2,
    )
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()
    print(table.html())

    # markdown/latex/html example
    table = ANSITable("col1", "column 2 has a big header", "column 3")
    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()
    print(table.latex())
    print(table.markdown())
    print(table.csv())
    print(table.html())
    print(table.rest())
    print(table.wikitable())

    df = table.pandas()
    print(df)
    print(df.column_2_has_a_big_header.to_string())

    import pandas as pd

    df = pd.DataFrame({"calories": [420, 380, 390], "duration": [50, 40, 45]})
    table = ANSITable.Pandas(df, border="thin")
    table.print()

    table = ANSITable(
        Column("col1", headalign="<", colcolor="red", headstyle="underlined"),  # CHANGE
        Column("column 2 has a big header", colalign="^", colstyle="bold"),  # CHANGE
        Column("column 3", colalign="<", colbgcolor="green"),  # CHANGE
        border="thick",
        bordercolor="blue",  # CHANGE
    )

    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6)
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1", headalign="<", colcolor="red", headstyle="underlined"),  # CHANGE
        Column("column 2 has a big header", colalign="^", colstyle="bold"),  # CHANGE
        Column("column 3", colalign="<", colbgcolor="green"),  # CHANGE
        border="thick",
        bordercolor="blue",  # CHANGE
    )

    table.row("aaaaaaaaa", 2.2, 3)
    table.row("bbbbbbbbbbbbb", -5.5, 6, bgcolor="yellow")  # CHANGE
    table.row("ccccccc", 8.8, -9)
    table.print()

    table = ANSITable(
        Column("col1", headalign="<", colcolor="red", headstyle="underlined"),  # CHANGE
        Column("column 2 has a big header", colalign="^", colstyle="bold"),  # CHANGE
        Column("column 3", colalign="<", colbgcolor="green"),  # CHANGE
        border="thick",
        bordercolor="blue",  # CHANGE
    )

    table.row("aaaaaaaaa", 2.2, 3)
    table.row(
        "bbbbbbbbbbbbb", Cell(-5.5, bgcolor="blue"), 6, bgcolor="yellow"
    )  # CHANGE
    table.row("ccccccc", 8.8, -9)
    table.print()

    ##
    table = ANSITable(
        Column("col1", headalign="<", headstyle="underlined"),  # CHANGE
        Column("column 2 has a big header", colalign="^", colstyle="bold"),  # CHANGE
        Column("column 3", colalign="<"),  # CHANGE
        border="thin",
    )

    table.row("aaaaaaaaa", "<<blue>>aaa", "")
    table.row(
        "bbbbbbbbbbbbb",
        "<<blue>>blue",
        6,
    )  # CHANGE
    table.row("ccccccc", 8.8, -9)
    table.print()
