# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Module to enable color terminal output
"""

import string
import os

_ansi = {}


def ansi(key):
    """Returns the escape sequence for a given ansi color key"""
    global _ansi
    return _ansi[key]


def enable_ANSI_colors():
    """
    Populates the global module dictionary `ansi` with ANSI escape sequences.
    """
    global _ansi
    color_order = [
        'black', 'red', 'green', 'yellow', 'blue', 'purple', 'cyan', 'white'
    ]
    short_colors = {
        'black': 'k', 'red': 'r', 'green': 'g', 'yellow': 'y', 'blue': 'b',
        'purple': 'p', 'cyan': 'c', 'white': 'w'
    }
    _ansi = {
        'escape': '\033', 'reset': 0, '|': 0,
        'boldon': 1, '!': 1, 'italicson': 3, '/': 3, 'ulon': 4, '_': 4,
        'invon': 7, 'boldoff': 22, 'italicsoff': 23,
        'uloff': 24, 'invoff': 27
    }

    # Convert plain numbers to escapes
    for key in _ansi:
        if key != 'escape':
            _ansi[key] = '{0}[{1}m'.format(_ansi['escape'], _ansi[key])

    # Foreground
    for index, color in enumerate(color_order):
        _ansi[color] = '{0}[{1}m'.format(_ansi['escape'], 30 + index)
        _ansi[color + 'f'] = _ansi[color]
        _ansi[short_colors[color] + 'f'] = _ansi[color + 'f']

    # Background
    for index, color in enumerate(color_order):
        _ansi[color + 'b'] = '{0}[{1}m'.format(_ansi['escape'], 40 + index)
        _ansi[short_colors[color] + 'b'] = _ansi[color + 'b']

    # Fmt sanitizers
    _ansi['atexclimation'] = '@!'
    _ansi['atfwdslash'] = '@/'
    _ansi['atunderscore'] = '@_'
    _ansi['atbar'] = '@|'


def disable_ANSI_colors():
    """
    Sets all the ANSI escape sequences to empty strings, effectively disabling
    console colors.
    """
    global _ansi
    for key in _ansi:
        _ansi[key] = ''


# Default to ansi colors on
enable_ANSI_colors()
if os.name in ['nt']:
    disable_ANSI_colors()

_color_on = True


def set_color(state):
    """Sets the global colorization setting.

    Setting this to False will cause all ansi colorization sequences to get
    replaced with empty strings.

    :parma state: colorization On or Off, True or False respectively
    :type state: bool
    """
    global _color_on
    if state:
        enable_ANSI_colors()
        _color_on = True
    else:
        disable_ANSI_colors()
        _color_on = False


class ColorTemplate(string.Template):
    delimiter = '@'


def sanitize(msg):
    """Sanitizes the existing msg, use before adding color annotations"""
    msg = msg.replace('@', '@@')
    msg = msg.replace('{', '{{')
    msg = msg.replace('}', '}}')
    msg = msg.replace('@@!', '@{atexclimation}')
    msg = msg.replace('@@/', '@{atfwdslash}')
    msg = msg.replace('@@_', '@{atunderscore}')
    msg = msg.replace('@@|', '@{atbar}')
    return msg


def fmt(msg, reset=True):
    """Replaces color annotations with ansi escape sequences"""
    global _ansi
    msg = msg.replace('@!', '@{boldon}')
    msg = msg.replace('@/', '@{italicson}')
    msg = msg.replace('@_', '@{ulon}')
    msg = msg.replace('@|', '@{reset}')
    t = ColorTemplate(msg)
    return t.substitute(_ansi) + (ansi('reset') if reset else '')


def test_colors():
    def cprint(msg):
        print(fmt(msg))

    cprint("| @{kf}Black      @|| @!@{kf}Black Bold")
    cprint("| @{rf}Red        @|| @!@{rf}Red Bold")
    cprint("| @{gf}Green      @|| @!@{gf}Green Bold")
    cprint("| @{yf}Yellow     @|| @!@{yf}Yellow Bold")
    cprint("| @{bf}Blue       @|| @!@{bf}Blue Bold")
    cprint("| @{pf}Purple     @|| @!@{pf}Purple Bold")
    cprint("| @{cf}Cyan       @|| @!@{cf}Cyan Bold")
    cprint("| White      | @!White Bold")


class ColorMapper(object):

    """This class encapsulates a set of human-readable format strings and the
    functionality to convert them into colorized version.
    """

    # This map translates more human reable format strings into colorized versions
    default_color_translation_map = {
        # 'output': 'colorized_output'
        '': fmt('@!' + sanitize('') + '@|')
    }

    def __init__(self, color_map={}):
        """Create a color mapper with a given map.

        :param color_map: A dictionary of format strings and colorized verisons
        :type color_map: dict
        """
        self.color_map = ColorMapper.default_color_translation_map
        self.color_map.update(color_map)

    def clr(self, key):
        """Returns a colorized version of the string given.

        This is occomplished by either returning a hit from the color translation
        map or by calling :py:func:`fmt` on the string and returning it.

        :param key: string to be colorized
        :type key: str
        """
        global _color_on
        if not _color_on:
            return fmt(key)
        val = self.color_map.get(key, None)
        if val is None:
            return fmt(key)
        return val
