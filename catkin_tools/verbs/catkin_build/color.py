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

"""This module implements many of the colorization functions used by catkin build"""

from catkin_tools.terminal_color import ansi
from catkin_tools.terminal_color import fmt
from catkin_tools.terminal_color import sanitize
from catkin_tools.terminal_color import ColorMapper

# This map translates more human reable format strings into colorized versions
_color_translation_map = {
    # 'output': 'colorized_output'
    '': fmt('@!' + sanitize('') + '@|'),

    "[{package}] ==> '{cmd.cmd_str}' in '{location}'":
    fmt("[@{cf}{package}@|] @!@{bf}==>@| '@!{cmd.cmd_str}@|' @{kf}@!in@| '@!{location}@|'"),

    "Starting ==> {package}":
    fmt("Starting @!@{gf}==>@| @!@{cf}{package}@|"),

    "[{package}] {msg}":
    fmt("[@{cf}{package}@|] {msg}"),

    "[{package}] <== '{cmd.cmd_str}' failed with return code '{retcode}'":
    fmt("[@{cf}{package}@|] @!@{rf}<==@| '@!{cmd.cmd_str}@|' @{rf}failed with return code@| '@!{retcode}@|'"),

    "[{package}] <== '{cmd.cmd_str}' finished with return code '{retcode}'":
    fmt("[@{cf}{package}@|] @{gf}<==@| '@!{cmd.cmd_str}@|' finished with return code '@!{retcode}@|'"),

    "Finished <== {package:<":
    fmt("@!@{kf}Finished@| @{gf}<==@| @{cf}{package:<").rstrip(ansi('reset')),

    "Failed <== {package:<":
    fmt("@!@{rf}Failed@|   @{gf}<==@| @{cf}{package:<").rstrip(ansi('reset')),

    "} [ {time} ]":
    fmt("}@| [ @{yf}{time}@| ]"),

    "[build - {run_time}] ":
    fmt("[@{pf}build@| - @{yf}{run_time}@|] "),

    "[{name} - {run_time}] ":
    fmt("[@{cf}{name}@| - @{yf}{run_time}@|] "),

    "[{0}/{1} Active | {2}/{3} Completed]":
    fmt("[@!@{gf}{0}@|/@{gf}{1}@| Active | @!@{gf}{2}@|/@{gf}{3}@| Completed]"),

    "[!{package}] ":
    fmt("[@!@{rf}!@|@{cf}{package}@|] "),
}

color_mapper = ColorMapper(_color_translation_map)

clr = color_mapper.clr


def colorize_cmake(line):
    """Colorizes output from CMake

    :param line: one, new line terminated, line from `cmake` which needs coloring.
    :type line: str
    """
    cline = sanitize(line)
    if line.startswith('-- '):
        cline = '@{cf}-- @|' + cline[len('-- '):]
        if ':' in cline:
            split_cline = cline.split(':')
            cline = split_cline[0] + ':@{yf}' + ':'.join(split_cline[1:]) + '@|'
    if line.lower().startswith('warning'):
        # WARNING
        cline = fmt('@{yf}') + cline
    if line.startswith('CMake Warning'):
        # CMake Warning...
        cline = cline.replace('CMake Warning', '@{yf}@!CMake Warning@|')
    if line.startswith('ERROR:'):
        # ERROR:
        cline = cline.replace('ERROR:', '@!@{rf}ERROR:@|')
    if line.startswith('CMake Error'):
        # CMake Error...
        cline = cline.replace('CMake Error', '@{rf}@!CMake Error@|')
    if line.startswith('Call Stack (most recent call first):'):
        # CMake Call Stack
        cline = cline.replace('Call Stack (most recent call first):',
                              '@{cf}@_Call Stack (most recent call first):@|')
    return fmt(cline)
