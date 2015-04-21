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

from __future__ import print_function

import datetime
import os
import re

from catkin_pkg.packages import find_packages

from .terminal_color import ColorMapper

color_mapper = ColorMapper()
clr = color_mapper.clr

try:
    string_type = basestring
except NameError:
    string_type = str


class FakeLock(object):

    """Fake lock used to mimic a Lock but without causing synchronization"""

    def acquire(self, blocking=False):
        return True

    def release(self):
        pass

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_value, traceback):
        pass


def getcwd(symlinks=True):
    """Get the current working directory.

    :param symlinks: If True, then get the path considering symlinks. If false,
    resolve the path to the actual path.
    :type symlinks: bool
    :returns: the current working directory
    :rtype: str
    """

    cwd = ''

    # Get the real path
    realpath = os.getcwd()

    # The `PWD` environment variable should contain the path that we took to
    # get here, includng symlinks
    if symlinks:
        cwd = os.environ.get('PWD', '')

    # Fallback on `getcwd` if the `PWD` variable is wrong
    if not cwd or not os.path.exists(cwd) or os.path.realpath(cwd) != realpath:
        cwd = realpath

    return cwd


def format_time_delta(delta):
    """Formats a given time delta, in seconds, into a day-hour-minute-second string

    Seconds are limited to one decimal point accuracy. Days, hours, and minutes
    are not printed unless required.

    Examples:
        1.45        => 1.4 seconds
        61.45       => 1 minute and 1.4 seconds
        121.45      => 2 minutes and 1.4 seconds
        3721.45     => 1 hour 2 minutes and 1.4 seconds
        7321.45     => 2 hours 2 minutes and 1.4 seconds
        93821.45    => 1 days, 2 hours 2 minutes and 1.4 seconds

    :param delta: time delta to format, in seconds
    :type delta: float
    :returns: formatted time string
    :rtype: str
    """
    days = "0"
    date_str = str(datetime.timedelta(seconds=delta))
    if ', ' in date_str:
        days, date_str = date_str.split(', ')
    hours, minutes, seconds = date_str.split(':')
    msg = "" if int(days.split(' ')[0]) == 0 else days + " "
    msg += "" if int(hours) == 0 else (hours + " hour{0} ".format('' if int(hours) <= 1 else 's'))
    msg += "" if int(minutes) == 0 else ("{0} minute{1} and ".format(int(minutes), '' if int(minutes) <= 1 else 's'))
    msg += "{0:.1f}".format(float(seconds))
    msg += " seconds"
    return msg


def format_time_delta_short(delta):
    """Formats a given time delta, in seconds, into a short day-hour-minute-second string

    Seconds are limited to one decimal point accuracy. Days, hours, and minutes
    are not printed unless required.

    Examples:
        1.45        => 1.4
        61.45       => 1:01.4
        121.45      => 2:01.4
        3721.45     => 1:02:01.4
        7321.45     => 2:02:01.4
        93821.45    => 1 days, 2:02:01.4

    :param delta: time delta to format, in seconds
    :type delta: float
    :returns: formatted time string
    :rtype: str
    """
    days = "0"
    date_str = str(datetime.timedelta(seconds=delta))
    if ', ' in date_str:
        days, date_str = date_str.split(', ')
    hours, minutes, seconds = date_str.split(':')
    msg = "" if int(days.split(' ')[0]) == 0 else days + " "
    msg += "" if int(hours) == 0 else (hours + ":")
    msg += "" if int(minutes) == 0 else (minutes + ":")
    msg += ("{0:.1f}" if int(minutes) == 0 else "{0:04.1f}").format(float(seconds))
    return msg

__recursive_build_depends_cache = {}


def get_cached_recursive_build_depends_in_workspace(package, workspace_packages):
    """Returns cached or calculated recursive build dependes for a given package

    If the recursive build depends for this package and this set of workspace
    packages has already been calculated, the cached results are returned.

    :param package: package for which the recursive depends should be calculated
    :type package: :py:class:`catkin_pkg.package.Package`
    :param workspace_packages: packages in the workspace, keyed by name, with
        value being a tuple of package path and package object
    :type workspace_packages: dict(package_name, tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive build depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """
    workspace_key = ','.join([pkg.name for pth, pkg in workspace_packages])
    if workspace_key not in __recursive_build_depends_cache:
        __recursive_build_depends_cache[workspace_key] = {}
    cache = __recursive_build_depends_cache[workspace_key]
    if package.name not in cache:
        cache[package.name] = get_recursive_build_depends_in_workspace(package, workspace_packages)
    __recursive_build_depends_cache[workspace_key] = cache
    return __recursive_build_depends_cache[workspace_key][package.name]


def get_recursive_build_depends_in_workspace(package, ordered_packages):
    """Calculates the recursive build dependencies of a package which are also in the ordered_packages

    :param package: package for which the recursive depends should be calculated
    :type package: :py:class:`catkin_pkg.package.Package`
    :param ordered_packages: packages in the workspace, ordered topologically,
        stored as a list of tuples of package path and package object
    :type ordered_packages: list(tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive build depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """
    workspace_packages_by_name = dict([(pkg.name, (pth, pkg)) for pth, pkg in ordered_packages])
    workspace_package_names = [pkg.name for pth, pkg in ordered_packages]
    recursive_depends = []
    deps = package.build_depends + package.buildtool_depends + package.test_depends
    depends = set([dep.name for dep in deps])
    checked_depends = set()
    while list(depends - checked_depends):
        # Get a dep
        dep = list(depends - checked_depends).pop()
        # Add the dep to the checked list
        checked_depends.add(dep)
        # If it is not in the workspace, continue
        if dep not in workspace_package_names:
            continue
        # Add the build, buildtool, and run depends of this dep to the list to be checked
        dep_pth, dep_pkg = workspace_packages_by_name[dep]
        dep_depends = dep_pkg.build_depends + dep_pkg.buildtool_depends + dep_pkg.run_depends
        depends.update(set([d.name for d in dep_depends]))
        # Add this package to the list of recursive dependencies for this package
        recursive_depends.append((dep_pth, dep_pkg))
    return recursive_depends


def get_recursive_run_depends_in_workspace(packages, ordered_packages):
    """Calculates the recursive run depends of a set of packages which are also in the ordered_packages
    but excluding packages which are build depended on by another package in the list

    :param packages: packages for which the recursive depends should be calculated
    :type packages: list of :py:class:`catkin_pkg.package.Package`
    :param ordered_packages: packages in the workspace, ordered topologically,
        stored as a list of tuples of package path and package object
    :type ordered_packages: list(tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive run depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """
    workspace_packages_by_name = dict([(pkg.name, (pth, pkg)) for pth, pkg in ordered_packages])
    workspace_package_names = [pkg.name for pth, pkg in ordered_packages]
    recursive_depends = []
    depends = set([dep.name for package in packages for dep in package.run_depends])
    checked_depends = set()
    while len(depends - checked_depends) > 0:
        # Get a dep
        dep = list(depends - checked_depends).pop()
        # Add the dep to the checked list
        checked_depends.add(dep)
        # If it is not in the workspace, continue
        if dep not in workspace_package_names:
            continue
        # Add the run depends of this dep to the list to be checked
        dep_pth, dep_pkg = workspace_packages_by_name[dep]
        depends.update(set([d.name for d in dep_pkg.run_depends]))
        # Also update the checked_depends with its build depends
        checked_depends.update(set([d.name for d in (dep_pkg.buildtool_depends + dep_pkg.build_depends)]))
        # Add this package to the list of recursive dependencies for this package
        recursive_depends.append((dep_pth, dep_pkg))
    return recursive_depends


def is_tty(stream):
    """Returns True if the given stream is a tty, else False"""
    return hasattr(stream, 'isatty') and stream.isatty()


def log(*args, **kwargs):
    """Wrapper for print, allowing for special handling where necessary"""
    if 'end_with_escape' not in kwargs or kwargs['end_with_escape'] is True:
        args = list(args)
        escape_reset = clr('@|')
        if escape_reset:
            args.append(escape_reset)
        if 'end_with_escape' in kwargs:
            del kwargs['end_with_escape']
    print(*args, **kwargs)


def terminal_width_windows():
    """Returns the estimated width of the terminal on Windows"""
    from ctypes import windll, create_string_buffer
    h = windll.kernel32.GetStdHandle(-12)
    csbi = create_string_buffer(22)
    res = windll.kernel32.GetConsoleScreenBufferInfo(h, csbi)

    # return default size if actual size can't be determined
    if not res:
        return 80

    import struct
    (bufx, bufy, curx, cury, wattr, left, top, right, bottom, maxx, maxy)\
        = struct.unpack("hhhhHhhhhhh", csbi.raw)
    width = right - left + 1

    return width


def terminal_width_linux():
    """Returns the estimated width of the terminal on linux"""
    width = os.popen('tput cols', 'r').readline()

    return int(width)


def terminal_width():
    """Returns the estimated width of the terminal"""
    try:
        return terminal_width_windows() if os.name == 'nt' else terminal_width_linux()
    except ValueError:
        # Failed to get the width, use the default 80
        return 80

_ansi_escape = re.compile(r'\x1b[^m]*m')


def remove_ansi_escape(string):
    """Removes any ansi escape sequences from a string and returns it"""
    global _ansi_escape
    return _ansi_escape.sub('', string)


def slice_to_printed_length(string, length):
    """Truncates a string, which may contain non-printable characters, to a printed length

    For example:

        msg = '\033[32mfoo\033[31mbar\033[0m'

    has a length of 20, but a printed length of 6. If you wanted to truncate the
    printed string to 4, then printing ``msg[4]`` would not provide the desired
    result. Instead the actual slice index must consider the non-printable
    characters.

    :param string: string to be truncated
    :type string: str
    :param length: printed length of the resulting string
    :type length: int
    :returns: truncated string
    :rtype: str
    """
    global _ansi_escape
    lookup_array = []
    current_index = 0
    matches = list(_ansi_escape.finditer(string))
    for m in matches:
        for x in range(m.start() - current_index):
            lookup_array.append(current_index)
            current_index += 1
        current_index += len(m.group())
    if not matches:
        # If no matches, then set the lookup_array to a plain range
        lookup_array = range(len(string))
    return string[:lookup_array[length]] + clr('@|')


def printed_fill(string, length):
    """Textwrapping for strings with esacpe characters."""

    splat = string.replace('\\n', ' \\n ').split()
    count = 0
    lines = []
    cur_line = []
    for word in splat:
        word_len = len(remove_ansi_escape(word))
        found_newline = (word == '\\n')
        if found_newline:
            lines.append(cur_line)
            cur_line = []
            count = 0
        elif count + word_len < length:
            cur_line.append(word)
            count += word_len + 1
        else:
            if len(cur_line) > 0:
                lines.append(cur_line)
            cur_line = [word]
            count = word_len + 1

    if len(cur_line) > 0:
        lines.append(cur_line)

    return ("\n".join([' '.join(line) for line in lines])).replace('\\t', '\t')


def __wide_log(msg, **kwargs):
    width = terminal_width()
    rhs = ''
    if 'rhs' in kwargs:
        rhs = ' ' + kwargs['rhs']
        del kwargs['rhs']
    if rhs:
        kwargs['truncate'] = True
    rhs_len = len(remove_ansi_escape(rhs))
    msg_len = len(remove_ansi_escape(msg))
    if 'truncate' in kwargs:
        if kwargs['truncate'] and msg_len >= width - 1:
            msg = slice_to_printed_length(msg, width - rhs_len - 4) + '...'
            msg_len = len(remove_ansi_escape(msg))
        del kwargs['truncate']
    if (msg_len + rhs_len) < width:
        log(msg + (' ' * (width - msg_len - rhs_len - 1)) + rhs, **kwargs)
    else:
        log(msg, **kwargs)

wide_log_fn = __wide_log


def disable_wide_log():
    """Disables wide logging globally

    :see: :py:func:`wide_log`
    """
    global wide_log_fn

    def disabled_wide_log(msg, **kwargs):
        if 'rhs' in kwargs:
            del kwargs['rhs']
        if 'truncate' in kwargs:
            del kwargs['truncate']
        log(msg, **kwargs)

    wide_log_fn = disabled_wide_log


def wide_log(msg, **kwargs):
    """Prints a message to the screen, filling the remainder of the screen with spaces

    This is useful for printing lines which will completely overwrite previous
    content printed with a carriage return at the end.

    If the message is wider than the terminal, then no filling is done.

    The wide logging can be disabled with :py:func:`disable_wide_log`, in order
    to prevent queries to the terminal width, which is useful when output is
    not to a terminal, like when being used with Continuous Integration.

    Truncating and right hand side messages are disabled when wide_log is
    disabled as well.

    When a right hand side message is given, it implies truncate is True.

    :param msg: message to be printed
    :type msg: str
    :param rhs: message to print at the right hand side of the screen
    :type rhs: str
    :param truncate: If True, messages wider the then terminal will be truncated
    :type truncate: bool
    """
    global wide_log_fn
    wide_log_fn(msg, **kwargs)


def find_enclosing_package(search_start_path=None, ws_path=None, warnings=None, symlinks=True):
    """Get the package containing the current directory."""

    search_start_path = search_start_path or getcwd(symlinks=symlinks)
    child_path = ''

    while True:
        pkgs = find_packages(search_start_path, warnings=warnings)

        # Check if the previous directory is a catkin package
        if child_path in pkgs:
            return pkgs[child_path].name

        # Update search path or end
        (search_start_path, child_path) = os.path.split(search_start_path)
        if len(child_path) == 0 or search_start_path == ws_path:
            break

    return None


def version_tuple(v):
    """Get an integer version tuple from a string."""
    return tuple(map(int, (str(v).split("."))))
