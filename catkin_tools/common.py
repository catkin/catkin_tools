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

import asyncio
import datetime
import errno
import os
import re
import shutil
import sys
from fnmatch import fnmatch
from itertools import chain
from shlex import split as cmd_split
from shlex import quote as cmd_quote

from catkin_pkg.packages import find_packages

from .terminal_color import ColorMapper

color_mapper = ColorMapper()
clr = color_mapper.clr


class FakeLock(asyncio.locks.Lock):

    """Fake lock used to mimic an asyncio.Lock but without causing synchronization"""

    def locked(self):
        return False

    async def acquire(self):
        return True

    def release(self):
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
    msg += "" if len(msg) == 0 and int(hours) == 0 else (hours + ":")
    msg += "" if len(msg) == 0 and int(minutes) == 0 else (minutes + ":")
    msg += ("{0:.1f}" if len(msg) == 0 and int(minutes) == 0 else "{0:04.1f}").format(float(seconds))
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


def get_recursive_depends_in_workspace(
        packages,
        ordered_packages,
        include_function,
        exclude_function):
    """Computes the recursive dependencies of a package in a workspace based on
    include and exclude functions of each package's dependencies.

    :param packages: package for which the recursive depends should be calculated
    :type packages: list(:py:class:`catkin_pkg.package.Package`)
    :param ordered_packages: packages in the workspace, ordered topologically
    :type ordered_packages: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    :param include_function: a function which take a package and returns a list of packages to include
    :type include_function: callable
    :param exclude_function: a function which take a package and returns a list of packages to exclude
    :type exclude_function: callable

    :returns: list of package path, package object tuples which are the
        recursive build depends for the given package. These are in the
        same order as that of `ordered_packages`
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """

    # Get a package name map
    workspace_packages_by_name = {
        pkg.name: (pth, pkg)
        for pth, pkg in ordered_packages
    }

    # Initialize working sets
    pkgs_to_check = set(
        pkg.name
        # Only include the packages where the condition has evaluated to true
        for pkg in chain(*(filter(lambda pkg: pkg.evaluated_condition, include_function(p)) for p in packages))
    )

    checked_pkgs = set()
    recursive_deps = set()

    while len(pkgs_to_check) > 0:
        # Get a dep
        pkg_name = pkgs_to_check.pop()
        # If it is not in the workspace, continue
        if pkg_name not in workspace_packages_by_name:
            continue
        # Add this package's dependencies which should be checked
        _, pkg = workspace_packages_by_name[pkg_name]
        pkgs_to_check.update(
            d.name
            for d in filter(lambda pkg: pkg.evaluated_condition, include_function(pkg))
            if d.name not in checked_pkgs
        )
        # Add this package's dependencies which shouldn't be checked
        checked_pkgs.update(
            d.name
            for d in exclude_function(pkg)
        )
        # Add the package itself in case we have a circular dependency
        checked_pkgs.add(pkg.name)
        # Add this package to the list of recursive dependencies for this package
        recursive_deps.add(pkg.name)

    # Return packages in the same order as ordered_packages
    ordered_recursive_deps = [
        (pth, pkg_obj)
        for pth, pkg_obj in ordered_packages
        if pkg_obj.name in recursive_deps
    ]

    return ordered_recursive_deps


def get_recursive_build_depends_in_workspace(package, ordered_packages):
    """Calculates the recursive build dependencies of a package which are also in the ordered_packages

    :param package: package for which the recursive depends should be calculated
    :type package: :py:class:`catkin_pkg.package.Package`
    :param ordered_packages: packages in the workspace, ordered topologically
    :type ordered_packages: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive build depends for the given package. These are in the
        same order as that of `ordered_packages`
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """

    return get_recursive_depends_in_workspace(
        [package],
        ordered_packages,
        include_function=lambda p: (
            p.build_depends +
            p.buildtool_depends +
            p.test_depends +
            p.run_depends),
        exclude_function=lambda p: []
    )


def get_recursive_run_depends_in_workspace(packages, ordered_packages):
    """Calculates the recursive run depends of a set of packages which are also in the ordered_packages
    but excluding packages which are build depended on by another package in the list

    :param packages: packages for which the recursive depends should be calculated
    :type packages: list(:py:class:`catkin_pkg.package.Package`)
    :param ordered_packages: packages in the workspace, ordered topologically
    :type ordered_packages: list(tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive run depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """

    return get_recursive_depends_in_workspace(
        packages,
        ordered_packages,
        include_function=lambda p: p.run_depends,
        exclude_function=lambda p: p.buildtool_depends + p.build_depends
    )


def get_recursive_build_dependents_in_workspace(package_name, ordered_packages):
    """Calculates the recursive build dependents of a package which are also in
    the ordered_packages

    :param package_name: name of the package for which the recursive depends should be calculated
    :type package_name: str
    :param ordered_packages: packages in the workspace, ordered topologically
    :type ordered_packages: list(tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive build depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """
    recursive_dependents = list()

    for pth, pkg in reversed(ordered_packages):
        # Break if this is one to check
        if pkg.name == package_name:
            break

        # Check if this package depends on the target package
        deps = get_cached_recursive_build_depends_in_workspace(pkg, ordered_packages)
        deps_names = [p.name for _, p in deps]
        if package_name in deps_names:
            recursive_dependents.insert(0, (pth, pkg))

    return recursive_dependents


def get_recursive_run_dependents_in_workspace(package_name, ordered_packages):
    """Calculates the recursive run dependents of a package which are also in
    the ordered_packages

    :param package_name: name of the package for which the recursive depends should be calculated
    :type package_name: str
    :param ordered_packages: packages in the workspace, ordered topologically
    :type ordered_packages: list(tuple(package path,
        :py:class:`catkin_pkg.package.Package`))
    :returns: list of package path, package object tuples which are the
        recursive run depends for the given package
    :rtype: list(tuple(package path, :py:class:`catkin_pkg.package.Package`))
    """
    recursive_dependents = list()

    for pth, pkg in reversed(ordered_packages):
        # Break if this is one to check
        if pkg.name == package_name:
            break

        # Check if this package depends on the target package
        deps = get_recursive_run_depends_in_workspace([pkg], ordered_packages)
        deps_names = [p.name for _, p in deps]
        if package_name in deps_names:
            recursive_dependents.insert(0, (pth, pkg))

    return recursive_dependents


def is_tty(stream):
    """Returns True if the given stream is a tty, else False"""
    return hasattr(stream, 'isatty') and stream.isatty()


unicode_error_printed = False
unicode_sanitizer = re.compile(r'[^\x00-\x7F]+')


def log(*args, **kwargs):
    """Wrapper for print, allowing for special handling where necessary"""
    global unicode_error_printed
    try:
        print(*args, **kwargs)
    except UnicodeEncodeError:
        # Strip unicode characters from string args
        sanitized_args = [unicode_sanitizer.sub('?', a)
                          if isinstance(a, str)
                          else a
                          for a in args]
        print(*sanitized_args, **kwargs)

        # Warn the user that
        if not unicode_error_printed:
            print('WARNING: Could not encode unicode characters. Please set the'
                  ' PYTHONIOENCODING environment variable to see complete output.'
                  ' (i.e. PYTHONIOENCODING=UTF-8)',
                  file=sys.stderr)
            unicode_error_printed = True


def terminal_width():
    """Returns the estimated width of the terminal"""
    return shutil.get_terminal_size().columns


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
        lookup_array = list(range(len(string)))
    lookup_array.append(len(string))
    if length > len(lookup_array):
        return string
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
    try:
        global wide_log_fn
        wide_log_fn(msg, **kwargs)
    except IOError:
        # This happens when someone ctrl-c's during a log message
        pass


def find_enclosing_package(search_start_path=None, ws_path=None, warnings=None, symlinks=True):
    """Get the package containing a specific directory.

    :param search_start_path: The path to crawl upward to find a package, CWD if None
    :param ws_path: The path at which the search should stop
    """

    search_path = search_start_path or getcwd(symlinks=symlinks)
    stop_path = ws_path or '/'
    child_path = '.'

    while search_path != stop_path:
        # Find packages under the search path
        try:
            pkgs = find_packages(search_path, warnings=warnings)
        except RuntimeError:
            return None

        # Check if the directory is a catkin package
        if child_path in pkgs:
            return pkgs[child_path].name

        # Update search path
        (search_path, child_path) = os.path.split(search_path)

    return None


def version_tuple(v):
    """Get an integer version tuple from a string."""
    return tuple(map(int, (str(v).split("."))))


def mkdir_p(path):
    """Equivalent to UNIX mkdir -p"""
    if os.path.exists(path):
        return
    try:
        return os.makedirs(path)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else:
            raise


def format_env_dict(environ):
    """Format an environment dict for printing to console similarly to `typeset` builtin."""

    return '\n'.join([
        'typeset -x {}={}'.format(k, cmd_quote(v))
        for k, v in environ.items()
    ])


def parse_env_str(environ_str):
    """Parse a quoted environment string generated by format_env_dict, or `typeset` builtin.

    environ_str must be encoded in utf-8
    """

    try:
        split_envs = [e.split('=', 1) for e in cmd_split(environ_str.decode())]
        return {
            e[0]: e[1] for e
            in split_envs
            if len(e) == 2
        }
    except ValueError:
        print('WARNING: Could not parse env string: `{}`'.format(environ_str),
              file=sys.stderr)
        raise


def expand_glob_package(pattern, all_workspace_packages):
    """Return all packages that match the pattern"""
    return [p for p in all_workspace_packages if fnmatch(p, pattern)]
