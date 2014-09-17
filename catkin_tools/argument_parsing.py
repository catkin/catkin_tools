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

import os
import re
import sys

from multiprocessing import cpu_count

from catkin_tools.common import wide_log


def add_context_args(parser):
    """Add common workspace and profile args to an argparse parser.

    :param parser: The python argparse parser object (or subparser)
    :type parser: ArgumentParser
    """

    add = parser.add_argument
    add_workspace_arg(parser)
    add('--profile', default=None,
        help='The name of a config profile to use (default: active profile)')


def add_workspace_arg(parser):
    """Add common workspace arg to an argparse parser.

    :param parser: The python argparse parser object (or subparser)
    :type parser: ArgumentParser
    """

    add = parser.add_argument
    add('--workspace', '-w', default=None,
        help='The path to the catkin_tools workspace or a directory contained within it (default: ".")')


def add_cmake_and_make_and_catkin_make_args(parser):
    """Add common make and cmake args to an argparse parser.

    :param parser: The python argparse parser object (or subparser)
    :type parser: ArgumentParser
    """

    add = parser.add_argument
    add('--parallel-jobs', '--parallel', '-p', default=None,
        help='Maximum number of packages which could be built in parallel (default is cpu count)')

    add = parser.add_mutually_exclusive_group().add_argument
    add('--cmake-args', metavar='ARG', dest='cmake_args', nargs='+', required=False, type=str, default=None,
        help='Arbitrary arguments which are passes to CMake. '
             'It must be passed after other arguments since it collects all following options.')
    add('--no-cmake-args', dest='cmake_args', action='store_const', const='A', default=None,
        help='Pass no additional arguments to CMake.')

    add = parser.add_mutually_exclusive_group().add_argument
    add('--make-args', metavar='ARG', dest='make_args', nargs='+', required=False, type=str, default=None,
        help='Arbitrary arguments which are passes to make.'
             'It must be passed after other arguments since it collects all following options.')
    add('--no-make-args', dest='make_args', action='store_const', const=[], default=None,
        help='Pass no additional arguments to make (does not affect --catkin-make-args).')

    add = parser.add_mutually_exclusive_group().add_argument
    add('--catkin-make-args', metavar='ARG', dest='catkin_make_args', nargs='+', required=False, type=str, default=None,
        help='Arbitrary arguments which are passes to make but only for catkin packages.'
             'It must be passed after other arguments since it collects all following options.')
    add('--no-catkin-make-args', dest='catkin_make_args', action='store_const', const=[], default=None,
        help='Pass no additional arguments to make for catkin packages (does not affect --make-args).')


def _extract_cmake_and_make_arguments(args, extract_catkin_make):
    """Extract arguments which are meant to be passed to CMake and GNU Make
    through the catkin_tools command line interface.

    :param args: system arguments from which special arguments need to be extracted
    :type args: list
    :returns: tuple of separate args, cmake_args, make args, and catkin make args
    :rtype: tuple
    """
    cmake_args = []
    make_args = []
    catkin_make_args = []

    arg_types = {}

    if '--no-cmake-args' not in args:
        arg_types['--cmake-args'] = cmake_args
    if '--no-make-args' not in args:
        arg_types['--make-args'] = make_args

    if extract_catkin_make and '--no-catkin_make_args' not in args:
        arg_types['--catkin-make-args'] = catkin_make_args

    arg_indexes = {}
    for k in arg_types.keys():
        if k in args:
            arg_indexes[args.index(k)] = k

    def split_arguments(args, splitter_name):
        if splitter_name not in args:
            return args, None
        index = args.index(splitter_name)
        return args[0:index], args[index + 1:]

    for index in reversed(sorted(arg_indexes.keys())):
        arg_type = arg_indexes[index]
        args, specific_args = split_arguments(args, arg_type)
        arg_types[arg_type].extend(specific_args)

    # classify -D* and -G* arguments as cmake specific arguments
    if '--cmake-args' in arg_types:
        implicit_cmake_args = [a for a in args if a.startswith('-D') or a.startswith('-G')]
        args = [a for a in args if a not in implicit_cmake_args]
        cmake_args = implicit_cmake_args + cmake_args

    if '--no-cmake-args' not in args and len(cmake_args) == 0:
        cmake_args = None
    if '--no-make-args' not in args and len(make_args) == 0:
        make_args = None
    if extract_catkin_make and '--no-catkin_make_args' not in args and len(catkin_make_args) == 0:
        catkin_make_args = None

    return args, cmake_args, make_args, catkin_make_args


def extract_cmake_and_make_and_catkin_make_arguments(args):
    """Extracts cmake, make, and catkin specific make arguments from given system arguments

    :param args: system arguments from which special arguments need to be extracted
    :type args: list
    :returns: tuple of separate args, cmake_args, make args, and catkin make args
    :rtype: tuple
    """
    return _extract_cmake_and_make_arguments(args, extract_catkin_make=True)


def extract_cmake_and_make_arguments(args):
    """Extracts cmake and make arguments from the given system arguments

    :param args: system arguments from which special arguments need to be extracted
    :type args: list
    :returns: tuple of separate args, cmake_args, and make_args
    :rtype: tuple
    """
    args, cmake_args, make_args, _ = _extract_cmake_and_make_arguments(args, extract_catkin_make=False)
    return args, cmake_args, make_args


def extract_jobs_flags(mflags):
    """Extracts make job flags from a list of other make flags, i.e. -j8 -l8

    :param mflags: string of space separated make arguments
    :type mflags: str
    :returns: space separated list of make jobs flags
    :rtype: str
    """
    regex = r'(?:^|\s)(-?(?:j|l)(?:\s*[0-9]+|\s|$))' + \
            r'|' + \
            r'(?:^|\s)((?:--)?(?:jobs|load-average)(?:(?:=|\s+)[0-9]+|(?:\s|$)))'
    matches = re.findall(regex, mflags) or []
    matches = [m[0] or m[1] for m in matches]
    return ' '.join([m.strip() for m in matches]) if matches else None


def handle_make_arguments(input_make_args, force_single_threaded_when_running_tests=False):
    """Special handling for make arguments.

    If force_single_threaded_when_running_tests is True, jobs flags are
    replaced with -j1, because tests cannot handle parallelization.

    If no job flags are present and there are none in the MAKEFLAGS environment
    variable, then make flags are set to the cpu_count, e.g. -j4 -l4.

    :param input_make_args: list of make arguments to be handled
    :type input_make_args: list
    :param force_single_threaded_when_running_tests: self explanatory
    :type force_single_threaded_when_running_tests: bool
    :returns: copied list of make arguments, potentially with some modifications
    :rtype: list
    """
    make_args = list(input_make_args)

    if force_single_threaded_when_running_tests:
        # force single threaded execution when running test since rostest does not support multiple parallel runs
        run_tests = [a for a in make_args if a.startswith('run_tests')]
        if run_tests:
            wide_log('Forcing "-j1" for running unit tests.')
            make_args.append('-j1')

    # If no -j/--jobs/-l/--load-average flags are in make_args
    if not extract_jobs_flags(' '.join(make_args)):
        # If -j/--jobs/-l/--load-average are in MAKEFLAGS
        if 'MAKEFLAGS' in os.environ and extract_jobs_flags(os.environ['MAKEFLAGS']):
            # Do not extend make arguments, let MAKEFLAGS set things
            pass
        else:
            # Else extend the make_arguments to include some jobs flags
            # Use the number of CPU cores
            try:
                jobs = cpu_count()
                make_args.append('-j{0}'.format(jobs))
                make_args.append('-l{0}'.format(jobs))
            except NotImplementedError:
                # If the number of cores cannot be determined, do not extend args
                pass
    return make_args


def argument_preprocessor(args):
    """Perform processing of argument patterns which are not captured by
    argparse, before being passed to argparse

    :param args: system arguments from which special arguments need to be extracted
    :type args: list
    :returns: a tuple contianing a list of the arguments which can be handled
    by argparse and a dict of the extra arguments which this function has
    extracted
    :rtype: tuple
    """
    # CMake/make pass-through flags collect dashed options. They require special
    # handling or argparse will complain about unrecognized options.
    # NOTE: http://bugs.python.org/issue9334
    args = sys.argv[1:] if args is None else args
    extract_make_args = extract_cmake_and_make_and_catkin_make_arguments
    args, cmake_args, make_args, catkin_make_args = extract_make_args(args)
    # Extract make jobs flags.
    jobs_flags = extract_jobs_flags(' '.join(args))
    if jobs_flags:
        args = re.sub(jobs_flags, '', ' '.join(args)).split()
        jobs_flags = jobs_flags.split()

    extras = {
        'cmake_args': cmake_args,
        'make_args': make_args + jobs_flags if make_args else jobs_flags if jobs_flags else make_args,
        'catkin_make_args': catkin_make_args,
    }
    return args, extras
