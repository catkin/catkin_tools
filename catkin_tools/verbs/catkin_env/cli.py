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

import os
import re
import sys

from osrf_pycommon.process_utils import execute_process

from catkin_tools.common import format_env_dict
from catkin_tools.common import parse_env_str


def prepare_arguments(parser):

    add = parser.add_argument

    add('-i', '--ignore-environment', default=False, action='store_true',
        help='Start with an empty environment.')
    add('-s', '--stdin', default=False, action='store_true',
        help='Read environment variable definitions from stdin. '
             'Variables should be given in NAME=VALUE format. ')

    add('envs_', metavar='NAME=VALUE', nargs='*', type=str, default=[],
        help='Explicitly set environment variables for the subcommand. '
             'These override variables given to stdin.')

    cmd_group = parser.add_argument_group('command')
    add = cmd_group.add_argument
    add('cmd_', metavar='COMMAND', nargs='?', type=str, default=None,
        help='Command to run. If omitted, the environment is printed to stdout.')
    add('args_', metavar='ARG', nargs='*', type=str, default=[],
        help='Arguments to the command.')

    return parser


def argument_preprocessor(args):
    """This preprocessor extracts environment variables and the raw subcommand
    to be executed by the env verb.

    :param args: system arguments from which special arguments need to be extracted
    :type args: list
    :returns: a tuple contianing a list of the arguments which can be handled
    by argparse and a dict of the extra arguments which this function has
    extracted
    :rtype: tuple
    """

    raw_args = list(sys.argv[1:]) if args is None else args
    args = []

    # Get leading optional arguments
    for arg in list(raw_args):
        if arg.startswith('-'):
            args.append(arg)
            raw_args.pop(0)
        else:
            # Done parsing options
            break

    # Get envs
    envs = {}
    for arg in list(raw_args):
        env_match = re.match('(.+?)=(.+)', arg)
        if env_match is not None and len(env_match.groups()) == 2:
            envs.update(dict([env_match.groups()]))
            raw_args.pop(0)
        else:
            # Done parsing envs
            break

    # Get command
    cmd = raw_args

    extras = {
        'envs': envs,
        'cmd': cmd,
    }

    return args, extras


def main(opts):

    environ = {}

    # Get the initial environment
    if not opts.ignore_environment:
        environ.update(os.environ)

    # Update environment from stdin
    if opts.stdin:
        input_env_str = sys.stdin.read()
        environ.update(parse_env_str(input_env_str.encode()))

    # Finally, update with explicit vars
    environ.update(opts.envs)

    if len(opts.cmd) == 0:
        # Print environment and exit if there's no command
        print(format_env_dict(environ))
    else:
        # Run the subcommand with the modified environment
        for ret in execute_process(opts.cmd, env=environ, emulate_tty=True):
            if ret:
                if isinstance(ret, int):
                    return ret
                else:
                    print(ret.decode(), end='')

    # Flush stdout
    # NOTE: This is done to ensure that automated use of this tool doesn't miss
    # the output
    sys.stdout.flush()
    return 0
