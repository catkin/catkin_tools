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

import argparse
import os
import pkg_resources
import sys

from catkin_tools.config import get_verb_aliases
from catkin_tools.config import initialize_config

from catkin_tools.terminal_color import fmt
from catkin_tools.terminal_color import test_colors

CATKIN_COMMAND_VERB_GROUP = 'catkin_tools.commands.catkin.verbs'


def list_verbs():
    verbs = []
    for entry_point in pkg_resources.iter_entry_points(group=CATKIN_COMMAND_VERB_GROUP):
        verbs.append(entry_point.name)
    return verbs


def load_verb_description(verb_name):
    for entry_point in pkg_resources.iter_entry_points(group=CATKIN_COMMAND_VERB_GROUP):
        if entry_point.name == verb_name:
            return entry_point.load()


def default_argument_preprocessor(args):
    extras = {}
    return args, extras


def create_subparsers(parser, verbs):
    metavar = '[' + ' | '.join(verbs) + ']'
    subparser = parser.add_subparsers(
        title='catkin command',
        metavar=metavar,
        description='Call `catkin {0} -h` for help on a each verb.'.format(metavar),
        dest='verb'
    )

    argument_preprocessors = {}

    for verb in verbs:
        desc = load_verb_description(verb)
        cmd_parser = subparser.add_parser(desc['verb'], description=desc['description'])
        cmd_parser = desc['prepare_arguments'](cmd_parser)

        cmd_parser.set_defaults(main=desc['main'])

        if 'argument_preprocessor' in desc:
            argument_preprocessors[verb] = desc['argument_preprocessor']
        else:
            argument_preprocessors[verb] = default_argument_preprocessor

    return argument_preprocessors


def main(sysargs=None):
    # Initialize config
    try:
        initialize_config()
    except RuntimeError as exc:
        sys.exit("Failed to initialize config: {0}".format(exc))

    # Create a top level parser
    parser = argparse.ArgumentParser(description="catkin command")
    add = parser.add_argument
    add('-a', '--list-aliases', action="store_true", default=False,
        help="lists the current verb aliases and then quits, all other arguments are ignored")
    add('--test-colors', action='store_true', default=False,
        help="prints a color test pattern to the screen and then quits, all other arguments are ignored")

    # Generate a list of verbs available
    verbs = list_verbs()

    # Create the subparsers for each verb and collect the argument preprocessors
    argument_preprocessors = create_subparsers(parser, verbs)

    # Get verb aliases
    verb_aliases = get_verb_aliases()

    # Setup sysargs
    sysargs = sys.argv[1:] if sysargs is None else sysargs
    cmd = os.path.basename(sys.argv[0])

    # Check for --test-colors
    for arg in sysargs:
        if arg == '--test-colors':
            test_colors()
            sys.exit(0)
        if not arg.startswith('-'):
            break

    # Check for --list-aliases
    for arg in sysargs:
        if arg == '--list-aliases' or arg == '-a':
            for alias in sorted(list(verb_aliases.keys())):
                print("{0}: {1}".format(alias, verb_aliases[alias]))
            sys.exit(0)
        if not arg.startswith('-'):
            break

    # Do alias expansion
    expanding_verb_aliases = True
    used_aliases = []
    while expanding_verb_aliases:
        expanding_verb_aliases = False
        for index, arg in enumerate(sysargs):
            if not arg.startswith('-'):
                if arg in used_aliases:
                    print(fmt(
                        "@!@{gf}==>@| Expanding alias '@!@{yf}" +
                        arg +
                        "@|' was previously expanded, ignoring this time to prevent infinite recursion."
                    ))
                if arg in verb_aliases:
                    before = [] if index == 0 else sysargs[:index - 1]
                    after = [] if index == len(sysargs) else sysargs[index + 1:]
                    sysargs = before + verb_aliases[arg].split() + after
                    print(fmt(
                        "@!@{gf}==>@| Expanding alias "
                        "'@!@{yf}{alias}@|' "
                        "from '@{yf}{before} @!{alias}@{boldoff}{after}@|' "
                        "to '@{yf}{before} @!{expansion}@{boldoff}{after}@|'"
                    ).format(
                        alias=arg,
                        expansion=verb_aliases[arg],
                        before=' '.join([cmd] + before),
                        after=(' '.join([''] + after) if after else '')
                    ))
                    expanding_verb_aliases = True
                    # Prevent the alias from being used again, to prevent infinite recursion
                    used_aliases.append(arg)
                    del verb_aliases[arg]
                break

    # Determine the verb, splitting arguments into pre and post verb
    verb = None
    pre_verb_args = []
    post_verb_args = []
    for index, arg in enumerate(sysargs):
        # If the arg does not start with a `-` then it is a positional argument
        # The first positional argument must be the verb
        if not arg.startswith('-'):
            verb = arg
            post_verb_args = sysargs[index + 1:]
            break
        # If the `-h` or `--help` option comes before the verb, parse_args
        if arg in ['-h', '--help']:
            parser.parse_args(sysargs)
        # Otherwise it is a pre-verb option
        pre_verb_args.append(arg)

    # Error on no verb provided
    if verb is None:
        print(parser.format_usage())
        sys.exit("Error: No verb provided.")
    # Error on unknown verb provided
    if verb not in verbs:
        print(parser.format_usage())
        sys.exit("Error: Unknown verb '{0}' provided.".format(verb))

    # First allow the verb's argument preprocessor to strip any args
    # and return any "extra" information it wants as a dict
    processed_post_verb_args, extras = argument_preprocessors[verb](post_verb_args)
    # Then allow argparse to process the left over post-verb arguments along
    # with the pre-verb arguments and the verb itself
    args = parser.parse_args(pre_verb_args + [verb] + processed_post_verb_args)
    # Extend the argparse result with the extras from the preprocessor
    for key, value in extras.items():
        setattr(args, key, value)

    # Finally call the subparser's main function with the processed args
    # and the extras which the preprocessor may have returned
    sys.exit(args.main(args) or 0)
