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

import argparse
import os
import pkg_resources
import sys
from datetime import date
from shlex import quote as cmd_quote

from catkin_tools.common import is_tty

from catkin_tools.config import get_verb_aliases
from catkin_tools.config import initialize_config

from catkin_tools.terminal_color import fmt
from catkin_tools.terminal_color import set_color
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
    verbs = sorted(verbs)
    verb_array_str = '[' + ' | '.join(verbs) + ']'
    verb_list_str = 'Call `catkin VERB -h` for help on each verb listed below:\n'
    for verb in verbs:
        desc = load_verb_description(verb)
        verb_list_str += '\n  %s\t%s' % (desc['verb'], desc['description'])

    subparser = parser.add_subparsers(
        title='catkin command',
        metavar=verb_array_str,
        description=verb_list_str,
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


def expand_one_verb_alias(sysargs, verb_aliases, used_aliases):
    """Iterate through sysargs looking for expandable verb aliases.

    When a verb alias is found, sysargs is modified to effectively expand the alias.
    The alias is removed from verb_aliases and added to used_aliases.
    After finding and expanding an alias, this function returns True.
    If no alias is found to be expanded, this function returns False.
    """
    cmd = os.path.basename(sys.argv[0])
    for index, arg in enumerate(sysargs):
        if arg.startswith('-'):
            # Not a verb, continue through the arguments
            continue
        if arg in used_aliases:
            print(fmt(
                "@!@{gf}==>@| Expanding alias '@!@{yf}" + arg +
                "@|' was previously expanded, ignoring this time to prevent infinite recursion."
            ))
        if arg in verb_aliases:
            before = [] if index == 0 else sysargs[:index - 1]
            after = [] if index == len(sysargs) else sysargs[index + 1:]
            sysargs[:] = before + verb_aliases[arg] + after
            print(fmt(
                "@!@{gf}==>@| Expanding alias "
                "'@!@{yf}{alias}@|' "
                "from '@{yf}{before} @!{alias}@{boldoff}{after}@|' "
                "to '@{yf}{before} @!{expansion}@{boldoff}{after}@|'"
            ).format(
                alias=arg,
                expansion=' '.join([cmd_quote(aarg) for aarg in verb_aliases[arg]]),
                before=' '.join([cmd] + before),
                after=(' '.join([''] + after) if after else '')
            ))
            # Prevent the alias from being used again, to prevent infinite recursion
            used_aliases.append(arg)
            del verb_aliases[arg]
            # Return True since one has been found
            return True
        # Return False since no verb alias was found
        return False


def expand_verb_aliases(sysargs, verb_aliases):
    """Expands aliases in sysargs which are found in verb_aliases until none are found."""
    used_aliases = []
    while expand_one_verb_alias(sysargs, verb_aliases, used_aliases):
        pass
    return sysargs


def catkin_main(sysargs):
    # Initialize config
    try:
        initialize_config()
    except RuntimeError as exc:
        sys.exit("Failed to initialize config: {0}".format(exc))

    # Create a top level parser
    parser = argparse.ArgumentParser(
        description="catkin command", formatter_class=argparse.RawDescriptionHelpFormatter)
    add = parser.add_argument
    add('-a', '--list-aliases', action="store_true", default=False,
        help="Lists the current verb aliases and then quits, all other arguments are ignored")
    add('--test-colors', action='store_true', default=False,
        help="Prints a color test pattern to the screen and then quits, all other arguments are ignored")
    add('--version', action='store_true', default=False,
        help="Prints the catkin_tools version.")
    color_control_group = parser.add_mutually_exclusive_group()
    add = color_control_group.add_argument
    add('--force-color', action='store_true', default=False,
        help='Forces catkin to output in color, even when the terminal does not appear to support it.')
    add('--no-color', action='store_true', default=False,
        help='Forces catkin to not use color in the output, regardless of the detect terminal type.')

    # Deprecated, moved to `catkin locate --shell-verbs
    add('--locate-extra-shell-verbs', action='store_true', help=argparse.SUPPRESS)

    # Generate a list of verbs available
    verbs = list_verbs()

    # Create the subparsers for each verb and collect the argument preprocessors
    argument_preprocessors = create_subparsers(parser, verbs)

    # Get verb aliases
    verb_aliases = get_verb_aliases()

    # Setup sysargs
    sysargs = sys.argv[1:] if sysargs is None else sysargs

    # Get colors config
    no_color = False
    force_color = os.environ.get('CATKIN_TOOLS_FORCE_COLOR', False)
    for arg in sysargs:
        if arg == '--no-color':
            no_color = True
        if arg == '--force-color':
            force_color = True

    if no_color or not force_color and not is_tty(sys.stdout):
        set_color(False)

    # Check for version
    if '--version' in sysargs:
        print('catkin_tools {} (C) 2014-{} Open Source Robotics Foundation'.format(
            pkg_resources.get_distribution('catkin_tools').version,
            date.today().year)
        )
        print('catkin_tools is released under the Apache License,'
              ' Version 2.0 (http://www.apache.org/licenses/LICENSE-2.0)')
        print('---')
        print('Using Python {}'.format(''.join(sys.version.split('\n'))))
        sys.exit(0)

    # Deprecated option
    if '--locate-extra-shell-verbs' in sysargs:
        print('Please use `catkin locate --shell-verbs` instead of `catkin --locate-extra-shell-verbs`',
              file=sys.stderr)
        sys.exit(0)

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
                print("{0}: {1}".format(alias, ' '.join([cmd_quote(aarg) for aarg in verb_aliases[alias]])))
            sys.exit(0)
        if not arg.startswith('-'):
            break

    # Do verb alias expansion
    sysargs = expand_verb_aliases(sysargs, verb_aliases)

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


def main(sysargs=None):
    try:
        catkin_main(sysargs)
    except KeyboardInterrupt:
        print('Interrupted by user!')
