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
import pkg_resources
import sys

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


def default_parse_arguments(parser, args):
    return parser.parse_args(args)


def create_subparsers(parser, verbs):
    metavar = '[' + ' | '.join(verbs) + ']'
    subparser = parser.add_subparsers(
        title='catkin command',
        metavar=metavar,
        description='Call `catkin {0} -h` for help on a each verb.'.format(metavar),
        dest='verb'
    )
    for verb in verbs:
        desc = load_verb_description(verb)
        cmd_parser = subparser.add_parser(desc['title'], description=desc['description'])
        cmd_parser = desc['prepare_arguments'](cmd_parser)

        cmd_parser.set_defaults(func=desc['main'])
        # add_global_arguments(cmd_parser)


def main(sysargs=None):
    parser = argparse.ArgumentParser(description="catkin command")

    verbs = list_verbs()

    create_subparsers(parser, verbs)

    args = parser.parse_args(sysargs)

    # handle_global_arguments(args)

    sys.exit(args.func(args) or 0)
