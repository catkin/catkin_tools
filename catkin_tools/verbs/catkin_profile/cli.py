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

from catkin_pkg.packages import find_packages
from catkin_tools import metadata


def prepare_arguments(parser):

    subparsers = parser.add_subparsers(dest='subcommand', help='sub-command help')
    parser_info = subparsers.add_parser('info', help='Get information about the active and available profiles.')
    parser_set = subparsers.add_parser('set', help='Set the active profile by name.')
    parser_remove = subparsers.add_parser('remove', help='Remove a profile by name.')

    add = parser.add_argument
    info_add = parser_info.add_argument
    set_add = parser_set.add_argument
    rm_add = parser_remove.add_argument

    add('--workspace', '-w', default=None,
        help='The path to the catkin workspace. Default: current working directory')

    info_add('name', nargs='?', default='',
             help='The profile to describe. Default: active profile')
    set_add('name', nargs='?', default='default',
            help='The profile name. Default: "default"')
    set_add('--reset', action='store_true', default=False,
            help='Reset (delete) the metadata for the given profile.')

    rm_add('name', nargs='?', default='default',
           help='The profile name. Default: "default"')

    return parser


def main(opts):
    try:
        # Check if the workspace is initialized
        if opts.workspace is None:
            opts.workspace = os.getcwd()
        marked_workspace = metadata.find_enclosing_workspace(opts.workspace)

        if marked_workspace is None:
            print("A catkin workspace must be initialized before profiles can be managed.")
            return 1

        profiles = metadata.get_profile_names(marked_workspace)
        active_profile = metadata.get_active_profile(marked_workspace)

        if opts.subcommand == 'info':

            if opts.name == '':
                profile = active_profile
            else:
                profile = opts.name

            if profile != active_profile and profile not in profiles:
                print('# Profile `%s` does not exist.' % profile)
            else:
                print('# Information for profile: `%s`' % profile)

                if profile == active_profile:
                    print('#  Profile is ACTIVE')
                else:
                    print('#  Profile is INACTIVE')

                if profile not in profiles:
                    print('#  Profile has not been initialized, it contains no information.')
                else:
                    print('#  Metadata stored for the following catkin verbs:')
                    # TODO: Add metadata output

            if len(profiles) > 0:
                print('#\n# Available profiles:')
                for p in profiles:
                    print('#  `%s`' % p)
            else:
                print('#\n# There are no currently initialized metadata profiles.')

        elif opts.subcommand == 'set':
            metadata.set_active_profile(marked_workspace, opts.name)
            if opts.reset:
                if opts.name in profiles:
                    metadata.remove_profile(marked_workspace, opts.name)

            active_profile = metadata.get_active_profile(marked_workspace)
            print('Active catkin metadata profile: `%s`' % active_profile)
        elif opts.subcommand == 'remove':
            if opts.name == active_profile:
                print('Profile `%s` is currently active. Re-setting active profile to `%s`.' 
                      % (opts.name, metadata.DEFAULT_PROFILE_NAME))
                metadata.set_active(marked_workspace, DEFAULT_PROFILE_NAME)

            if opts.name in profiles:
                metadata.remove_profile(marked_workspace, opts.name)
            else:
                raise IOError('Profile `%s` does not exist in workspace `%s`.' % (opts.name, marked_workspace))

            print('Removed profile: `%s`' % opts.name)

    except IOError as exc:
        # Usually happens if workspace is already underneath another catkin_tools workspace
        print('error: could not %s catkin profile: %s' % (opts.subcommand, exc.message))
        return 1

    return 0
