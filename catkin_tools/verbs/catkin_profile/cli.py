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

from catkin_tools.context import Context

from catkin_tools.metadata import get_active_profile
from catkin_tools.metadata import get_profile_names
from catkin_tools.metadata import remove_profile
from catkin_tools.metadata import set_active_profile
from catkin_tools.metadata import DEFAULT_PROFILE_NAME

from catkin_tools.terminal_color import ColorMapper

color_mapper = ColorMapper()
clr = color_mapper.clr


def prepare_arguments(parser):

    subparsers = parser.add_subparsers(dest='subcommand', help='sub-command help')
    parser_list = subparsers.add_parser('list', help='List the available profiles.')
    parser_set = subparsers.add_parser('set', help='Set the active profile by name.')
    parser_add = subparsers.add_parser('add', help='Add a new profile by name.')
    parser_rename = subparsers.add_parser('rename', help='Rename a given profile.')
    parser_remove = subparsers.add_parser('remove', help='Remove a profile by name.')

    add = parser.add_argument
    add('--workspace', '-w', default=None,
        help='The path to the catkin workspace. Default: current working directory')

    add = parser_list.add_argument
    add('--unformatted', '-u', default=None, action='store_true',
        help='Print profile list without punctuation and additional details.')

    add = parser_list.add_argument
    add('--active', default=None, action='store_true',
        help='Print only active profile.')

    add = parser_set.add_argument
    add('name', type=str,
        help='The profile to activate.')

    add = parser_add.add_argument
    add('name', type=str,
        help='The new profile name.')
    add('-f', '--force', action='store_true', default=False,
        help="Overwrite an existing profile.")
    copy_group = parser_add.add_mutually_exclusive_group()
    add = copy_group.add_argument
    add('--copy', metavar='BASE_PROFILE', type=str,
        help="Copy the settings from an existing profile. (default: None)")
    add('--copy-active', action='store_true', default=False,
        help="Copy the settings from the active profile.")
    add('--extend', metavar='PARENT_PROFILE', type=str,
        help="Extend another profile")

    add = parser_rename.add_argument
    add('current_name', type=str,
        help='The current name of the profile to be renamed.')
    add('new_name', type=str,
        help='The new name for the profile.')
    add('-f', '--force', action='store_true', default=False,
        help="Overwrite an existing profile.")

    add = parser_remove.add_argument
    add('name', nargs='*',
        help='One or more profile names to remove.')

    return parser


def list_profiles(profiles, active_profile, unformatted=False, active=False):

    entry_format = '@{pf}-@| @{cf}%s@|' if not unformatted else '%s'
    entry_active_format = entry_format + (' (@{yf}active@|)' if not unformatted else '')

    ret = []
    if len(profiles) > 0:
        if not active:
            if not unformatted:
                ret += [clr('[profile] Available profiles:')]
            for p in profiles:
                if p == active_profile:
                    ret += [clr(entry_active_format % p)]
                else:
                    ret += [clr(entry_format % p)]
        else:
            if not unformatted:
                ret += [clr(entry_active_format % active_profile)]
            else:
                ret += [active_profile]
    else:
        if not unformatted:
            ret += [clr(
                '[profile] This workspace has no metadata profiles. Any '
                'configuration settings will automatically by applied to a new '
                'profile called `default`.')]

    return '\n'.join(ret)


def main(opts):
    try:
        # Load a context with initialization
        ctx = Context.load(opts.workspace, load_env=False)

        if not ctx.initialized():
            print("A catkin workspace must be initialized before profiles can be managed.")
            return 1

        profiles = get_profile_names(ctx.workspace)
        active_profile = get_active_profile(ctx.workspace)

        if opts.subcommand == 'list':
            print(list_profiles(profiles, active_profile, unformatted=opts.unformatted, active=opts.active))

        elif opts.subcommand == 'add':
            if opts.name in profiles:
                if opts.force:
                    print(clr('[profile] @{yf}Warning:@| Overwriting existing profile named @{cf}%s@|' % (opts.name)))
                else:
                    print(clr('catkin profile: error: A profile named '
                              '@{cf}%s@| already exists. Use `--force` to '
                              'overwrite.' % (opts.name)))
                    return 1
            if opts.copy_active:
                ctx.profile = opts.name
                Context.save(ctx)
                print(clr('[profile] Created a new profile named @{cf}%s@| '
                          'based on active profile @{cf}%s@|' % (opts.name, active_profile)))
            elif opts.copy:
                if opts.copy in profiles:
                    new_ctx = Context.load(opts.workspace, profile=opts.copy)
                    new_ctx.profile = opts.name
                    Context.save(new_ctx)
                    print(clr('[profile] Created a new profile named @{cf}%s@| '
                              'based on profile @{cf}%s@|' % (opts.name, opts.copy)))
                else:
                    print(clr('[profile] @{rf}A profile with this name does not exist: %s@|' % opts.copy))
            elif opts.extend:
                if opts.extend in profiles:
                    new_ctx = Context(workspace=ctx.workspace, profile=opts.name, extends=opts.extend)
                    Context.save(new_ctx)
                    print(clr('[profile] Created a new profile named @{cf}%s@| '
                              'extending profile @{cf}%s@|' % (opts.name, opts.extend)))
            else:
                new_ctx = Context(workspace=ctx.workspace, profile=opts.name)
                Context.save(new_ctx)
                print(clr('[profile] Created a new profile named @{cf}%s@| with default settings.' % (opts.name)))

            profiles = get_profile_names(ctx.workspace)
            active_profile = get_active_profile(ctx.workspace)
            print(list_profiles(profiles, active_profile))

        elif opts.subcommand == 'set':
            if opts.name in profiles:
                set_active_profile(ctx.workspace, opts.name)

                active_profile = get_active_profile(ctx.workspace)
                print(clr('[profile] Activated catkin metadata profile: @{cf}%s@|' % active_profile))
            else:
                print('catkin profile: error: Profile `%s` does not exist in workspace `%s`.' %
                      (opts.name, ctx.workspace))
                return 1

            profiles = get_profile_names(ctx.workspace)
            active_profile = get_active_profile(ctx.workspace)
            print(list_profiles(profiles, active_profile))

        elif opts.subcommand == 'rename':
            if opts.current_name in profiles:
                if opts.new_name in profiles:
                    if opts.force:
                        print(clr('[profile] @{yf}Warning:@| Overwriting '
                                  'existing profile named @{cf}%s@|' % (opts.new_name)))
                    else:
                        print(clr('catkin profile: error: A profile named '
                                  '@{cf}%s@| already exists. Use `--force` to '
                                  'overwrite.' % (opts.new_name)))
                        return 1
                ctx.profile = opts.new_name
                Context.save(ctx)
                remove_profile(ctx.workspace, opts.current_name)
                if opts.current_name == active_profile:
                    set_active_profile(ctx.workspace, opts.new_name)
                print(clr('[profile] Renamed profile @{cf}%s@| to @{cf}%s@|' % (opts.current_name, opts.new_name)))
            else:
                print('catkin profile: error: Profile `%s` does not exist in workspace `%s`.' %
                      (opts.current_name, ctx.workspace))
                return 1

            profiles = get_profile_names(ctx.workspace)
            active_profile = get_active_profile(ctx.workspace)
            print(list_profiles(profiles, active_profile))

        elif opts.subcommand == 'remove':
            for name in opts.name:
                if name == active_profile:
                    print('Profile `%s` is currently active. Re-setting active profile to `%s`.'
                          % (name, DEFAULT_PROFILE_NAME))
                    set_active_profile(ctx.workspace, DEFAULT_PROFILE_NAME)

                if name in profiles:
                    remove_profile(ctx.workspace, name)
                else:
                    print('catkin profile: error: Profile `%s` does not exist in workspace `%s`.' %
                          (name, ctx.workspace))
                    return 1

                print(clr('[profile] Removed profile: @{rf}%s@|' % name))
            profiles = get_profile_names(ctx.workspace)
            active_profile = get_active_profile(ctx.workspace)
            print(list_profiles(profiles, active_profile))

    except IOError as exc:
        # Usually happens if workspace is already underneath another catkin_tools workspace
        print('error: could not %s catkin profile: %s' % (opts.subcommand, exc.message))
        return 1

    return 0
