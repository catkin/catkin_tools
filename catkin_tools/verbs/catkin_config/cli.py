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

from catkin_tools.argument_parsing import add_cmake_and_make_and_catkin_make_args
from catkin_tools.argument_parsing import add_context_args

from catkin_tools.context import Context

from catkin_tools.terminal_color import ColorMapper, sanitize

color_mapper = ColorMapper()
clr = color_mapper.clr


def prepare_arguments(parser):

    parser.description = "This verb is used to configure a catkin workspace's\
    configuration and layout. Calling `catkin config` with no arguments will\
    display the current config and affect no changes if a config already exists\
    for the current workspace and profile."

    # Workspace / profile args
    add_context_args(parser)

    behavior_group = parser.add_argument_group('Behavior', 'Options affecting argument handling.')
    add = behavior_group.add_mutually_exclusive_group().add_argument
    add('--append-args', '-a', action='store_true', default=False,
        help='For list-type arguments, append elements.')
    add('--remove-args', '-r', action='store_true', default=False,
        help='For list-type arguments, remove elements.')

    context_group = parser.add_argument_group('Workspace Context', 'Options affecting the context of the workspace.')
    add = context_group.add_argument
    add('--init', action='store_true', default=False,
        help='Initialize a workspace if it does not yet exist.')
    add = context_group.add_mutually_exclusive_group().add_argument
    add('--extend', '-e', dest='extend_path', type=str,
        help='Explicitly extend the result-space of another catkin workspace, '
        'overriding the value of $CMAKE_PREFIX_PATH.')
    add('--no-extend', dest='extend_path', action='store_const', const='',
        help='Un-set the explicit extension of another workspace as set by --extend.')
    add = context_group.add_argument
    add('--mkdirs', action='store_true', default=False,
        help='Create directories required by the configuration (e.g. source space) if they do not already exist.')

    create_group = parser.add_argument_group(
        'Package Create Defaults', 'Information of default authors/maintainers of created packages')
    add = create_group.add_mutually_exclusive_group().add_argument
    add('--authors', metavar=('NAME', 'EMAIL'), dest='authors', nargs='+', required=False, type=str, default=None,
        help='Set the default authors of created packages')
    add('--maintainers', metavar=('NAME', 'EMAIL'), dest='maintainers', nargs='+',
        required=False, type=str, default=None,
        help='Set the default maintainers of created packages')
    add('--licenses', metavar=('LICENSE'), dest='licenses', nargs='+', required=False, type=str, default=None,
        help='Set the default licenses of created packages')

    lists_group = parser.add_argument_group(
        'Package Build Defaults', 'Packages to include or exclude from default build behavior.')
    add = lists_group.add_mutually_exclusive_group().add_argument
    add('--whitelist', metavar="PKG", dest='whitelist', nargs="+", required=False, type=str, default=None,
        help='Set the packages on the whitelist. If the whitelist is non-empty, '
        'only the packages on the whitelist are built with a bare call to '
        '`catkin build`.')
    add('--no-whitelist', dest='whitelist', action='store_const', const=[], default=None,
        help='Clear all packages from the whitelist.')
    add = lists_group.add_mutually_exclusive_group().add_argument
    add('--blacklist', metavar="PKG", dest='blacklist', nargs="+", required=False, type=str, default=None,
        help='Set the packages on the blacklist. Packages on the blacklist are '
        'not built with a bare call to `catkin build`.')
    add('--no-blacklist', dest='blacklist', action='store_const', const=[], default=None,
        help='Clear all packages from the blacklist.')

    spaces_group = parser.add_argument_group('Spaces', 'Location of parts of the catkin workspace.')
    Context.setup_space_keys()
    for space, space_dict in Context.SPACES.items():
        add = spaces_group.add_mutually_exclusive_group().add_argument
        flags = ['--{}-space'.format(space)]
        flags.extend([space_dict['short_flag']] if 'short_flag' in space_dict else [])
        add(*flags, default=None,
            help='The path to the {} space.'.format(space))
        add('--default-{}-space'.format(space),
            action='store_const', dest='{}_space'.format(space), default=None, const=space_dict['default'],
            help='Use the default path to the {} space ("{}")'.format(space, space_dict['default']))
    add = spaces_group.add_argument
    add('-x', '--space-suffix',
        help='Suffix for build, devel, and install space if they are not otherwise explicitly set.')

    devel_group = parser.add_argument_group(
        'Devel Space', 'Options for configuring the structure of the devel space.')
    add = devel_group.add_mutually_exclusive_group().add_argument
    add('--link-devel', dest='devel_layout', action='store_const', const='linked', default=None,
        help='Build products from each catkin package into isolated spaces,'
        ' then symbolically link them into a merged devel space.')
    add('--merge-devel', dest='devel_layout', action='store_const', const='merged', default=None,
        help='Build products from each catkin package into a single merged devel spaces.')
    add('--isolate-devel', dest='devel_layout', action='store_const', const='isolated', default=None,
        help='Build products from each catkin package into isolated devel spaces.')

    install_group = parser.add_argument_group(
        'Install Space', 'Options for configuring the structure of the install space.')
    add = install_group.add_mutually_exclusive_group().add_argument
    add('--install', action='store_true', default=None,
        help='Causes each package to be installed to the install space.')
    add('--no-install', dest='install', action='store_false', default=None,
        help='Disables installing each package into the install space.')

    add = install_group.add_mutually_exclusive_group().add_argument
    add('--isolate-install', action='store_true', default=None,
        help='Install each catkin package into a separate install space.')
    add('--merge-install', dest='isolate_install', action='store_false', default=None,
        help='Install each catkin package into a single merged install space.')

    build_group = parser.add_argument_group('Build Options', 'Options for configuring the way packages are built.')
    add_cmake_and_make_and_catkin_make_args(build_group)

    return parser


def main(opts):
    try:
        # Determine if the user is trying to perform some action, in which
        # case, the workspace should be automatically initialized
        ignored_opts = ['main', 'verb']
        actions = [v for k, v in vars(opts).items() if k not in ignored_opts]
        no_action = not any(actions)

        # Try to find a metadata directory to get context defaults
        # Otherwise use the specified directory
        context = Context.load(
            opts.workspace,
            opts.profile,
            opts,
            append=opts.append_args,
            remove=opts.remove_args)

        do_init = opts.init or not no_action
        summary_notes = []

        if not context.initialized() and do_init:
            summary_notes.append(clr('@!@{cf}Initialized new catkin workspace in `%s`@|' % sanitize(context.workspace)))

        if context.initialized() or do_init:
            Context.save(context)

        if opts.mkdirs and not context.source_space_exists():
            os.makedirs(context.source_space_abs)

        print(context.summary(notes=summary_notes))

    except IOError as exc:
        # Usually happens if workspace is already underneath another catkin_tools workspace
        print('error: could not configure catkin workspace: %s' % exc.message)
        return 1

    return 0
