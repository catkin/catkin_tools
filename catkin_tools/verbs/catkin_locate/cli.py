# Copyright 2015 Open Source Robotics Foundation, Inc.
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
import sys

from catkin_pkg.package import InvalidPackage
from catkin_pkg.packages import find_packages

from catkin_tools.argument_parsing import add_context_args
from catkin_tools.common import find_enclosing_package
from catkin_tools.common import getcwd
from catkin_tools.context import Context
from catkin_tools.metadata import find_enclosing_workspace
from catkin_tools.terminal_color import ColorMapper

color_mapper = ColorMapper()
clr = color_mapper.clr


def prepare_arguments(parser):
    add_context_args(parser)  # Adds the --profile option, possibly other things.

    # Behavior
    behavior_group = parser.add_argument_group('Behavior')
    add = behavior_group.add_argument
    add('-e', '--existing-only', action='store_true',
        help="Only print paths to existing directories.")
    add('-r', '--relative', action='store_true',
        help="Print relative paths instead of the absolute paths.")
    add('-q', '--quiet', action='store_true',
        help="Suppress warning output.")

    # Path options
    spaces_group = parser.add_argument_group(
        'Sub-Space Options',
        'Get the absolute path to one of the following locations in the given '
        'workspace with the given profile.')
    Context.setup_space_keys()
    add = spaces_group.add_mutually_exclusive_group().add_argument
    for space, space_dict in Context.SPACES.items():
        flags = [space_dict['short_flag']] if 'short_flag' in space_dict else []
        flags.append('--{}'.format(space_dict['default']))
        flags.append('--{}-space'.format(space))
        add(*flags, dest='space', action='store_const', const=space,
            help='Get the path to the {} space.'.format(space))

    pkg_group = parser.add_argument_group(
        'Package Directories',
        "Get the absolute path to package directories in the given workspace "
        "and sub-space. By default this will output paths in the workspace's "
        "source space. If the -b (--build) flag is given, it will output the "
        "path to the package's build directory. If the -d or -i (--devel or "
        "--install) flags are given, it will output the path to the package's "
        "share directory in that space. If no package is provided, the base "
        "space paths are printed, e.g. `catkin locate -s` might return "
        "`/path/to/ws/src` and `catkin locate -s foo` might return "
        "`/path/to/ws/src/foo`.")
    pkg_group_mut = pkg_group.add_mutually_exclusive_group()
    add = pkg_group_mut.add_argument
    add('package', metavar='PACKAGE', nargs='?',
        help="The name of a package to locate.")
    add('--this', action="store_true",
        help="Locate package containing current working directory.")

    special_group = parser.add_argument_group(
        'Special Directories',
        'Get the absolute path to a special catkin location')
    add = special_group.add_argument
    add('--shell-verbs', action='store_true',
        help="Get the path to the shell verbs script.")
    add('--examples', action='store_true',
        help="Get the path to the examples directory.")

    return parser


def main(opts):
    # Initialize dictionary version of opts namespace
    opts_vars = vars(opts) if opts else {}

    # Check for special locations
    root_resource_path = os.path.join(os.path.dirname(__file__), '..', '..')
    if opts.shell_verbs:
        shell_verbs = os.path.join(root_resource_path, 'verbs', 'catkin_shell_verbs.bash')
        print(os.path.normpath(shell_verbs))
        sys.exit(0)
    elif opts.examples:
        shell_verbs = os.path.join(root_resource_path, '..', 'docs', 'examples')
        print(os.path.normpath(shell_verbs))
        sys.exit(0)

    # Get the workspace (either the given directory or the enclosing ws)
    workspace_hint = opts_vars.get('workspace', None) or getcwd()
    workspace = find_enclosing_workspace(workspace_hint)

    if not workspace:
        if not opts.quiet:
            print(clr("[locate] @!@{rf}Error:@| No workspace found containing '{}'")
                  .format(workspace_hint), file=sys.stderr)
        sys.exit(1)

    # Load the context to get the subspaces
    ctx = Context.load(workspace, opts.profile, opts, load_env=False)

    path = None

    if opts.space:
        path = getattr(ctx, "{}_space_abs".format(opts.space))

    package = None
    if opts.package or opts.this:
        if opts.this:
            try:
                package = find_enclosing_package(
                    search_start_path=getcwd(),
                    ws_path=ctx.workspace,
                    warnings=[])
                if package is None:
                    sys.exit(clr("[locate] @!@{rf}Error:@| Passed '--this' but could not determine enclosing package. "
                                 "Is '{}' in a package in '{}' workspace?").format(getcwd(), ctx.workspace))
            except InvalidPackage as ex:
                sys.exit(clr("[locate] @!@{rf}Error:@| The file {} is an invalid package.xml file."
                             " See below for details:\n\n{}").format(ex.package_path, ex.msg))
        else:
            package = opts.package
        # Get the path to the given package
        path = path or ctx.source_space_abs
        if not opts.space or opts.space == 'source':
            try:
                packages = find_packages(path, warnings=[])
                catkin_package = [pkg_path for pkg_path, p in packages.items() if p.name == package]
                if catkin_package:
                    path = os.path.join(path, catkin_package[0])
                else:
                    sys.exit(clr("[locate] @!@{rf}Error:@| Could not locate a package named '{}' in path '{}'").format(
                                 package, path))
            except RuntimeError as e:
                sys.exit(clr('[locate] @!@{rf}Error:@| {}').format(str(e)))
        elif opts.space in ['devel', 'install']:
            path = os.path.join(path, 'share', package)
        else:
            path = os.path.join(path, package)

    if not opts.space and package is None:
        # Get the path to the workspace root
        path = workspace

    # Check if the path exists
    if opts.existing_only and not os.path.exists(path):
        sys.exit(clr("[locate] @!@{rf}Error:@| Requested path '{}' does not exist.").format(path))

    # Make the path relative if desired
    if opts.relative:
        path = os.path.relpath(path, getcwd())

    # Print the path
    print(path)
