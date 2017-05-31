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

import sys
from os import path

try:
    from catkin_pkg.packages import find_packages
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin_tools.argument_parsing import add_context_args
from catkin_tools.context import Context

from .fetcher.dependency_parser import Parser
from .fetcher.downloader import Downloader


def prepare_arguments(parser):
    parser.description = """\
Download dependencies for one or more packages in a catkin workspace. This
reads dependencies from package.xml file of each of the packages in the
workspace and tries to download their sources from version control system of
choice.\
"""
    # Workspace / profile args
    add_context_args(parser)

    # What packages to build
    pkg_group = parser.add_argument_group('Packages',
                                          'Control which packages get built.')
    add = pkg_group.add_argument
    add('packages', metavar='PKGNAME', nargs='*',
        help='Packages for which the dependencies will be downloaded. '
             'If no packages are given, all dependencies are downloaded.')

    # Behavior
    behavior_group = parser.add_argument_group(
        'Interface', 'The behavior of the command-line interface.')
    add = behavior_group.add_argument
    add('--verbose', '-v', action='store_true', default=False,
        help='Print output from commands.')

    return parser


def main(opts):
    # Load the context
    context = Context.load(opts.workspace, opts.profile, opts, append=True)
    workspace_packages = find_packages(context.source_space_abs,
                                       exclude_subspaces=True, warnings=[])
    # TODO: get rid of this hardcoded path, add this as parameter or even
    # setting. Should be possible to store this per workspace.
    default_url = "git@gitlab.ipb.uni-bonn.de:ipb-tools/{package}.git"
    if not opts.workspace:
        print("Please define workspace!")
        return 1
    fetch_all = False
    if len(opts.packages) == 0:
        fetch_all = True

    available_pkgs = [pkg.name for _, pkg in workspace_packages.items()]
    if len(available_pkgs) == 0:
        print("There are no packages in the workspace.")

    for package_path, package in workspace_packages.items():
        if fetch_all or (package.name in opts.packages):
            parser = Parser(default_url, package.name)
            ws_path = path.join(opts.workspace, 'src')
            package_folder = path.join(ws_path, package_path)
            downloader = Downloader(ws_path, available_pkgs)
            dep_dict = parser.get_dependencies(package_folder)
            downloader.download_dependencies(dep_dict)
