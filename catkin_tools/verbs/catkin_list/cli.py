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
from catkin_pkg.package import InvalidPackage

from catkin_tools.terminal_color import ColorMapper

color_mapper = ColorMapper()
clr = color_mapper.clr


def prepare_arguments(parser):
    add = parser.add_argument
    # What packages to build
    add('folders', nargs='*',
        help='Folders in which to find packages. (default: cwd)')
    add('--deps', '--dependencies', default=False, action='store_true',
        help="List dependencies of each package.")
    add('--depends-on', nargs='*',
        help="List all packages that depend on supplied argument package(s).")

    return parser


def main(opts):
    folders = opts.folders or [os.getcwd()]
    opts.depends_on = set(opts.depends_on) if opts.depends_on else set()
    try:
        for folder in folders:
            for pkg_pth, pkg in find_packages(folder).items():
                build_depend_names = [d.name for d in pkg.build_depends]
                is_build_dep = opts.depends_on.intersection(
                    build_depend_names)
                run_depend_names = [d.name for d in pkg.run_depends]
                is_run_dep = opts.depends_on.intersection(
                    run_depend_names)
                if not opts.depends_on or is_build_dep or is_run_dep:
                    print(clr("@{pf}-@| @{cf}%s@|" % pkg.name))
                    if opts.deps:
                        if build_depend_names:
                            print(clr('  @{yf}build_depend:@|'))
                            for dep in build_depend_names:
                                print(clr('  @{pf}-@| %s' % dep))
                        if run_depend_names:
                            print(clr('  @{yf}run_depend:@|'))
                            for dep in run_depend_names:
                                print(clr('  @{pf}-@| %s' % dep))
    except InvalidPackage as ex:
        message = '\n'.join(ex.args)
        print(clr("@{rf}Error:@| The directory %s contains an invalid package."
                  " See below for details:\n\n%s" % (folder, message)))
