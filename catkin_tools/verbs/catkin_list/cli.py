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
        help="One or more dependencies a package must have to be listed.")

    return parser


def main(opts):
    folders = opts.folders or [os.getcwd()]
    try:
        for folder in folders:
            for pkg_pth, pkg in find_packages(folder).items():
                build_depend_names = [d.name for d in pkg.build_depends]
                if not opts.depends_on or not [x for x in opts.depends_on if x not in build_depend_names]:
                    print(clr("@{pf}-@| @{cf}%s@|" % pkg.name))
                    if opts.deps:
                        if len(pkg.build_depends) > 0:
                            print(clr('  @{yf}build_depend:@|'))
                            for dep in pkg.build_depends:
                                print(clr('  @{pf}-@| %s' % dep.name))
                        if len(pkg.run_depends) > 0:
                            print(clr('  @{yf}run_depend:@|'))
                            for dep in pkg.run_depends:
                                print(clr('  @{pf}-@| %s' % dep.name))
    except InvalidPackage as ex:
        message = '\n'.join(ex.args)
        print("Error: The directory %s contains an invalid package. See below for details:\n\n%s" % (folder, message))
