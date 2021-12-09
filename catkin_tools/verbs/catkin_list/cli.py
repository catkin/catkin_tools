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

from catkin_tools.argument_parsing import add_context_args

from catkin_tools.context import Context

from catkin_tools.common import find_enclosing_package
from catkin_tools.common import get_recursive_build_dependents_in_workspace
from catkin_tools.common import get_recursive_build_depends_in_workspace
from catkin_tools.common import get_recursive_run_dependents_in_workspace
from catkin_tools.common import get_recursive_run_depends_in_workspace
from catkin_tools.common import getcwd

from catkin_pkg.packages import find_packages
from catkin_pkg.package import InvalidPackage
from catkin_pkg.topological_order import topological_order_packages

from catkin_tools.terminal_color import ColorMapper

color_mapper = ColorMapper()
clr = color_mapper.clr


def prepare_arguments(parser):

    add_context_args(parser)

    add = parser.add_argument

    information_group = parser.add_argument_group('Information', 'Control which information is shown.')
    group = information_group.add_mutually_exclusive_group()
    group.add_argument('--deps', '--dependencies', default=False, action='store_true',
                       help="Show direct dependencies of each package.")
    group.add_argument('--rdeps', '--recursive-dependencies', default=False, action='store_true',
                       help="Show recursive dependencies of each package.")

    packages_group = parser.add_argument_group('Packages', 'Control which packages are listed.')
    add = packages_group.add_argument
    add('--depends-on', nargs='*', metavar='PKG', default=[],
        help="Only show packages that directly depend on specific package(s).")
    add('--rdepends-on', '--recursive-depends-on', nargs='*', metavar='PKG', default=[],
        help="Only show packages that recursively depend on specific package(s).")
    add('--this', action='store_true',
        help="Show the package which contains the current working directory.")
    add('--directory', '-d', nargs='*', default=[],
        help="Pass list of directories process all packages in directory")

    behavior_group = parser.add_argument_group('Interface', 'The behavior of the command-line interface.')
    add = behavior_group.add_argument
    add('--quiet', default=False, action='store_true',
        help="Don't print out detected package warnings.")
    add('--unformatted', '-u', default=None, action='store_true',
        help='Print list without punctuation and additional details.')

    return parser


def main(opts):

    # Load the context
    ctx = Context.load(opts.workspace, opts.profile, load_env=False)

    if not ctx:
        sys.exit(clr("@{rf}ERROR: Could not determine workspace.@|"))

    if opts.directory:
        folders = opts.directory
    else:
        folders = [ctx.source_space_abs]

    list_entry_format = '@{pf}-@| @{cf}%s@|' if not opts.unformatted else '%s'

    opts.depends_on = set(opts.depends_on) if opts.depends_on else set()
    warnings = []
    for folder in folders:
        try:
            packages = find_packages(folder, warnings=warnings)
            ordered_packages = topological_order_packages(packages)
            if ordered_packages and ordered_packages[-1][0] is None:
                sys.exit(clr("@{rf}ERROR: Circular dependency within packages:@| "
                             + ordered_packages[-1][1]))
            packages_by_name = {pkg.name: (pth, pkg) for pth, pkg in ordered_packages}

            if opts.depends_on or opts.rdepends_on:

                dependents = set()

                for pth, pkg in ordered_packages:
                    is_dep = opts.depends_on.intersection([
                        p.name for p in pkg.build_depends + pkg.run_depends])
                    if is_dep:
                        dependents.add(pkg.name)

                for pth, pkg in [packages_by_name.get(n) for n in opts.rdepends_on]:
                    if pkg is None:
                        continue
                    rbd = get_recursive_build_dependents_in_workspace(pkg.name, ordered_packages)
                    rrd = get_recursive_run_dependents_in_workspace(pkg.name, ordered_packages)
                    dependents.update([p.name for _, p in rbd])
                    dependents.update([p.name for _, p in rrd])

                filtered_packages = [
                    (pth, pkg)
                    for pth, pkg in ordered_packages
                    if pkg.name in dependents]
            elif opts.this:
                this_package = find_enclosing_package(
                    search_start_path=getcwd(),
                    ws_path=ctx.workspace,
                    warnings=[])
                if this_package is None:
                    sys.exit(1)
                if this_package in packages_by_name:
                    filtered_packages = [packages_by_name[this_package]]
                else:
                    filtered_packages = []
            else:
                filtered_packages = ordered_packages

            for pkg_pth, pkg in filtered_packages:
                print(clr(list_entry_format % pkg.name))
                if opts.rdeps:
                    build_deps = [p for dp, p in get_recursive_build_depends_in_workspace(pkg, ordered_packages)]
                    run_deps = [p for dp, p in get_recursive_run_depends_in_workspace([pkg], ordered_packages)]
                else:
                    build_deps = [dep for dep in pkg.build_depends if dep.evaluated_condition]
                    run_deps = [dep for dep in pkg.run_depends if dep.evaluated_condition]

                if opts.deps or opts.rdeps:
                    if len(build_deps) > 0:
                        print(clr('  @{yf}build_depend:@|'))
                        for dep in build_deps:
                            print(clr('  @{pf}-@| %s' % dep.name))
                    if len(run_deps) > 0:
                        print(clr('  @{yf}run_depend:@|'))
                        for dep in run_deps:
                            print(clr('  @{pf}-@| %s' % dep.name))
        except InvalidPackage as ex:
            sys.exit(clr("@{rf}Error:@| The file %s is an invalid package.xml file."
                         " See below for details:\n\n%s" % (ex.package_path, ex.msg)))

    # Print out warnings
    if not opts.quiet:
        for warning in warnings:
            print(clr("@{yf}Warning:@| %s" % warning), file=sys.stderr)
