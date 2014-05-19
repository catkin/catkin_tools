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

import re
import sys
import time

from .color import clr
from .color import set_color

from .common import extract_cmake_and_make_and_catkin_make_arguments
from .common import extract_jobs_flags
from .common import format_time_delta
from .common import get_build_type
from .common import is_tty
from .common import log

from .context import Context

from .build import build_isolated_workspace
from .build import determine_packages_to_be_built
from .build import topological_order_packages


def argument_preprocessor(args):
    """Processes the arguments for the build verb, before being passed to argparse"""
    # CMake/make pass-through flags collect dashed options. They require special
    # handling or argparse will complain about unrecognized options.
    args = sys.argv[1:] if args is None else args
    extract_make_args = extract_cmake_and_make_and_catkin_make_arguments
    args, cmake_args, make_args, catkin_make_args = extract_make_args(args)
    # Extract make jobs flags.
    jobs_flags = extract_jobs_flags(' '.join(args))
    if jobs_flags:
        args = re.sub(jobs_flags, '', ' '.join(args)).split()
        jobs_flags = jobs_flags.split()
    extras = {
        'cmake_args': cmake_args,
        'make_args': make_args + (jobs_flags or []),
        'catkin_make_args': catkin_make_args,
    }
    return args, extras


def prepare_arguments(parser):
    add = parser.add_argument
    # What packages to build
    add('packages', nargs='*',
        help='Workspace packages to build, package dependencies are built as well unless --no-deps is used. '
             'If no packages are given, then all the packages are built.')
    add('--no-deps', action='store_true', default=False,
        help='Only build specified packages, not their dependencies.')
    add('--start-with', metavar='PKGNAME',
        help='Start building with this package, skipping any before it.')
    # Context options
    add('--workspace', '-w', default=None,
        help='The base path of the workspace (default ".")')
    add('--source', '--source-space', default=None,
        help='The path to the source space (default "src")')
    add('--build', '--build-space', default=None,
        help='The path to the build space (default "build")')
    add('--devel', '--devel-space', default=None,
        help='Sets the target devel space (default "devel")')
    add('--isolate-devel', action='store_true', default=False,
        help='Build products from each catkin package into isolated devel spaces.')
    add('--install-space', dest='install_space', default=None,
        help='Sets the target install space (default "install")')
    add('--install', action='store_true', default=False,
        help='Causes each catkin package to be installed.')
    add('--isolate-install', action='store_true', default=False,
        help='Install each catkin package into a separate install space.')
    add('--space-suffix',
        help='suffix for build, devel, and install space if they are not otherwise explicitly set')
    # Build options
    add('--parallel-jobs', '--parallel', '-p', default=None,
        help='Maximum number of packages which could be built in parallel (default is cpu count)')
    add('--force-cmake', action='store_true', default=False,
        help='Runs cmake explicitly for each catkin package.')
    add('--no-install-lock', action='store_true', default=False,
        help='Prevents serialization of the install steps, which is on by default to prevent file install collisions')
    add('--cmake-args', dest='cmake_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to CMake. '
             'It must be passed after other arguments since it collects all following options.')
    add('--make-args', dest='make_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to make.'
             'It must be passed after other arguments since it collects all following options.')
    add('--catkin-make-args', dest='catkin_make_args', nargs='*', type=str,
        help='Arbitrary arguments which are passes to make but only for catkin packages.'
             'It must be passed after other arguments since it collects all following options.')
    # Behavior
    add('--force-color', action='store_true', default=False,
        help='Forces catkin build to ouput in color, even when the terminal does not appear to support it.')
    add('--verbose', '-v', action='store_true', default=False,
        help='Print output from commands in ordered blocks once the command finishes.')
    add('--interleave-output', '-i', action='store_true', default=False,
        help='Prevents ordering of command output when multiple commands are running at the same time.')
    add('--no-status', action='store_true', default=False,
        help='Suppresses status line, useful in situations where carriage return is not properly supported.')
    # Commands
    add('--list-only', '--list', action='store_true', default=False,
        help='List packages in topological order, then exit.')

    return parser


def list_only(context, packages, no_deps, start_with):
    # Print Summary
    log(context.summary())
    # Find list of packages in the workspace
    packages_to_be_built, packages_to_be_built_deps = determine_packages_to_be_built(packages, context)
    if not no_deps:
        # Extend packages to be built to include their deps
        packages_to_be_built.extend(packages_to_be_built_deps)
        # Also resort
        packages_to_be_built = topological_order_packages(dict(packages_to_be_built))
    # Print packages
    log("Packages to be built:")
    max_name_len = str(max([len(pkg.name) for pth, pkg in packages_to_be_built]))
    prefix = clr('@{pf}' + ('------ ' if start_with else '- ') + '@|')
    for pkg_path, pkg in packages_to_be_built:
        build_type = get_build_type(pkg)
        if build_type == 'catkin' and 'metapackage' in [e.tagname for e in pkg.exports]:
            build_type = 'metapackage'
        if start_with and pkg.name == start_with:
            start_with = None
        log(clr("{prefix}@{cf}{name:<" + max_name_len + "}@| (@{yf}{build_type}@|)")
            .format(prefix=clr('@!@{kf}(skip)@| ') if start_with else prefix, name=pkg.name, build_type=build_type))
    log("Total packages: " + str(len(packages_to_be_built)))


def main(opts):
    if opts.no_deps and not opts.packages:
        sys.exit("With --no-deps, you must specify packages to build.")

    if not opts.force_color and not is_tty(sys.stdout):
        set_color(False)

    context = Context(
        workspace=opts.workspace,
        source_space=opts.source,
        build_space=opts.build,
        devel_space=opts.devel,
        isolate_devel=opts.isolate_devel,
        install_space=opts.install_space,
        install=opts.install,
        isolate_install=opts.isolate_install,
        cmake_args=opts.cmake_args,
        make_args=opts.make_args,
        catkin_make_args=opts.catkin_make_args,
        space_suffix=opts.space_suffix
    )

    if opts.list_only:
        list_only(context, opts.packages, opts.no_deps, opts.start_with)
        return

    start = time.time()
    try:
        return build_isolated_workspace(
            context,
            packages=opts.packages,
            start_with=opts.start_with,
            no_deps=opts.no_deps,
            jobs=opts.parallel_jobs,
            force_cmake=opts.force_cmake,
            force_color=opts.force_color,
            quiet=not opts.verbose,
            interleave_output=opts.interleave_output,
            no_status=opts.no_status,
            lock_install=not opts.no_install_lock
        )
    finally:
        log("[build] Runtime: {0}".format(format_time_delta(time.time() - start)))
