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
import os
import sys
import time

from catkin_pkg.package import InvalidPackage

from catkin_tools.argument_parsing import add_context_args
from catkin_tools.argument_parsing import add_cmake_and_make_and_catkin_make_args
from catkin_tools.argument_parsing import configure_make_args

from catkin_tools.common import format_time_delta
from catkin_tools.common import getcwd
from catkin_tools.common import log
from catkin_tools.common import find_enclosing_package

from catkin_tools.context import Context

from catkin_tools.make_jobserver import set_jobserver_max_mem

from catkin_tools.metadata import find_enclosing_workspace

from catkin_tools.metadata import get_metadata
from catkin_tools.metadata import update_metadata

from catkin_tools.resultspace import load_resultspace_environment

from .color import clr

from .common import get_build_type

from .build import build_isolated_workspace
from .build import determine_packages_to_be_built
from .build import topological_order_packages
from .build import verify_start_with_option

#
# Hack
#

# TODO(wjwwood): remove this, once it is no longer needed.
# argparse may not support mutually exclusive groups inside other groups, see:
#   http://bugs.python.org/issue10680

# Backup the original constructor
backup__ArgumentGroup___init__ = argparse._ArgumentGroup.__init__


# Make a new constructor with the fix
def fixed__ArgumentGroup___init__(self, container, title=None, description=None, **kwargs):
    backup__ArgumentGroup___init__(self, container, title, description, **kwargs)
    # Make sure this line is run, maybe redundant on versions which already have it
    self._mutually_exclusive_groups = container._mutually_exclusive_groups

# Monkey patch in the fixed constructor
argparse._ArgumentGroup.__init__ = fixed__ArgumentGroup___init__

#
# End Hack
#


def prepare_arguments(parser):
    parser.description = """\
Build one or more packages in a catkin workspace.
This invokes `CMake`, `make`, and optionally `make install` for either all
or the specified packages in a catkin workspace.

Arguments passed to this verb can temporarily override persistent options
stored in the catkin profile config. If you want to save these options, use
the --save-config argument. To see the current config, use the
`catkin config` command.\
"""

    # Workspace / profile args
    add_context_args(parser)
    # Sub-commands
    add = parser.add_argument
    add('--dry-run', '-n', action='store_true', default=False,
        help='List the packages which will be built with the given arguments without building them.')
    # What packages to build
    pkg_group = parser.add_argument_group('Packages', 'Control which packages get built.')
    add = pkg_group.add_argument
    add('packages', metavar='PKGNAME', nargs='*',
        help='Workspace packages to build, package dependencies are built as well unless --no-deps is used. '
             'If no packages are given, then all the packages are built.')
    add('--this', dest='build_this', action='store_true', default=False,
        help='Build the package containing the current working directory.')
    add('--no-deps', action='store_true', default=False,
        help='Only build specified packages, not their dependencies.')
    start_with_group = pkg_group.add_mutually_exclusive_group()
    add = start_with_group.add_argument
    add('--start-with', metavar='PKGNAME', type=str,
        help='Build a given package and those which depend on it, skipping any before it.')
    add('--start-with-this', action='store_true', default=False,
        help='Similar to --start-with, starting with the package containing the current directory.')
    add = pkg_group.add_argument
    add('--continue-on-failure', '-c', action='store_true', default=False,
        help='Try to continue building packages whose dependencies built successfully even if some other requested '
             'packages fail to build.')

    # Build options
    build_group = parser.add_argument_group('Build', 'Control the build behavior.')
    add = build_group.add_argument
    add('--force-cmake', action='store_true', default=None,
        help='Runs cmake explicitly for each catkin package.')
    add('--no-install-lock', action='store_true', default=None,
        help='Prevents serialization of the install steps, which is on by default to prevent file install collisions')

    config_group = parser.add_argument_group('Config', 'Parameters for the underlying build system.')
    add = config_group.add_argument
    add('--save-config', action='store_true', default=False,
        help='Save any configuration options in this section for the next build invocation.')
    add_cmake_and_make_and_catkin_make_args(config_group)

    # Behavior
    behavior_group = parser.add_argument_group('Interface', 'The behavior of the command-line interface.')
    add = behavior_group.add_argument
    add('--verbose', '-v', action='store_true', default=False,
        help='Print output from commands in ordered blocks once the command finishes.')
    add('--interleave-output', '-i', action='store_true', default=False,
        help='Prevents ordering of command output when multiple commands are running at the same time.')
    add('--no-status', action='store_true', default=False,
        help='Suppresses status line, useful in situations where carriage return is not properly supported.')
    add('--summarize', '--summary', '-s', action='store_true', default=None,
        help='Adds a build summary to the end of a build; defaults to on with --continue-on-failure, off otherwise')
    add('--no-summarize', '--no-summary', action='store_false', dest='summarize',
        help='explicitly disable the end of build summary')

    # Deprecated args now handled by main catkin command
    add('--no-color', action='store_true', help=argparse.SUPPRESS)
    add('--force-color', action='store_true', help=argparse.SUPPRESS)

    # Experimental args
    add('--mem-limit', default=None, help=argparse.SUPPRESS)

    def status_rate_type(rate):
        rate = float(rate)
        if rate < 0:
            raise argparse.ArgumentTypeError("must be greater than or equal to zero.")
        return rate

    add('--limit-status-rate', '--status-rate', type=status_rate_type, default=0.0,
        help='Limit the update rate of the status bar to this frequency. Zero means unlimited. '
             'Must be positive, default is 0.')
    add('--no-notify', action='store_true', default=False,
        help='Suppresses system pop-up notification.')

    return parser


def dry_run(context, packages, no_deps, start_with):
    # Print Summary
    log(context.summary())
    # Find list of packages in the workspace
    packages_to_be_built, packages_to_be_built_deps, all_packages = determine_packages_to_be_built(packages, context)
    # Assert start_with package is in the workspace
    verify_start_with_option(start_with, packages, all_packages, packages_to_be_built + packages_to_be_built_deps)
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
    # Context-aware args
    if opts.build_this or opts.start_with_this:
        # Determine the enclosing package
        try:
            ws_path = find_enclosing_workspace(getcwd())
            # Suppress warnings since this won't necessaraly find all packages
            # in the workspace (it stops when it finds one package), and
            # relying on it for warnings could mislead people.
            this_package = find_enclosing_package(
                search_start_path=getcwd(),
                ws_path=ws_path,
                warnings=[])
        except (InvalidPackage, RuntimeError):
            this_package = None

        # Handle context-based package building
        if opts.build_this:
            if this_package:
                opts.packages += [this_package]
            else:
                sys.exit("catkin build: --this was specified, but this directory is not in a catkin package.")

        # If --start--with was used without any packages and --this was specified, start with this package
        if opts.start_with_this:
            if this_package:
                opts.start_with = this_package
            else:
                sys.exit("catkin build: --this was specified, but this directory is not in a catkin package.")

    if opts.no_deps and not opts.packages:
        sys.exit("With --no-deps, you must specify packages to build.")

    # Load the context
    ctx = Context.load(opts.workspace, opts.profile, opts, append=True)

    # Initialize the build configuration
    make_args, makeflags, cli_flags, jobserver = configure_make_args(ctx.make_args, ctx.use_internal_make_jobserver)

    # Set the jobserver memory limit
    if jobserver and opts.mem_limit:
        log(clr("@!@{pf}EXPERIMENTAL: limit memory to '%s'@|" % str(opts.mem_limit)))
        # At this point psuitl will be required, check for it and bail out if not set
        try:
            import psutil  # noqa
        except ImportError as exc:
            log("Could not import psutil, but psutil is required when using --mem-limit.")
            log("Please either install psutil or avoid using --mem-limit.")
            sys.exit("Exception: {0}".format(exc))
        set_jobserver_max_mem(opts.mem_limit)

    ctx.make_args = make_args

    # Load the environment of the workspace to extend
    if ctx.extend_path is not None:
        try:
            load_resultspace_environment(ctx.extend_path)
        except IOError as exc:
            log(clr("@!@{rf}Error:@| Unable to extend workspace from \"%s\": %s" %
                    (ctx.extend_path, exc.message)))
            return 1

    # Display list and leave the file system untouched
    if opts.dry_run:
        dry_run(ctx, opts.packages, opts.no_deps, opts.start_with)
        return

    # Check if the context is valid before writing any metadata
    if not ctx.source_space_exists():
        print("catkin build: error: Unable to find source space `%s`" % ctx.source_space_abs)
        return 1

    # Always save the last context under the build verb
    update_metadata(ctx.workspace, ctx.profile, 'build', ctx.get_stored_dict())

    build_metadata = get_metadata(ctx.workspace, ctx.profile, 'build')
    if build_metadata.get('needs_force', False):
        opts.force_cmake = True
        update_metadata(ctx.workspace, ctx.profile, 'build', {'needs_force': False})

    # Save the context as the configuration
    if opts.save_config:
        Context.save(ctx)

    start = time.time()
    try:
        return build_isolated_workspace(
            ctx,
            packages=opts.packages,
            start_with=opts.start_with,
            no_deps=opts.no_deps,
            jobs=opts.parallel_jobs,
            force_cmake=opts.force_cmake,
            force_color=opts.force_color,
            quiet=not opts.verbose,
            interleave_output=opts.interleave_output,
            no_status=opts.no_status,
            limit_status_rate=opts.limit_status_rate,
            lock_install=not opts.no_install_lock,
            no_notify=opts.no_notify,
            continue_on_failure=opts.continue_on_failure,
            summarize_build=opts.summarize  # Can be True, False, or None
        )
    finally:
        log("[build] Runtime: {0}".format(format_time_delta(time.time() - start)))
