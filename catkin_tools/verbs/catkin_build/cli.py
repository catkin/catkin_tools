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

import argparse
import logging
import os
import sys

try:
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin_pkg.package import InvalidPackage
from catkin_pkg.tool_detection import get_previous_tool_used_on_the_space
from catkin_pkg.tool_detection import mark_space_as_built_by

from catkin_tools.argument_parsing import add_context_args
from catkin_tools.argument_parsing import add_cmake_and_make_and_catkin_make_args
from catkin_tools.argument_parsing import configure_make_args

from catkin_tools.common import getcwd
from catkin_tools.common import is_tty
from catkin_tools.common import log
from catkin_tools.common import find_enclosing_package
from catkin_tools.common import format_env_dict

from catkin_tools.context import Context

import catkin_tools.execution.job_server as job_server

from catkin_tools.jobs.utils import CommandMissing
from catkin_tools.jobs.utils import loadenv

from catkin_tools.metadata import find_enclosing_workspace
from catkin_tools.metadata import get_metadata
from catkin_tools.metadata import update_metadata

from catkin_tools.resultspace import load_resultspace_environment

from catkin_tools.terminal_color import set_color

from .color import clr

from .build import build_isolated_workspace
from .build import determine_packages_to_be_built
from .build import verify_start_with_option


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
    add('--get-env', dest='get_env', metavar='PKGNAME', nargs=1,
        help='Print the environment in which PKGNAME is built to stdout.')

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
    add('--unbuilt', action='store_true', default=False,
        help='Build packages which have yet to be built.')

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
    add('--pre-clean', action='store_true', default=None,
        help='Runs `make clean` before building each package.')
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
        help='Explicitly disable the end of build summary')
    add('--override-build-tool-check', action='store_true', default=False,
        help='use to override failure due to using differnt build tools on the same workspace.')

    # Deprecated args now handled by main catkin command
    add('--no-color', action='store_true', help=argparse.SUPPRESS)
    add('--force-color', action='store_true', help=argparse.SUPPRESS)

    # Experimental args
    add('--mem-limit', default=None, help=argparse.SUPPRESS)

    # Advanced args
    add('--develdebug', metavar='LEVEL', default=None, help=argparse.SUPPRESS)

    def status_rate_type(rate):
        rate = float(rate)
        if rate < 0:
            raise argparse.ArgumentTypeError("must be greater than or equal to zero.")
        return rate

    add('--limit-status-rate', '--status-rate', type=status_rate_type, default=10.0,
        help='Limit the update rate of the status bar to this frequency. Zero means unlimited. '
             'Must be positive, default is 10 Hz.')
    add('--no-notify', action='store_true', default=False,
        help='Suppresses system pop-up notification.')

    return parser


def dry_run(context, packages, no_deps, start_with):
    # Print Summary
    log(context.summary())
    # Get all the packages in the context source space
    # Suppress warnings since this is a utility function
    workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])
    # Find list of packages in the workspace
    packages_to_be_built, packages_to_be_built_deps, all_packages = determine_packages_to_be_built(
        packages, context, workspace_packages)
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
        build_type = pkg.get_build_type()
        if build_type == 'catkin' and 'metapackage' in [e.tagname for e in pkg.exports]:
            build_type = 'metapackage'
        if start_with and pkg.name == start_with:
            start_with = None
        log(clr("{prefix}@{cf}{name:<" + max_name_len + "}@| (@{yf}{build_type}@|)")
            .format(prefix=clr('@!@{kf}(skip)@| ') if start_with else prefix, name=pkg.name, build_type=build_type))
    log("Total packages: " + str(len(packages_to_be_built)))


def print_build_env(context, package_name):
    workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])
    # Load the environment used by this package for building
    for pth, pkg in workspace_packages.items():
        if pkg.name == package_name:
            environ = dict(os.environ)
            loadenv(None, None, environ, pkg, context)
            print(format_env_dict(environ))
            return 0
    print('[build] Error: Package `{}` not in workspace.'.format(package_name),
          file=sys.stderr)
    return 1


def main(opts):

    # Check for develdebug mode
    if opts.develdebug is not None:
        os.environ['PYTHONASYNCIODEBUG'] = opts.develdebug.lower()
        logging.basicConfig(level=opts.develdebug.upper())

    # Set color options
    opts.force_color = os.environ.get('CATKIN_TOOLS_FORCE_COLOR', opts.force_color)
    if (opts.force_color or is_tty(sys.stdout)) and not opts.no_color:
        set_color(True)
    else:
        set_color(False)

    # Context-aware args
    if opts.build_this or opts.start_with_this:
        # Determine the enclosing package
        try:
            ws_path = find_enclosing_workspace(getcwd())
            # Suppress warnings since this won't necessarily find all packages
            # in the workspace (it stops when it finds one package), and
            # relying on it for warnings could mislead people.
            this_package = find_enclosing_package(
                search_start_path=getcwd(),
                ws_path=ws_path,
                warnings=[])
        except InvalidPackage as ex:
            sys.exit(clr("@{rf}Error:@| The file %s is an invalid package.xml file."
                         " See below for details:\n\n%s" % (ex.package_path, ex.msg)))

        # Handle context-based package building
        if opts.build_this:
            if this_package:
                opts.packages += [this_package]
            else:
                sys.exit(
                    "[build] Error: In order to use --this, the current directory must be part of a catkin package.")

        # If --start--with was used without any packages and --this was specified, start with this package
        if opts.start_with_this:
            if this_package:
                opts.start_with = this_package
            else:
                sys.exit(
                    "[build] Error: In order to use --this, the current directory must be part of a catkin package.")

    if opts.no_deps and not opts.packages and not opts.unbuilt:
        sys.exit(clr("[build] @!@{rf}Error:@| With --no-deps, you must specify packages to build."))

    # Load the context
    if opts.build_this or opts.start_with_this:
        ctx = Context.load(opts.workspace, opts.profile, opts, append=True, strict=True)
    else:
        ctx = Context.load(opts.workspace, opts.profile, opts, append=True)

    # Handle no workspace
    if ctx is None:
        sys.exit(clr("[build] @!@{rf}Error:@| The current folder is not part of a catkin workspace."))

    # Initialize the build configuration
    make_args, makeflags, cli_flags, jobserver = configure_make_args(
        ctx.make_args, ctx.jobs_args, ctx.use_internal_make_jobserver)

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
        job_server.set_max_mem(opts.mem_limit)

    ctx.make_args = make_args

    # Load the environment of the workspace to extend
    if ctx.extend_path is not None:
        try:
            load_resultspace_environment(ctx.extend_path)
        except IOError as exc:
            sys.exit(clr("[build] @!@{rf}Error:@| Unable to extend workspace from \"%s\": %s" %
                         (ctx.extend_path, exc.message)))

    # Check if the context is valid before writing any metadata
    if not ctx.source_space_exists():
        sys.exit(clr("[build] @!@{rf}Error:@| Unable to find source space `%s`") % ctx.source_space_abs)

    # ensure the build space was previously built by catkin_tools
    previous_tool = get_previous_tool_used_on_the_space(ctx.build_space_abs)
    if previous_tool is not None and previous_tool != 'catkin build':
        if opts.override_build_tool_check:
            log(clr(
                "@{yf}Warning: build space at '%s' was previously built by '%s', "
                "but --override-build-tool-check was passed so continuing anyways."
                % (ctx.build_space_abs, previous_tool)))
        else:
            sys.exit(clr(
                "@{rf}The build space at '%s' was previously built by '%s'. "
                "Please remove the build space or pick a different build space."
                % (ctx.build_space_abs, previous_tool)))
    # the build space will be marked as catkin build's if dry run doesn't return

    # ensure the devel space was previously built by catkin_tools
    previous_tool = get_previous_tool_used_on_the_space(ctx.devel_space_abs)
    if previous_tool is not None and previous_tool != 'catkin build':
        if opts.override_build_tool_check:
            log(clr(
                "@{yf}Warning: devel space at '%s' was previously built by '%s', "
                "but --override-build-tool-check was passed so continuing anyways."
                % (ctx.devel_space_abs, previous_tool)))
        else:
            sys.exit(clr(
                "@{rf}The devel space at '%s' was previously built by '%s'. "
                "Please remove the devel space or pick a different devel space."
                % (ctx.devel_space_abs, previous_tool)))
    # the devel space will be marked as catkin build's if dry run doesn't return

    # Display list and leave the file system untouched
    if opts.dry_run:
        # TODO: Add unbuilt
        dry_run(ctx, opts.packages, opts.no_deps, opts.start_with)
        return

    # Print the build environment for a given package and leave the filesystem untouched
    if opts.get_env:
        return print_build_env(ctx, opts.get_env[0])

    # Now mark the build and devel spaces as catkin build's since dry run didn't return.
    mark_space_as_built_by(ctx.build_space_abs, 'catkin build')
    mark_space_as_built_by(ctx.devel_space_abs, 'catkin build')

    # Get the last build context
    build_metadata = get_metadata(ctx.workspace, ctx.profile, 'build')

    # Force cmake if the CMake arguments have changed
    if build_metadata.get('cmake_args') != ctx.cmake_args:
        opts.force_cmake = True

    # Check the devel layout compatibility
    last_devel_layout = build_metadata.get('devel_layout', ctx.devel_layout)
    if last_devel_layout != ctx.devel_layout:
        sys.exit(clr(
            "@{rf}@!Error:@|@{rf} The current devel space layout, `{}`, "
            "is incompatible with the configured layout, `{}`.@|").format(
            last_devel_layout, ctx.devel_layout))

    # Check if some other verb has changed the workspace in such a way that it needs to be forced
    if build_metadata.get('needs_force', False):
        opts.force_cmake = True
        update_metadata(ctx.workspace, ctx.profile, 'build', {'needs_force': False})

    # Always save the last context under the build verb
    update_metadata(ctx.workspace, ctx.profile, 'build', ctx.get_stored_dict())

    # Save the context as the configuration
    if opts.save_config:
        Context.save(ctx)

    # Get parallel toplevel jobs
    try:
        parallel_jobs = int(opts.parallel_jobs)
    except TypeError:
        parallel_jobs = None

    # Set VERBOSE environment variable
    if opts.verbose and 'VERBOSE' not in os.environ:
        os.environ['VERBOSE'] = '1'

    try:
        return build_isolated_workspace(
            ctx,
            packages=opts.packages,
            start_with=opts.start_with,
            no_deps=opts.no_deps,
            unbuilt=opts.unbuilt,
            n_jobs=parallel_jobs,
            force_cmake=opts.force_cmake,
            pre_clean=opts.pre_clean,
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
    except CommandMissing as e:
        sys.exit(clr("[build] @!@{rf}Error:@| {0}").format(e))
