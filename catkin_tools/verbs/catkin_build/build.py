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

"""This modules implements the engine for building packages in parallel"""

import os
import pkg_resources
from queue import Queue
import sys
import time
import traceback
import yaml
import asyncio

try:
    from catkin_pkg.package import parse_package
    from catkin_pkg.package import InvalidPackage
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'Importing "catkin_pkg" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin_tools.common import FakeLock, expand_glob_package
from catkin_tools.common import format_time_delta
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace
from catkin_tools.common import get_recursive_run_depends_in_workspace
from catkin_tools.common import log
from catkin_tools.common import wide_log

from catkin_tools.execution.controllers import ConsoleStatusController
from catkin_tools.execution.executor import execute_jobs
from catkin_tools.execution.executor import run_until_complete

from catkin_tools.jobs.catkin import create_catkin_build_job
from catkin_tools.jobs.catkin import create_catkin_clean_job
from catkin_tools.jobs.catkin import get_prebuild_package

from .color import clr

BUILDSPACE_MARKER_FILE = '.catkin_tools.yaml'
BUILDSPACE_IGNORE_FILE = 'CATKIN_IGNORE'
DEVELSPACE_MARKER_FILE = '.catkin_tools.yaml'


def determine_packages_to_be_built(packages, context, workspace_packages):
    """Returns list of packages which should be built, and those package's deps.

    :param packages: list of packages to be built, if None all packages are built
    :type packages: list
    :param context: Workspace context
    :type context: :py:class:`catkin_tools.verbs.catkin_build.context.Context`
    :returns: tuple of packages to be built and those package's deps
    :rtype: tuple
    """
    start = time.time()

    # If there are no packages raise
    if not workspace_packages:
        log("[build] No packages were found in the source space '{0}'".format(context.source_space_abs))
    else:
        wide_log("[build] Found '{0}' packages in {1}."
                 .format(len(workspace_packages), format_time_delta(time.time() - start)))

    # Order the packages by topology
    ordered_packages = topological_order_packages(workspace_packages)
    # Set the packages in the workspace for the context
    context.packages = ordered_packages
    # Determine the packages which should be built
    packages_to_be_built = []
    packages_to_be_built_deps = []

    # Check if topological_order_packages determined any circular dependencies, if so print an error and fail.
    # If this is the case, the last entry of ordered packages is a tuple that starts with nil.
    if ordered_packages and ordered_packages[-1][0] is None:
        guilty_packages = ", ".join(ordered_packages[-1][1:])
        sys.exit("[build] Circular dependency detected in the following packages: {}".format(guilty_packages))

    workspace_package_names = dict([(pkg.name, (path, pkg)) for path, pkg in ordered_packages])
    # Determine the packages to be built
    if packages:
        # First assert all of the packages given are in the workspace
        for package in packages:
            if package not in workspace_package_names:
                # Try whether package is a pattern and matches
                glob_packages = expand_glob_package(package, workspace_package_names)
                if len(glob_packages) > 0:
                    packages.extend(glob_packages)
                    continue
                else:
                    sys.exit("[build] Given package '{0}' is not in the workspace "
                             "and pattern does not match any package".format(package))
            # If metapackage, include run depends which are in the workspace
            package_obj = workspace_package_names[package][1]
            if 'metapackage' in [e.tagname for e in package_obj.exports]:
                for rdep in package_obj.run_depends:
                    if rdep.name in workspace_package_names:
                        packages.append(rdep.name)
        # Limit the packages to be built to just the provided packages
        for pkg_path, package in ordered_packages:
            if package.name in packages:
                packages_to_be_built.append((pkg_path, package))
                # Get the recursive dependencies for each of these packages
                pkg_deps = get_cached_recursive_build_depends_in_workspace(package, ordered_packages)
                packages_to_be_built_deps.extend(pkg_deps)
    else:
        # Only use whitelist when no other packages are specified
        if len(context.whitelist) > 0:
            # Expand glob patterns in whitelist
            whitelist = []
            for whitelisted_package in context.whitelist:
                whitelist.extend(expand_glob_package(whitelisted_package, workspace_package_names))
            packages_to_be_built = [p for p in ordered_packages if (p[1].name in whitelist)]
        else:
            packages_to_be_built = ordered_packages

    # Filter packages with blacklist
    if len(context.blacklist) > 0:
        # Expand glob patterns in blacklist
        blacklist = []
        for blacklisted_package in context.blacklist:
            blacklist.extend(expand_glob_package(blacklisted_package, workspace_package_names))
        # Apply blacklist to packages and dependencies
        packages_to_be_built = [
            (path, pkg) for path, pkg in packages_to_be_built
            if (pkg.name not in blacklist or pkg.name in packages)]
        packages_to_be_built_deps = [
            (path, pkg) for path, pkg in packages_to_be_built_deps
            if (pkg.name not in blacklist or pkg.name in packages)]

    return packages_to_be_built, packages_to_be_built_deps, ordered_packages


def verify_start_with_option(start_with, packages, all_packages, packages_to_be_built):
    if start_with is not None:
        if start_with not in [pkg.name for pth, pkg in all_packages]:
            sys.exit("Package given for --start-with, '{0}', is not in the workspace.".format(start_with))
        elif start_with not in [pkg.name for pth, pkg in packages_to_be_built]:
            sys.exit("Package given for --start-with, '{0}', "
                     "is in the workspace but would not be built with given package arguments: '{1}'"
                     .format(start_with, ' '.join(packages)))


def get_built_unbuilt_packages(context, workspace_packages):
    """Get list of packages in workspace which have not been built."""

    # Get the names of all packages which have already been built
    built_packages = set([
        pkg.name for (path, pkg) in
        find_packages(context.package_metadata_path(), warnings=[]).items()])

    # Get names of all unbuilt packages
    unbuilt_pkgs = set()
    for path, pkg in workspace_packages.items():
        if pkg.name not in built_packages:
            unbuilt_pkgs.add(pkg.name)

    return built_packages, unbuilt_pkgs


def build_isolated_workspace(
    context,
    packages=None,
    start_with=None,
    no_deps=False,
    unbuilt=False,
    n_jobs=None,
    force_cmake=False,
    pre_clean=False,
    force_color=False,
    quiet=False,
    interleave_output=False,
    no_status=False,
    limit_status_rate=10.0,
    lock_install=False,
    no_notify=False,
    continue_on_failure=False,
    summarize_build=None,
):
    """Builds a catkin workspace in isolation

    This function will find all of the packages in the source space, start some
    executors, feed them packages to build based on dependencies and topological
    ordering, and then monitor the output of the executors, handling loggings of
    the builds, starting builds, failing builds, and finishing builds of
    packages, and handling the shutdown of the executors when appropriate.

    :param context: context in which to build the catkin workspace
    :type context: :py:class:`catkin_tools.verbs.catkin_build.context.Context`
    :param packages: list of packages to build, by default their dependencies will also be built
    :type packages: list
    :param start_with: package to start with, skipping all packages which proceed it in the topological order
    :type start_with: str
    :param no_deps: If True, the dependencies of packages will not be built first
    :type no_deps: bool
    :param n_jobs: number of parallel package build n_jobs
    :type n_jobs: int
    :param force_cmake: forces invocation of CMake if True, default is False
    :type force_cmake: bool
    :param force_color: forces colored output even if terminal does not support it
    :type force_color: bool
    :param quiet: suppresses the output of commands unless there is an error
    :type quiet: bool
    :param interleave_output: prints the output of commands as they are received
    :type interleave_output: bool
    :param no_status: disables status bar
    :type no_status: bool
    :param limit_status_rate: rate to which status updates are limited; the default 0, places no limit.
    :type limit_status_rate: float
    :param lock_install: causes executors to synchronize on access of install commands
    :type lock_install: bool
    :param no_notify: suppresses system notifications
    :type no_notify: bool
    :param continue_on_failure: do not stop building other jobs on error
    :type continue_on_failure: bool
    :param summarize_build: if True summarizes the build at the end, if None and continue_on_failure is True and the
        the build fails, then the build will be summarized, but if False it never will be summarized.
    :type summarize_build: bool

    :raises: SystemExit if buildspace is a file or no packages were found in the source space
        or if the provided options are invalid
    """
    pre_start_time = time.time()

    # Assert that the limit_status_rate is valid
    if limit_status_rate < 0:
        sys.exit("[build] @!@{rf}Error:@| The value of --status-rate must be greater than or equal to zero.")

    # Declare a buildspace marker describing the build config for error checking
    buildspace_marker_data = {
        'workspace': context.workspace,
        'profile': context.profile,
        'install': context.install,
        'install_space': context.install_space_abs,
        'devel_space': context.devel_space_abs,
        'source_space': context.source_space_abs}

    # Check build config
    if os.path.exists(os.path.join(context.build_space_abs, BUILDSPACE_MARKER_FILE)):
        with open(os.path.join(context.build_space_abs, BUILDSPACE_MARKER_FILE)) as buildspace_marker_file:
            existing_buildspace_marker_data = yaml.safe_load(buildspace_marker_file)
            misconfig_lines = ''
            for (k, v) in existing_buildspace_marker_data.items():
                new_v = buildspace_marker_data.get(k, None)
                if new_v != v:
                    misconfig_lines += (
                        '\n - %s: %s (stored) is not %s (commanded)' %
                        (k, v, new_v))
            if len(misconfig_lines) > 0:
                sys.exit(clr(
                    "\n@{rf}Error:@| Attempting to build a catkin workspace using build space: "
                    "\"%s\" but that build space's most recent configuration "
                    "differs from the commanded one in ways which will cause "
                    "problems. Fix the following options or use @{yf}`catkin "
                    "clean -b`@| to remove the build space: %s" %
                    (context.build_space_abs, misconfig_lines)))

    # Summarize the context
    summary_notes = []
    if force_cmake:
        summary_notes += [clr("@!@{cf}NOTE:@| Forcing CMake to run for each package.")]
    log(context.summary(summary_notes))

    # Make sure there is a build folder and it is not a file
    if os.path.exists(context.build_space_abs):
        if os.path.isfile(context.build_space_abs):
            sys.exit(clr(
                "[build] @{rf}Error:@| " +
                "Build space '{0}' exists but is a file and not a folder."
                .format(context.build_space_abs)))
    # If it doesn't exist, create it
    else:
        log("[build] Creating build space: '{0}'".format(context.build_space_abs))
        os.makedirs(context.build_space_abs)

    # Write the current build config for config error checking
    with open(os.path.join(context.build_space_abs, BUILDSPACE_MARKER_FILE), 'w') as buildspace_marker_file:
        buildspace_marker_file.write(yaml.dump(buildspace_marker_data, default_flow_style=False))

    # Get all the packages in the context source space
    # Suppress warnings since this is a utility function
    try:
        workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])
    except InvalidPackage as ex:
        sys.exit(clr("@{rf}Error:@| The file %s is an invalid package.xml file."
                     " See below for details:\n\n%s" % (ex.package_path, ex.msg)))

    # Get packages which have not been built yet
    built_packages, unbuilt_pkgs = get_built_unbuilt_packages(context, workspace_packages)

    # Handle unbuilt packages
    if unbuilt:
        # Check if there are any unbuilt
        if len(unbuilt_pkgs) > 0:
            # Add the unbuilt packages
            packages.extend(list(unbuilt_pkgs))
        else:
            log("[build] No unbuilt packages to be built.")
            return

    # If no_deps is given, ensure packages to build are provided
    if no_deps and packages is None:
        log(clr("[build] @!@{rf}Error:@| With no_deps, you must specify packages to build."))
        return

    # Find list of packages in the workspace
    packages_to_be_built, packages_to_be_built_deps, all_packages = determine_packages_to_be_built(
        packages, context, workspace_packages)

    if not no_deps:
        # Extend packages to be built to include their deps
        packages_to_be_built.extend(packages_to_be_built_deps)

    # Also re-sort
    try:
        packages_to_be_built = topological_order_packages(dict(packages_to_be_built))
    except AttributeError:
        log(clr("[build] @!@{rf}Error:@| The workspace packages have a circular "
                "dependency, and cannot be built. Please run `catkin list "
                "--deps` to determine the problematic package(s)."))
        return

    # Check the number of packages to be built
    if len(packages_to_be_built) == 0:
        log(clr('[build] No packages to be built.'))

    # Assert start_with package is in the workspace
    verify_start_with_option(
        start_with,
        packages,
        all_packages,
        packages_to_be_built + packages_to_be_built_deps)

    # Populate .catkin file if we're not installing
    # NOTE: This is done to avoid the Catkin CMake code from doing it,
    # which isn't parallel-safe. Catkin CMake only modifies this file if
    # it's package source path isn't found.
    if not context.install:
        dot_catkin_file_path = os.path.join(context.devel_space_abs, '.catkin')
        # If the file exists, get the current paths
        if os.path.exists(dot_catkin_file_path):
            dot_catkin_paths = open(dot_catkin_file_path, 'r').read().split(';')
        else:
            dot_catkin_paths = []

        # Update the list with the new packages (in topological order)
        packages_to_be_built_paths = [
            os.path.join(context.source_space_abs, path)
            for path, pkg in packages_to_be_built
        ]

        new_dot_catkin_paths = [
            os.path.join(context.source_space_abs, path)
            for path in [os.path.join(context.source_space_abs, path) for path, pkg in all_packages]
            if path in dot_catkin_paths or path in packages_to_be_built_paths
        ]

        # Write the new file if it's different, otherwise, leave it alone
        if dot_catkin_paths == new_dot_catkin_paths:
            wide_log("[build] Package table is up to date.")
        else:
            wide_log("[build] Updating package table.")
            open(dot_catkin_file_path, 'w').write(';'.join(new_dot_catkin_paths))

    # Remove packages before start_with
    if start_with is not None:
        for path, pkg in list(packages_to_be_built):
            if pkg.name != start_with:
                wide_log(clr("@!@{pf}Skipping@|  @{gf}---@| @{cf}{}@|").format(pkg.name))
                packages_to_be_built.pop(0)
            else:
                break

    # Get the names of all packages to be built
    packages_to_be_built_names = [p.name for _, p in packages_to_be_built]
    packages_to_be_built_deps_names = [p.name for _, p in packages_to_be_built_deps]

    # Generate prebuild and prebuild clean jobs, if necessary
    prebuild_jobs = {}
    setup_util_present = os.path.exists(os.path.join(context.devel_space_abs, '_setup_util.py'))
    if context.install:
        setup_util_present &= os.path.exists(os.path.join(context.install_space_abs, '_setup_util.py'))
    catkin_present = 'catkin' in (packages_to_be_built_names + packages_to_be_built_deps_names)
    catkin_built = 'catkin' in built_packages
    prebuild_built = 'catkin_tools_prebuild' in built_packages

    # Handle the prebuild jobs if the develspace is linked
    prebuild_pkg_deps = []
    if context.link_devel:
        prebuild_pkg = None

        # Construct a dictionary to lookup catkin package by name
        pkg_dict = dict([(pkg.name, (pth, pkg)) for pth, pkg in all_packages])

        if setup_util_present:
            # Setup util is already there, determine if it needs to be
            # regenerated
            if catkin_built:
                if catkin_present:
                    prebuild_pkg_path, prebuild_pkg = pkg_dict['catkin']
            elif prebuild_built:
                if catkin_present:
                    # TODO: Clean prebuild package
                    ct_prebuild_pkg_path = get_prebuild_package(
                        context.build_space_abs, context.devel_space_abs, force_cmake)
                    ct_prebuild_pkg = parse_package(ct_prebuild_pkg_path)

                    prebuild_jobs['caktin_tools_prebuild'] = create_catkin_clean_job(
                        context,
                        ct_prebuild_pkg,
                        ct_prebuild_pkg_path,
                        dependencies=[],
                        dry_run=False,
                        clean_build=True,
                        clean_devel=True,
                        clean_install=True)

                    # TODO: Build catkin package
                    prebuild_pkg_path, prebuild_pkg = pkg_dict['catkin']
                    prebuild_pkg_deps.append('catkin_tools_prebuild')
            else:
                # How did these get here??
                log("Warning: devel space setup files have an unknown origin.")
        else:
            # Setup util needs to be generated
            if catkin_built or prebuild_built:
                log("Warning: generated devel space setup files have been deleted.")

            if catkin_present:
                # Build catkin package
                prebuild_pkg_path, prebuild_pkg = pkg_dict['catkin']
            else:
                # Generate and buildexplicit prebuild package
                prebuild_pkg_path = get_prebuild_package(context.build_space_abs, context.devel_space_abs, force_cmake)
                prebuild_pkg = parse_package(prebuild_pkg_path)

        if prebuild_pkg is not None:
            # Create the prebuild job
            prebuild_job = create_catkin_build_job(
                context,
                prebuild_pkg,
                prebuild_pkg_path,
                dependencies=prebuild_pkg_deps,
                force_cmake=force_cmake,
                pre_clean=pre_clean,
                prebuild=True)

            # Add the prebuld job
            prebuild_jobs[prebuild_job.jid] = prebuild_job

    # Remove prebuild jobs from normal job list
    for prebuild_jid, prebuild_job in prebuild_jobs.items():
        if prebuild_jid in packages_to_be_built_names:
            packages_to_be_built_names.remove(prebuild_jid)

    # Initial jobs list is just the prebuild jobs
    jobs = [] + list(prebuild_jobs.values())

    # Get all build type plugins
    build_job_creators = {
        ep.name: ep.load()['create_build_job']
        for ep in pkg_resources.iter_entry_points(group='catkin_tools.jobs')
    }

    # It's a problem if there aren't any build types available
    if len(build_job_creators) == 0:
        sys.exit('Error: No build types available. Please check your catkin_tools installation.')

    # Construct jobs
    for pkg_path, pkg in all_packages:
        if pkg.name not in packages_to_be_built_names:
            continue

        # Get actual build deps
        deps = [
            p.name for _, p
            in get_cached_recursive_build_depends_in_workspace(pkg, packages_to_be_built)
            if p.name not in prebuild_jobs
        ]
        # All jobs depend on the prebuild jobs if they're defined
        if not no_deps:
            for j in prebuild_jobs.values():
                deps.append(j.jid)

        # Determine the job parameters
        build_job_kwargs = dict(
            context=context,
            package=pkg,
            package_path=pkg_path,
            dependencies=deps,
            force_cmake=force_cmake,
            pre_clean=pre_clean)

        # Create the job based on the build type
        build_type = pkg.get_build_type()

        if build_type in build_job_creators:
            jobs.append(build_job_creators[build_type](**build_job_kwargs))
        else:
            wide_log(clr(
                "[build] @!@{yf}Warning:@| Skipping package `{}` because it "
                "has an unsupported package build type: `{}`"
            ).format(pkg.name, build_type))

            wide_log(clr("[build] Note: Available build types:"))
            for bt_name in build_job_creators.keys():
                wide_log(clr("[build]  - `{}`".format(bt_name)))

    # Queue for communicating status
    event_queue = Queue()

    try:
        # Spin up status output thread
        status_thread = ConsoleStatusController(
            'build',
            ['package', 'packages'],
            jobs,
            n_jobs,
            [pkg.name for _, pkg in context.packages],
            [p for p in context.whitelist],
            [p for p in context.blacklist],
            event_queue,
            show_notifications=not no_notify,
            show_active_status=not no_status,
            show_buffered_stdout=not quiet and not interleave_output,
            show_buffered_stderr=not interleave_output,
            show_live_stdout=interleave_output,
            show_live_stderr=interleave_output,
            show_stage_events=not quiet,
            show_full_summary=(summarize_build is True),
            pre_start_time=pre_start_time,
            active_status_rate=limit_status_rate)
        status_thread.start()

        # Initialize locks
        locks = {
            'installspace': asyncio.Lock() if lock_install else FakeLock()
        }

        # Block while running N jobs asynchronously
        try:
            all_succeeded = run_until_complete(execute_jobs(
                'build',
                jobs,
                locks,
                event_queue,
                context.log_space_abs,
                max_toplevel_jobs=n_jobs,
                continue_on_failure=continue_on_failure,
                continue_without_deps=False))
        except Exception:
            status_thread.keep_running = False
            all_succeeded = False
            status_thread.join(1.0)
            wide_log(str(traceback.format_exc()))

        status_thread.join(1.0)

        # Warn user about new packages
        now_built_packages, now_unbuilt_pkgs = get_built_unbuilt_packages(context, workspace_packages)
        new_pkgs = [p for p in unbuilt_pkgs if p not in now_unbuilt_pkgs]
        if len(new_pkgs) > 0:
            log(clr("[build] @/@!Note:@| @/Workspace packages have changed, "
                    "please re-source setup files to use them.@|"))

        if all_succeeded:
            # Create isolated devel setup if necessary
            if context.isolate_devel:
                if not context.install:
                    _create_unmerged_devel_setup(context, now_unbuilt_pkgs)
                else:
                    _create_unmerged_devel_setup_for_install(context)
            return 0
        else:
            return 1

    except KeyboardInterrupt:
        wide_log("[build] Interrupted by user!")
        event_queue.put(None)

        return 130  # EOWNERDEAD return code is not part of the errno module.


def _create_unmerged_devel_setup(context, unbuilt):
    # Find all of the leaf packages in the workspace
    # where leaf means that nothing in the workspace depends on it

    ordered_packages = context.packages
    workspace_packages = dict([(p.name, p) for pth, p in ordered_packages])

    # Get all packages which are dependencies of packages in the workspace which have been built
    dependencies = set(sum([
        [d.name for d in p.buildtool_depends + p.build_depends + p.run_depends]
        for _, p in workspace_packages.items()
        if p.name not in unbuilt
    ], []))

    # Compute the packages on which no other packages depend
    leaf_packages = [
        pkg.name
        for name, pkg in workspace_packages.items()
        if pkg.name not in dependencies
    ]
    leaf_paths = [
        os.path.join(context.devel_space_abs, p, 'setup.sh')
        for p in leaf_packages
    ]
    leaf_sources = [
        '. {}'.format(source_path)
        for source_path in leaf_paths
        if os.path.isfile(source_path)
    ]

    # In addition to the leaf packages, we need to source the recursive run depends of the leaf packages
    run_depends_packages = get_recursive_run_depends_in_workspace(
        [workspace_packages[p] for p in leaf_packages], ordered_packages)
    run_depends_paths = [
        os.path.join(context.devel_space_abs, pth, 'setup.sh')
        for pth, pkg in run_depends_packages
    ]
    run_depends_sources = [
        '. {}'.format(source_path)
        for source_path in run_depends_paths
        if os.path.isfile(source_path)
    ]

    # Create the setup.sh file
    setup_sh_path = os.path.join(context.devel_space_abs, 'setup.sh')
    env_file = SETUP_SH_TEMPLATE.format(
        first_source=leaf_sources[0],
        leaf_sources='\n'.join(leaf_sources[1:]),
        run_depends_sources='\n'.join(run_depends_sources)
    )
    with open(setup_sh_path, 'w') as f:
        f.write(env_file)

    # Create setup.bash file
    setup_bash_path = os.path.join(context.devel_space_abs, 'setup.bash')
    with open(setup_bash_path, 'w') as f:
        f.write(SETUP_BASH_TEMPLATE)

    # Create setup.zsh file
    setup_zsh_path = os.path.join(context.devel_space_abs, 'setup.zsh')
    with open(setup_zsh_path, 'w') as f:
        f.write(SETUP_ZSH_TEMPLATE)


def _create_unmerged_devel_setup_for_install(context):
    """Create non-functioning placeholder scripts in develspace."""
    for path in [os.path.join(context.devel_space_abs, f) for f in ['setup.sh', 'setup.bash', 'setup.zsh']]:
        with open(path, 'w') as f:
            f.write(SETUP_PLACEHOLDER_TEMPLATE)


SETUP_SH_TEMPLATE = """\
#!/usr/bin/env sh
# generated from within catkin_tools/verbs/catkin_build/build.py

# This file is aggregates the many setup.sh files in the various
# unmerged devel spaces in this folder.
# This is occomplished by sourcing each leaf package and all the
# recursive run dependencies of those leaf packages

# Source the first package's setup.sh without the --extend option
{first_source}

# remove all passed in args, resetting $@, $*, $#, $n
shift $#
# set the --extend arg for rest of the packages setup.sh's
set -- $@ "--extend"
# source setup.sh for each of the leaf packages in the workspace
{leaf_sources}

# And now the setup.sh for each of their recursive run dependencies
{run_depends_sources}
"""

SETUP_BASH_TEMPLATE = """\
#!/usr/bin/env bash
# generated from within catkin_tools/verbs/catkin_build/build.py

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_BUILD_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)
. "$_BUILD_SETUP_DIR/setup.sh"
"""

SETUP_ZSH_TEMPLATE = """\
#!/usr/bin/env zsh
# generated from within catkin_tools/verbs/catkin_build/build.py

CATKIN_SHELL=zsh

# source setup.sh from same directory as this file
_BUILD_SETUP_DIR=$(builtin cd -q "`dirname "$0"`" && pwd)
emulate sh # emulate POSIX
. "$_BUILD_SETUP_DIR/setup.sh"
emulate zsh # back to zsh mode
"""

SETUP_PLACEHOLDER_TEMPLATE = """\
#!/usr/bin/env sh
# generated from within catkin_tools/verbs/catkin_build/build.py

echo "Error: This workspace was built with the '--install' option."
echo "       You should source the setup files in the install space instead."
echo "       Your environment has not been changed."
"""
