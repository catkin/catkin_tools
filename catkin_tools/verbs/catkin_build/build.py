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

import operator
import os
import stat
import sys
import time
import yaml

try:
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin_tools.common import format_time_delta
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace
from catkin_tools.common import get_recursive_run_depends_in_workspace
from catkin_tools.common import log
from catkin_tools.common import wide_log

from catkin_tools.jobs.catkin import CatkinBuildJob
from catkin_tools.jobs.cmake import CMakeBuildJob
from catkin_tools.jobs.executor import execute_jobs
from catkin_tools.jobs.job import get_build_type

from .color import clr


BUILDSPACE_MARKER_FILE = '.catkin_tools.yaml'
DEVELSPACE_MARKER_FILE = '.catkin_tools.yaml'


def determine_packages_to_be_built(packages, context):
    """Returns list of packages which should be built, and those package's deps.

    :param packages: list of packages to be built, if None all packages are built
    :type packages: list
    :param context: Workspace context
    :type context: :py:class:`catkin_tools.verbs.catkin_build.context.Context`
    :returns: tuple of packages to be built and those package's deps
    :rtype: tuple
    """
    start = time.time()

    # Get all the packages in the context source space
    # Suppress warnings since this is a utility function
    workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])

    # If there are no packages raise
    if not workspace_packages:
        sys.exit("No packages were found in the source space '{0}'".format(context.source_space_abs))
    log("Found '{0}' packages in {1}."
        .format(len(workspace_packages), format_time_delta(time.time() - start)))

    # Order the packages by topology
    ordered_packages = topological_order_packages(workspace_packages)
    # Set the packages in the workspace for the context
    context.packages = ordered_packages
    # Determin the packages which should be built
    packages_to_be_built = []
    packages_to_be_built_deps = []

    # Determine the packages to be built
    if packages:
        # First assert all of the packages given are in the workspace
        workspace_package_names = dict([(pkg.name, (path, pkg)) for path, pkg in ordered_packages])
        for package in packages:
            if package not in workspace_package_names:
                sys.exit("Given package '{0}' is not in the workspace".format(package))
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
            packages_to_be_built = [p for p in ordered_packages if (p[1].name in context.whitelist)]
        else:
            packages_to_be_built = ordered_packages

    # Filter packages with blacklist
    if len(context.blacklist) > 0:
        packages_to_be_built = [
            (path, pkg) for path, pkg in packages_to_be_built
            if (pkg.name not in context.blacklist or pkg.name in packages)]
        packages_to_be_built_deps = [
            (path, pkg) for path, pkg in packages_to_be_built_deps
            if (pkg.name not in context.blacklist or pkg.name in packages)]
        ordered_packages = ordered_packages

    return packages_to_be_built, packages_to_be_built_deps, ordered_packages


def verify_start_with_option(start_with, packages, all_packages, packages_to_be_built):
    if start_with is not None:
        if start_with not in [pkg.name for pth, pkg in all_packages]:
            sys.exit("Package given for --start-with, '{0}', is not in the workspace.".format(start_with))
        elif start_with not in [pkg.name for pth, pkg in packages_to_be_built]:
            sys.exit("Package given for --start-with, '{0}', "
                     "is in the workspace but would not be built with given package arguments: '{1}'"
                     .format(start_with, ' '.join(packages)))


def build_job_factory(context, path, package, force_cmake):
    job = None
    build_type = get_build_type(package)
    if build_type == 'catkin':
        job = CatkinBuildJob(context, package, path, force_cmake)
    elif build_type == 'cmake':
        job = CMakeBuildJob(context, package, path, force_cmake)
    return job


def build_isolated_workspace(
    context,
    packages=None,
    start_with=None,
    no_deps=False,
    jobs=None,
    force_cmake=False,
    force_color=False,
    quiet=False,
    interleave_output=False,
    no_status=False,
    limit_status_rate=0.0,
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
    :param jobs: number of parallel package build jobs
    :type jobs: int
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
    # Assert that the limit_status_rate is valid
    if limit_status_rate < 0:
        sys.exit("The value of --status-rate must be greater than or equal to zero.")
    # If no_deps is given, ensure packages to build are provided
    if no_deps and packages is None:
        sys.exit("With --no-deps, you must specify packages to build.")
    # Make sure there is a build folder and it is not a file
    if os.path.exists(context.build_space_abs):
        if os.path.isfile(context.build_space_abs):
            sys.exit(clr(
                "@{rf}Error:@| Build space '{0}' exists but is a file and not a folder."
                .format(context.build_space_abs)))
    # If it dosen't exist, create it
    else:
        log("Creating build space directory, '{0}'".format(context.build_space_abs))
        os.makedirs(context.build_space_abs)

    # Check for catkin_make droppings
    if context.corrupted_by_catkin_make():
        sys.exit(
            clr("@{rf}Error:@| Build space `{0}` exists but appears to have previously been "
                "created by the `catkin_make` or `catkin_make_isolated` tool. "
                "Please choose a different directory to use with `catkin build` "
                "or clean the build space.").format(context.build_space_abs))

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
            existing_buildspace_marker_data = yaml.load(buildspace_marker_file)
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

    # Write the current build config for config error checking
    with open(os.path.join(context.build_space_abs, BUILDSPACE_MARKER_FILE), 'w') as buildspace_marker_file:
        buildspace_marker_file.write(yaml.dump(buildspace_marker_data, default_flow_style=False))

    # Summarize the context
    summary_notes = []
    if force_cmake:
        summary_notes += [clr("@!@{cf}NOTE:@| Forcing CMake to run for each package.")]
    log(context.summary(summary_notes))

    # Find list of packages in the workspace
    packages_to_be_built, packages_to_be_built_deps, all_packages = determine_packages_to_be_built(packages, context)
    if not no_deps:
        # Extend packages to be built to include their deps
        packages_to_be_built.extend(packages_to_be_built_deps)
    # Also re-sort
    packages_to_be_built = topological_order_packages(dict(packages_to_be_built))
    # Check the number of packages to be built
    if len(packages_to_be_built) == 0:
        log(clr('[build] No packages to be built.'))
        return

    # Assert start_with package is in the workspace
    verify_start_with_option(start_with, packages, all_packages, packages_to_be_built + packages_to_be_built_deps)

    # Remove packages before start_with
    if start_with is not None:
        for path, pkg in packages_to_be_built:
            if pkg.name != start_with:
                wide_log("[build] Skipping package '{0}'".format(pkg.name))
                packages_to_be_built.pop(0)
            else:
                break

    execute_jobs(
        'build',
        context,
        jobs,
        build_job_factory,
        packages_to_be_built,
        force_cmake,
        force_color,
        quiet,
        interleave_output,
        no_status,
        limit_status_rate,
        lock_install,
        no_notify,
        continue_on_failure,
        summarize_build)
