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
import stat
import sys
import time
import traceback
import yaml

try:
    # Python3
    from queue import Queue
except ImportError:
    # Python2
    from Queue import Queue

try:
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'Importing "catkin_pkg" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin_pkg.package import parse_package

from catkin_tools.common import format_time_delta
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace
from catkin_tools.common import get_recursive_run_depends_in_workspace
from catkin_tools.common import log
from catkin_tools.common import wide_log

from catkin_tools.execution.controllers import ConsoleStatusController
from catkin_tools.execution.executor import execute_jobs
from catkin_tools.execution.executor import run_until_complete

from catkin_tools.jobs.catkin import create_catkin_build_job
from catkin_tools.jobs.cmake import create_cmake_build_job
from catkin_tools.jobs.job import get_build_type

from .color import clr


BUILDSPACE_MARKER_FILE = '.catkin_tools.yaml'
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
        sys.exit("[build] No packages were found in the source space '{0}'".format(context.source_space_abs))
    wide_log("[build] Found '{0}' packages in {1}."
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
                sys.exit("[build] Given package '{0}' is not in the workspace".format(package))
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


def print_error_summary(errors, no_notify, log_dir):
    wide_log(clr("[build] There were '" + str(len(errors)) + "' @!@{rf}errors@|:"))
    if not no_notify:
        notify("Build Failed", "there were {0} errors".format(len(errors)), icon_image='catkin_icon_red.png')
    for error in errors:
        if error.event_type == 'exit':
            wide_log("""\
Executor '{exec_id}' had an unhandled exception while processing package '{package}':

{data[exc]}""".format(exec_id=error.executor_id + 1, **error.__dict__))
        else:
            wide_log(clr("""
@{rf}Failed@| to build package '@{cf}{package}@|' because the following command:

@!@{kf}# Command to reproduce:@|
cd {location} && {cmd.cmd_str}; cd -

@!@{kf}# Path to log:@|
cat {log_dir}

@{rf}Exited@| with return code: @!{retcode}@|""").format(package=error.package,
                                                         log_dir=os.path.join(log_dir, error.package + '.log'),
                                                         **error.data))


def print_items_in_columns(items_in, number_of_columns):
    number_of_items_in_line = 0
    line_template = "{}" * number_of_columns
    line_items = []
    items = list(items_in)
    while items:
        line_items.append(items.pop(0))
        number_of_items_in_line += 1
        if number_of_items_in_line == number_of_columns:
            wide_log(line_template.format(*line_items))
            line_items = []
            number_of_items_in_line = 0
    if line_items:
        wide_log(("{}" * len(line_items)).format(*line_items))


def print_build_summary(context, packages_to_be_built, completed_packages, failed_packages):
    # Calculate the longest package name
    max_name_len = max([len(pkg.name) for _, pkg in context.packages])

    def get_template(template_name, column_width):
        templates = {
            'successful': " @!@{gf}Successful@| @{cf}{package:<" + str(column_width) + "}@|",
            'failed': " @!@{rf}Failed@|     @{cf}{package:<" + str(column_width) + "}@|",
            'not_built': " @!@{kf}Not built@|  @{cf}{package:<" + str(column_width) + "}@|",
        }
        return templates[template_name]

    # Setup templates for comparison
    successful_template = get_template('successful', max_name_len)
    failed_template = get_template('failed', max_name_len)
    not_built_template = get_template('not_built', max_name_len)
    # Calculate the maximum _printed_ length for each template
    faux_package_name = ("x" * max_name_len)
    templates = [
        remove_ansi_escape(clr(successful_template).format(package=faux_package_name)),
        remove_ansi_escape(clr(failed_template).format(package=faux_package_name)),
        remove_ansi_escape(clr(not_built_template).format(package=faux_package_name)),
    ]
    # Calculate the longest column using the longest template
    max_column_len = max([len(template) for template in templates])
    # Calculate the number of columns
    number_of_columns = (terminal_width() / max_column_len) or 1

    successfuls = {}
    faileds = {}
    not_builts = {}
    non_whitelisted = {}
    blacklisted = {}

    for (_, pkg) in context.packages:
        if pkg.name in context.blacklist:
            blacklisted[pkg.name] = clr(not_built_template).format(package=pkg.name)
        elif len(context.whitelist) > 0 and pkg.name not in context.whitelist:
            non_whitelisted[pkg.name] = clr(not_built_template).format(package=pkg.name)
        elif pkg.name in completed_packages:
            successfuls[pkg.name] = clr(successful_template).format(package=pkg.name)
        else:
            if pkg.name in failed_packages:
                faileds[pkg.name] = clr(failed_template).format(package=pkg.name)
            else:
                not_builts[pkg.name] = clr(not_built_template).format(package=pkg.name)

    # Combine successfuls and not_builts, sort by key, only take values
    wide_log("")
    wide_log("Build summary:")
    combined = dict(successfuls)
    combined.update(not_builts)
    non_failed = [v for k, v in sorted(combined.items(), key=operator.itemgetter(0))]
    print_items_in_columns(non_failed, number_of_columns)

    # Print out whitelisted packages
    if len(non_whitelisted) > 0:
        wide_log("")
        wide_log("Non-Whitelisted Packages:")
        non_whitelisted_list = [v for k, v in sorted(non_whitelisted.items(), key=operator.itemgetter(0))]
        print_items_in_columns(non_whitelisted_list, number_of_columns)

    # Print out blacklisted packages
    if len(blacklisted) > 0:
        wide_log("")
        wide_log("Blacklisted Packages:")
        blacklisted_list = [v for k, v in sorted(blacklisted.items(), key=operator.itemgetter(0))]
        print_items_in_columns(blacklisted_list, number_of_columns)

    # Faileds only, sort by key, only take values
    failed = [v for k, v in sorted(faileds.items(), key=operator.itemgetter(0))]
    if len(failed) > 0:
        wide_log("")
        wide_log("Failed packages:")
        print_items_in_columns(failed, number_of_columns)
    else:
        wide_log("")
        wide_log("All packages built successfully.")

    wide_log("")
    wide_log(clr("[build] @!@{gf}Successfully@| built '@!@{cf}{0}@|' packages, "
                 "@!@{rf}failed@| to build '@!@{cf}{1}@|' packages, "
                 "and @!@{kf}did not try to build@| '@!@{cf}{2}@|' packages.").format(
        len(successfuls), len(faileds), len(not_builts)
    ))


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

    # Summarize the context
    summary_notes = []
    if force_cmake:
        summary_notes += [clr("@!@{cf}NOTE:@| Forcing CMake to run for each package.")]
    log(context.summary(summary_notes))

    # Make sure there is a build folder and it is not a file
    if os.path.exists(context.build_space_abs):
        if os.path.isfile(context.build_space_abs):
            sys.exit(clr(
                "[build] @{rf}Error:@| Build space '{0}' exists but is a file and not a folder."
                .format(context.build_space_abs)))
    # If it dosen't exist, create it
    else:
        log("[build] Creating build space: '{0}'".format(context.build_space_abs))
        os.makedirs(context.build_space_abs)

    # Write the current build config for config error checking
    with open(os.path.join(context.build_space_abs, BUILDSPACE_MARKER_FILE), 'w') as buildspace_marker_file:
        buildspace_marker_file.write(yaml.dump(buildspace_marker_data, default_flow_style=False))

    # Get all the packages in the context source space
    # Suppress warnings since this is a utility function
    workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])

    # Set of unbuilt packages
    unbuilt_pkgs = set()

    # Look for packages without build directories
    for path, pkg in workspace_packages.items():
        if 'metapackage' not in [e.tagname for e in pkg.exports]:
            if not os.path.exists(os.path.join(context.build_space_abs, pkg.name)):
                unbuilt_pkgs.add(pkg.name)

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
    except AttributeError as err:
        log(clr("[build] @!@{rf}Error:@| The workspace packages have a circular "
                "dependency, and cannot be built. Please run `catkin list "
                "--deps` to determine the problematic package(s)."))
        return

    # Check the number of packages to be built
    if len(packages_to_be_built) == 0:
        log(clr('[build] No packages to be built.'))
        return

    # Assert start_with package is in the workspace
    verify_start_with_option(
        start_with,
        packages,
        all_packages,
        packages_to_be_built + packages_to_be_built_deps)

    # Remove packages before start_with
    if start_with is not None:
        for path, pkg in list(packages_to_be_built):
            if pkg.name != start_with:
                wide_log("[build] Skipping package '{0}'".format(pkg.name))
                packages_to_be_built.pop(0)
            else:
                break

    # Construct jobs
    jobs = []
    packages_to_be_built_names = [p.name for _, p in packages_to_be_built]
    for pkg_path, pkg in all_packages:
        if pkg.name == 'catkin':
            continue
        if pkg.name not in packages_to_be_built_names:
            continue
        # Ignore metapackages
        if 'metapackage' in [e.tagname for e in pkg.exports]:
            continue

        # Get actual execution deps
        deps = [p.name for _, p in get_cached_recursive_build_depends_in_workspace(
            pkg, packages_to_be_built) if p.name != 'catkin']

        # Create the job based on the build type
        build_type = get_build_type(pkg)
        build_job_kwargs = dict(
            context=context,
            package=pkg,
            package_path=pkg_path,
            dependencies=deps,
            force_cmake=force_cmake,
            pre_clean=pre_clean)
        build_job = None

        for entry_point in pkg_resources.iter_entry_points(group='catkin_tools.jobs'):
            if build_type == entry_point.name:
                loaded_ep = entry_point.load()
                build_job = loaded_ep['create_build_job'](**build_job_kwargs)

        if build_job is not None:
            jobs.append(build_job)
        else:
            wide_log(clr("[build] @!@{yf}Warning:@| Skipping package '{}'"
                         " because it has an unknown package build type: \"{}\"").format(
                pkg.name, build_type))

    # Queue for communicating status
    event_queue = Queue()

    try:
        # Spin up status output thread
        status_thread = ConsoleStatusController(
            'build',
            ['package', 'packages'],
            jobs,
            event_queue,
            show_notifications=not no_notify,
            show_active_status=not no_status,
            show_buffered_stdout=not quiet,
            show_stage_events=not quiet,
            show_full_summary=(summarize_build is True),
            pre_start_time=pre_start_time,
            active_status_rate=limit_status_rate)
        status_thread.start()

        # Block while running N jobs asynchronously
        try:
            all_succeeded = run_until_complete(execute_jobs(
                'build',
                jobs,
                event_queue,
                os.path.join(context.build_space_abs, '_logs'),
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
        if len(unbuilt_pkgs) > 0:
            log(clr("[build] @/@!Note:@| @/Workspace packages have changed, "
                    "please re-source setup files to use them.@|"))

        if all_succeeded:
            # Create isolated devel setup if necessary
            if context.isolate_devel:
                if not context.install:
                    _create_unmerged_devel_setup(context)
                else:
                    _create_unmerged_devel_setup_for_install(context)
            return 0
        else:
            return 1

    except KeyboardInterrupt:
        wide_log("[build] Interrupted by user!")
        event_queue.put(None)


def _create_unmerged_devel_setup(context):
    # Find all of the leaf packages in the workspace
    # where leaf means that nothing in the workspace depends on it

    # Find all packages in the source space
    # Suppress warnings since this is an internal function whose goal is not to
    # give feedback on the user's packages
    workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])

    ordered_packages = topological_order_packages(workspace_packages)
    workspace_packages = dict([(p.name, p) for pth, p in workspace_packages.items()])
    dependencies = set([])
    for name, pkg in workspace_packages.items():
        dependencies.update([d.name for d in pkg.buildtool_depends + pkg.build_depends + pkg.run_depends])
    leaf_packages = []
    for name, pkg in workspace_packages.items():
        if pkg.name not in dependencies:
            leaf_packages.append(pkg.name)
    assert leaf_packages, leaf_packages  # Defensive, there should always be at least one leaf
    leaf_sources = []
    for pkg_name in leaf_packages:
        source_path = os.path.join(context.devel_space_abs, pkg_name, 'setup.sh')
        if os.path.isfile(source_path):
            leaf_sources.append('. {0}'.format(source_path))
    # In addition to the leaf packages, we need to source the recursive run depends of the leaf packages
    run_depends = get_recursive_run_depends_in_workspace(
        [workspace_packages[p] for p in leaf_packages], ordered_packages)
    run_depends_sources = []
    for run_dep_name in [p.name for pth, p in run_depends]:
        source_path = os.path.join(context.devel_space_abs, run_dep_name, 'setup.sh')
        if os.path.isfile(source_path):
            run_depends_sources.append('. {0}'.format(source_path))
    # Create the setup.sh file
    setup_sh_path = os.path.join(context.devel_space_abs, 'setup.sh')
    env_file = """\
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
""".format(
        first_source=leaf_sources[0],
        leaf_sources='\n'.join(leaf_sources[1:]),
        run_depends_sources='\n'.join(run_depends_sources)
    )
    with open(setup_sh_path, 'w') as f:
        f.write(env_file)
    # Make this file executable
    os.chmod(setup_sh_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)
    # Create the setup.bash file
    setup_bash_path = os.path.join(context.devel_space_abs, 'setup.bash')
    with open(setup_bash_path, 'w') as f:
        f.write("""\
#!/usr/bin/env bash
# generated from within catkin_tools/verbs/catkin_build/build.py

CATKIN_SHELL=bash

# source setup.sh from same directory as this file
_BUILD_SETUP_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" && pwd)
. "$_BUILD_SETUP_DIR/setup.sh"
""")
    # Make this file executable
    os.chmod(setup_bash_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)
    setup_zsh_path = os.path.join(context.devel_space_abs, 'setup.zsh')
    with open(setup_zsh_path, 'w') as f:
        f.write("""\
#!/usr/bin/env zsh
# generated from within catkin_tools/verbs/catkin_build/build.py

CATKIN_SHELL=zsh

# source setup.sh from same directory as this file
_BUILD_SETUP_DIR=$(builtin cd -q "`dirname "$0"`" && pwd)
emulate sh # emulate POSIX
. "$_BUILD_SETUP_DIR/setup.sh"
emulate zsh # back to zsh mode
""")
    # Make this file executable
    os.chmod(setup_zsh_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)


def _create_unmerged_devel_setup_for_install(context):
    for path in [os.path.join(context.devel_space_abs, f) for f in ['setup.sh', 'setup.bash', 'setup.zsh']]:
        with open(path, 'w') as f:
            f.write("""\
#!/usr/bin/env sh
# generated from within catkin_tools/verbs/catkin_build/build.py

echo "Error: This workspace was built with the '--install' option."
echo "       You should source the setup files in the install space instead."
echo "       Your environment has not been changed."
""")
