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

"""This modules implements the engine for cleaning packages in parallel"""

import pkg_resources
import sys
import time
import traceback
from queue import Queue


try:
    from catkin_pkg.packages import find_packages
    from catkin_pkg.topological_order import topological_order_packages
except ImportError as e:
    sys.exit(
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin_tools.execution.controllers import ConsoleStatusController
from catkin_tools.execution.executor import execute_jobs
from catkin_tools.execution.executor import run_until_complete

from catkin_tools.common import expand_glob_package
from catkin_tools.common import get_recursive_build_dependents_in_workspace
from catkin_tools.common import wide_log


def determine_packages_to_be_cleaned(context, include_dependents, packages):
    """Returns list of packages which should be cleaned, and those packages' deps.

    :param context: Workspace context
    :type context: :py:class:`catkin_tools.verbs.catkin_build.context.Context`
    :param packages: list of package names to be cleaned
    :type packages: list
    :returns: full list of package names to be cleaned
    :rtype: list
    """

    # Get all the cached packages in the context source space
    workspace_packages = find_packages(context.package_metadata_path(), exclude_subspaces=True, warnings=[])
    # Order the packages by topology
    ordered_packages = topological_order_packages(workspace_packages)

    # Create a dict of all packages in the workspace by name
    workspace_packages_by_name = dict([(pkg.name, (path, pkg)) for path, pkg in ordered_packages])

    # Initialize empty output
    packages_to_be_cleaned = set()

    # Expand glob patterns in packages
    expanded_packages = []
    for package_name in packages:
        expanded_packages.extend(expand_glob_package(package_name, workspace_packages_by_name))
    packages = expanded_packages

    # Expand metapackages into their constituents
    for package_name in packages:
        # This is ok if it's orphaned
        if package_name not in workspace_packages_by_name:
            packages_to_be_cleaned.add(package_name)
        else:
            # Get the package object
            package = workspace_packages_by_name[package_name][1]
            # If metapackage, include run depends which are in the workspace
            if 'metapackage' in [e.tagname for e in package.exports]:
                for rdep in package.run_depends:
                    if rdep.name in workspace_packages_by_name:
                        packages_to_be_cleaned.add(rdep.name)
            else:
                packages_to_be_cleaned.add(package_name)

    # Determine the packages that depend on the given packages
    if include_dependents:
        for package_name in list(packages_to_be_cleaned):
            # Get the packages that depend on the packages to be cleaned
            dependents = get_recursive_build_dependents_in_workspace(package_name, ordered_packages)
            packages_to_be_cleaned.update([pkg.name for _, pkg in dependents])

    return [workspace_packages_by_name[n] for n in packages_to_be_cleaned if n in workspace_packages_by_name]


def clean_packages(
        context,
        names_of_packages_to_be_cleaned,
        clean_dependents,
        verbose,
        dry_run):

    pre_start_time = time.time()

    # Update the names of packages to be cleaned with dependents
    packages_to_be_cleaned = determine_packages_to_be_cleaned(
        context,
        clean_dependents,
        names_of_packages_to_be_cleaned)

    # print(packages_to_be_cleaned)
    # for path, pkg in packages_to_be_cleaned:
    # if os.path.exists(os.path.join(context.build_space_abs, pkg.name)):
    # print("[clean] Cleaning package: %s" % pkg.name)

    # Construct jobs
    jobs = []
    for path, pkg in packages_to_be_cleaned:

        # Get all build type plugins
        clean_job_creators = {
            ep.name: ep.load()['create_clean_job']
            for ep in pkg_resources.iter_entry_points(group='catkin_tools.jobs')
        }

        # It's a problem if there aren't any build types available
        if len(clean_job_creators) == 0:
            sys.exit('Error: No build types availalbe. Please check your catkin_tools installation.')

        # Determine the job parameters
        clean_job_kwargs = dict(
            context=context,
            package=pkg,
            package_path=path,
            dependencies=[],  # Unused because clean jobs are not parallelized
            dry_run=dry_run,
            clean_build=True,
            clean_devel=True,
            clean_install=True)

        # Create the job based on the build type
        build_type = pkg.get_build_type()

        if build_type in clean_job_creators:
            jobs.append(clean_job_creators[build_type](**clean_job_kwargs))

    if len(jobs) == 0:
        print("[clean] There are no products from the given packages to clean.")
        return False

    # Queue for communicating status
    event_queue = Queue()

    # Spin up status output thread
    status_thread = ConsoleStatusController(
        'clean',
        ['package', 'packages'],
        jobs,
        1,
        [pkg.name for _, pkg in context.packages],
        [p for p in context.whitelist],
        [p for p in context.blacklist],
        event_queue,
        show_notifications=False,
        show_active_status=False,
        show_buffered_stdout=verbose or False,
        show_buffered_stderr=True,
        show_live_stdout=False,
        show_live_stderr=False,
        show_stage_events=False,
        show_full_summary=False,
        pre_start_time=pre_start_time,
        active_status_rate=10.0)
    status_thread.start()

    # Initialize locks (none need to be configured here)
    locks = {
    }

    # Block while running N jobs asynchronously
    try:
        ej = execute_jobs(
            'clean',
            jobs,
            locks,
            event_queue,
            context.log_space_abs,
            max_toplevel_jobs=1,
            continue_on_failure=True,
            continue_without_deps=False)
        all_succeeded = run_until_complete(ej)
    except Exception:
        status_thread.keep_running = False
        all_succeeded = False
        status_thread.join(1.0)
        wide_log(str(traceback.format_exc()))

    status_thread.join(1.0)

    return all_succeeded
