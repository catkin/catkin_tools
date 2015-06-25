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

import os
import sys
import time

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
        'ImportError: "from catkin_pkg.topological_order import '
        'topological_order" failed: %s\nMake sure that you have installed '
        '"catkin_pkg", and that it is up to date and on the PYTHONPATH.' % e
    )

from catkin_tools.execution.controllers import ConsoleStatusController
from catkin_tools.execution.executor import execute_jobs
from catkin_tools.execution.executor import run_until_complete
from catkin_tools.execution.jobs import JobServer

from catkin_tools.common import get_recursive_build_dependants_in_workspace
from catkin_tools.common import log
from catkin_tools.common import wide_log
from catkin_tools.common import get_linked_devel_package_path

from catkin_tools.jobs.catkin import create_catkin_clean_job
from catkin_tools.jobs.catkin import DEVEL_MANIFEST_FILENAME
from catkin_tools.jobs.cmake import create_cmake_clean_job
from catkin_tools.jobs.cmake import INSTALL_MANIFEST_FILENAME
from catkin_tools.jobs.job import create_clean_buildspace_job

from catkin_tools.execution.jobs import JobServer


def determine_packages_to_be_cleaned(context, include_dependants, packages):
    """Returns list of packages which should be cleaned, and those packages' deps.

    :param context: Workspace context
    :type context: :py:class:`catkin_tools.verbs.catkin_build.context.Context`
    :param packages: list of package names to be cleaned
    :type packages: list
    :returns: full list of package names to be cleaned
    :rtype: list
    """
    start = time.time()

    # Get all the packages in the context source space
    workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])
    # Order the packages by topology
    ordered_packages = topological_order_packages(workspace_packages)

    # Create a dict of all packages in the workspace by name
    workspace_package_names = dict([(pkg.name, (path, pkg)) for path, pkg in ordered_packages])

    # Initialize empty output
    packages_to_be_cleaned = set()

    # Expand metapackages into their constituents
    for package_name in packages:
        # This is ok if it's orphaned
        if package_name not in workspace_package_names:
            packages_to_be_cleaned.add(package_name)
        else:
            # Get the package object
            package = workspace_package_names[package_name][1]
            # If metapackage, include run depends which are in the workspace
            if 'metapackage' in [e.tagname for e in package.exports]:
                for rdep in package.run_depends:
                    if rdep.name in workspace_package_names:
                        packages_to_be_cleaned.add(rdep.name)
            else:
                packages_to_be_cleaned.add(package_name)

    # Determine the packages that depend on the given packages
    if include_dependants:
        for package_name in list(packages_to_be_cleaned):
            # Get the packages that depend on the packages to be cleaned
            dependants = get_recursive_build_dependants_in_workspace(package_name, ordered_packages)
            packages_to_be_cleaned.update([pkg.name for _, pkg in dependants])

    return list(packages_to_be_cleaned)


def get_clean_type(context, resultspace, pkg_name):
    """Determine type of package in order to clean it.

    TODO: Currently, this relies on heuristics, instead it should be explicit.
    """

    linked_devel_package_path = get_linked_devel_package_path(resultspace, pkg_name)

    if os.path.exists(linked_devel_package_path):
        if os.path.exists(os.path.join(linked_devel_package_path, DEVEL_MANIFEST_FILENAME)):
            return 'catkin'
        elif os.path.exists(os.path.join(linked_devel_package_path, INSTALL_MANIFEST_FILENAME)):
            return 'cmake'

    return None


def clean_packages(
        context,
        names_of_packages_to_be_cleaned,
        clean_dependants,
        verbose,
        dry_run):

    # Update the names of packages to be cleaned with dependants
    names_of_packages_to_be_cleaned = determine_packages_to_be_cleaned(
        context,
        clean_dependants,
        names_of_packages_to_be_cleaned)

    # print(packages_to_be_cleaned)
    # for path, pkg in packages_to_be_cleaned:
    # if os.path.exists(os.path.join(context.build_space_abs, pkg.name)):
    # print("[clean] Cleaning package: %s" % pkg.name)

    # Construct jobs
    jobs = []
    for pkg_name in names_of_packages_to_be_cleaned:

        # Create clean jobs for the develspace
        devel_clean_type = get_clean_type(context, context.devel_space_abs, pkg_name)
        install_clean_type = get_clean_type(context, context.install_space_abs, pkg_name)

        if devel_clean_type == 'catkin':
            jobs.append(create_catkin_clean_job(context, pkg_name, []))
        elif devel_clean_type == 'cmake':
            jobs.append(create_cmake_clean_job(context, pkg_name, []))
        elif install_clean_type == 'cmake':
            jobs.append(create_cmake_clean_job(context, pkg_name, []))
        elif os.path.exists(os.path.join(context.build_space_abs, pkg_name)):
            jobs.append(create_clean_buildspace_job(context, pkg_name, []))

    if len(jobs) == 0:
        print("[clean] There are no products from the given packages to clean.")
        return False

    # Just list the packages to be cleaned
    if dry_run:
        log('[clean] The products from the following packages will be cleaned:')
        for job in jobs:
            log(' - {}'.format(job.jid))
        return False

    # Initialize jobserver with 1 job
    JobServer.initialize(max_jobs=1)

    # Queue for communicating status
    event_queue = Queue()

    try:
        # Spin up status output thread
        status_thread = ConsoleStatusController(
            'clean',
            ['package', 'packages'],
            jobs,
            event_queue,
            show_stage_events=verbose)
        status_thread.start()

        # Block while running N jobs asynchronously
        run_until_complete(execute_jobs(
            'clean',
            jobs,
            event_queue,
            os.path.join(context.build_space_abs, '_logs'),
            continue_on_failure=True,
            continue_without_deps=False))

        status_thread.join()

    except KeyboardInterrupt:
        wide_log("[build] User interrupted!")
        event_queue.put(None)

    return True
