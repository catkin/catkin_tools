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
import traceback
import time
from queue import Queue

import pkg_resources
from catkin_pkg.package import InvalidPackage
from catkin_pkg.packages import find_packages
from catkin_pkg.topological_order import topological_order_packages

from catkin_tools.common import clr, wide_log, expand_glob_package
from catkin_tools.execution import job_server
from catkin_tools.execution.controllers import ConsoleStatusController
from catkin_tools.execution.executor import run_until_complete, execute_jobs


def test_workspace(
    context,
    packages=None,
    n_jobs=None,
    quiet=False,
    interleave_output=False,
    no_status=False,
    no_notify=False,
    continue_on_failure=False,
    summarize_build=False,
):
    """Tests a catkin workspace

    :param context: context in which to test the catkin workspace
    :type context: :py:class:`catkin_tools.context.Context`
    :param packages: list of packages to test
    :type packages: list
    :param n_jobs: number of parallel package test jobs
    :type n_jobs: int
    :param quiet: suppresses verbose build or test information
    :type quiet: bool
    :param interleave_output: prints the output of commands as they are received
    :type interleave_output: bool
    :param no_status: suppresses the bottom status line
    :type no_status: bool
    :param no_notify: suppresses system notifications
    :type no_notify: bool
    :param continue_on_failure: do not stop testing other packages on error
    :type continue_on_failure: bool
    :param summarize_build: summarizes the build at the end
    :type summarize_build: bool
    """
    pre_start_time = time.time()

    # Get all the packages in the context source space
    # Suppress warnings since this is a utility function
    try:
        workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])
    except InvalidPackage as ex:
        sys.exit(clr("@{rf}Error:@| The file %s is an invalid package.xml file."
                     " See below for details:\n\n%s" % (ex.package_path, ex.msg)))

    # Get all build type plugins
    test_job_creators = {
        ep.name: ep.load()['create_test_job']
        for ep in pkg_resources.iter_entry_points(group='catkin_tools.jobs')
    }

    # It's a problem if there aren't any build types available
    if len(test_job_creators) == 0:
        sys.exit('Error: No build types available. Please check your catkin_tools installation.')

    # Get list of packages to test
    ordered_packages = topological_order_packages(workspace_packages)
    workspace_packages = dict([(pkg.name, (path, pkg)) for path, pkg in ordered_packages])
    packages_to_test = []
    if packages:
        for package in packages:
            if package not in workspace_packages:
                # Try whether package is a pattern and matches
                glob_packages = expand_glob_package(package, workspace_packages)
                if len(glob_packages) > 0:
                    packages.extend(glob_packages)
                else:
                    sys.exit("[test] Given packages '{}' is not in the workspace "
                             "and pattern does not match any package".format(package))
        for pkg_path, package in ordered_packages:
            if package.name in packages:
                packages_to_test.append((pkg_path, package))
    else:
        # Only use whitelist when no other packages are specified
        if len(context.whitelist) > 0:
            # Expand glob patterns in whitelist
            whitelist = []
            for whitelisted_package in context.whitelist:
                whitelist.extend(expand_glob_package(whitelisted_package, workspace_packages))
            packages_to_test = [p for p in ordered_packages if (p[1].name in whitelist)]
        else:
            packages_to_test = ordered_packages

    # Filter packages on blacklist
    if len(context.blacklist) > 0:
        # Expand glob patterns in blacklist
        blacklist = []
        for blacklisted_package in context.blacklist:
            blacklist.extend(expand_glob_package(blacklisted_package, workspace_packages))
        # Apply blacklist to packages and dependencies
        packages_to_test = [
            (path, pkg) for path, pkg in packages_to_test
            if (pkg.name not in blacklist or pkg.name in packages)]

    # Check if all packages to test are already built
    built_packages = set([
        pkg.name for (path, pkg) in
        find_packages(context.package_metadata_path(), warnings=[]).items()])

    packages_to_test_names = set(pkg.name for path, pkg in packages_to_test)
    if not built_packages.issuperset(packages_to_test_names):
        wide_log(clr("@{rf}Error: Packages have to be built before they can be tested.@|"))
        wide_log(clr("The following requested packages are not built yet:"))
        for package_name in packages_to_test_names.difference(built_packages):
            wide_log(' - ' + package_name)
        sys.exit(1)

    # Construct jobs
    jobs = []
    for pkg_path, pkg in packages_to_test:
        # Determine the job parameters
        test_job_kwargs = dict(
            context=context,
            package=pkg,
            package_path=pkg_path)

        # Create the job based on the build type
        build_type = pkg.get_build_type()

        if build_type in test_job_creators:
            jobs.append(test_job_creators[build_type](**test_job_kwargs))

    # Queue for communicating status
    event_queue = Queue()

    # Initialize job server
    job_server.initialize(
        max_jobs=n_jobs,
        max_load=None,
        gnu_make_enabled=context.use_internal_make_jobserver,
    )

    try:
        # Spin up status output thread
        status_thread = ConsoleStatusController(
            'test',
            ['package', 'packages'],
            jobs,
            n_jobs,
            [pkg.name for path, pkg in packages_to_test],
            [p for p in context.whitelist],
            [p for p in context.blacklist],
            event_queue,
            show_notifications=not no_notify,
            show_active_status=not no_status,
            show_buffered_stdout=not interleave_output,
            show_buffered_stderr=not interleave_output,
            show_live_stdout=interleave_output,
            show_live_stderr=interleave_output,
            show_full_summary=summarize_build,
            show_stage_events=not quiet,
            pre_start_time=pre_start_time,
        )

        status_thread.start()

        locks = {}

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

        if all_succeeded:
            return 0
        else:
            return 1

    except KeyboardInterrupt:
        wide_log("[test] Interrupted by user!")
        event_queue.put(None)

        return 130
