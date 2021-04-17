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
from queue import Queue

import pkg_resources
from catkin_pkg.package import InvalidPackage
from catkin_pkg.packages import find_packages
from catkin_pkg.topological_order import topological_order_packages

from catkin_tools.common import clr, wide_log
from catkin_tools.execution import job_server
from catkin_tools.execution.controllers import ConsoleStatusController
from catkin_tools.execution.executor import run_until_complete, execute_jobs


def test_workspace(
    context,
    packages=None,
):
    """Tests a catkin workspace

    :param context: context in which to test the catkin workspace
    :type context: :py:class:`catkin_tools.context.Context`
    :param packages: list of packages to test
    :type packages: list
    """
    # Get all the packages in the context source space
    # Suppress warnings since this is a utility function
    try:
        workspace_packages = find_packages(context.source_space_abs, exclude_subspaces=True, warnings=[])
    except InvalidPackage as ex:
        sys.exit(clr("@{rf}Error:@| The file %s is an invalid package.xml file."
                     " See below for details:\n\n%s" % (ex.package_path, ex.msg)))

    ordered_packages = topological_order_packages(workspace_packages)
    n_jobs = 1

    # Get all build type plugins
    test_job_creators = {
        ep.name: ep.load()['create_test_job']
        for ep in pkg_resources.iter_entry_points(group='catkin_tools.jobs')
    }

    # It's a problem if there aren't any build types available
    if len(test_job_creators) == 0:
        sys.exit('Error: No build types available. Please check your catkin_tools installation.')

    # Construct jobs
    jobs = []
    for pkg_path, pkg in ordered_packages:
        if pkg.name not in packages:
            continue

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
        max_jobs=1,
        max_load=None,
    )

    try:
        # Spin up status output thread
        status_thread = ConsoleStatusController(
            'test',
            ['package', 'packages'],
            jobs,
            n_jobs,
            [pkg.name for _, pkg in context.packages],
            [p for p in context.whitelist],
            [p for p in context.blacklist],
            event_queue,
            show_live_stdout=True,
            show_live_stderr=True,
            show_buffered_stdout=False,
            show_buffered_stderr=False,
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