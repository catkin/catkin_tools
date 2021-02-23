# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from catkin_tools.terminal_color import ColorMapper

mapper = ColorMapper()
clr = mapper.clr


class Job(object):

    """A Job is a series of operations, each of which is considered a "stage" of the job."""

    def __init__(self, jid, deps, env, stages, continue_on_failure=True):
        """
        jid: Unique job identifier
        deps: Dependencies (in terms of other jid's)
        stages: List of stages to be run in order

        """
        self.jid = jid
        self.deps = deps
        self.env = env
        self.stages = stages
        self.continue_on_failure = continue_on_failure

    def all_deps_completed(self, completed_jobs):
        """Return True if all dependencies have been completed."""
        return all([dep_id in completed_jobs for dep_id in self.deps])

    def all_deps_succeeded(self, completed_jobs):
        """Return True if all dependencies have been completed and succeeded."""
        return all([completed_jobs.get(dep_id, False) for dep_id in self.deps])

    def any_deps_failed(self, completed_jobs):
        """Return True if any dependencies which have been completed have failed."""
        return any([not completed_jobs.get(dep_id, True) for dep_id in self.deps])
