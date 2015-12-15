
from __future__ import print_function

from multiprocessing import cpu_count
from tempfile import mkstemp
from termios import FIONREAD

import array
import errno
import fcntl
import os
import re
import subprocess
import time

from catkin_tools.common import log
from catkin_tools.common import version_tuple

from catkin_tools.terminal_color import ColorMapper


mapper = ColorMapper()
clr = mapper.clr


class Job(object):

    """A Job is a series of operations, each of which is considered a "stage" of the job."""

    def __init__(self, jid, deps, env_loader, stages, continue_on_failure=True):
        """
        jid: Unique job identifier
        deps: Dependencies (in terms of other jid's)
        stages: List of stages to be run in order

        """
        self.jid = jid
        self.deps = deps
        self.env_loader = env_loader
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

    def getenv(self, env):
        return self.env_loader(env)
