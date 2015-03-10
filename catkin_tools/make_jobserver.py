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

from __future__ import print_function

from multiprocessing import cpu_count
from tempfile import mkstemp
from termios import FIONREAD

import array
import fcntl
import os
import psutil
import subprocess
import time

from .common import log

JOBSERVER_SUPPORT_MAKEFILE = b'''
all:
\techo $(MAKEFLAGS) | grep -- '--jobserver-fds'
'''


class MakeJobServer:

    """
    This class implements a GNU make job server.

    TODO:
     - use os.getloadavg() to maintain server load
    """

    _singleton = None

    @staticmethod
    def get_instance(*args, **kwargs):
        if not MakeJobServer._singleton:
            MakeJobServer._singleton = MakeJobServer(*args, **kwargs)
        elif len(args) > 0 or len(kwargs) > 0:
            log('WARNING: Attempting to instantiate more than one jobserver!')

        return MakeJobServer._singleton

    def __init__(self, num_jobs=None, max_load=None, max_mem=None, enable=True):
        """
        :param num_jobs: the maximum number of jobs available
        :param max_load: do not dispatch additional jobs if this system load
        value is exceeded
        :param max_mem: do not dispatch additional jobs if system physical
        memory usage exceeds this value
        """
        assert(MakeJobServer._singleton is None)

        if enable:
            if not num_jobs:
                try:
                    num_jobs = cpu_count()
                except NotImplementedError:
                    log('@{yf}WARNING: Failed to determine the cpu_count, falling back to 1 jobs as the default.@|')
                    num_jobs = 1
            else:
                num_jobs = int(num_jobs)

            self.num_jobs = num_jobs
            self.max_load = max_load
            self.max_mem = max_mem
            self.job_pipe = os.pipe()

            # Initialize the pipe with num_jobs tokens
            for i in range(num_jobs):
                os.write(self.job_pipe[1], b'+')

            # Check if the jobserver is supported
            self.supported = self._testSupport()

        if not enable or not self.supported:
            log('@{yf}WARNING: Failed to determine the cpu_count, falling back to 1 jobs as the default.@|')
            self.num_jobs = 0
            self.job_pipe = None
            self.supported = False


    def _testSupport(self):
        """
        Test if the system 'make' supports the job server implementation
        """

        fd, makefile = mkstemp()
        os.write(fd, JOBSERVER_SUPPORT_MAKEFILE)
        os.close(fd)

        ret = subprocess.call(['make', '-f', makefile, '-j2'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        os.unlink(makefile)
        return (ret == 0)

    def make_arguments(self):
        """
        Get required arguments for spawning child make processes
        """

        return ["--jobserver-fds=%d,%d" % self.job_pipe, "-j"] if self.supported else []

    def num_running_jobs(self):
        """
        Try to estimate the number of currently running jobs
        """

        if not self.supported:
            return '?'

        try:
            buf = array.array('i', [0])
            if fcntl.ioctl(self.job_pipe[0], FIONREAD, buf) == 0:
                return self.num_jobs - buf[0]
        except NotImplementedError:
            pass
        except OSError:
            pass

        return self.num_jobs

    def obtain(self):
        """
        Obtain a job server token. Be sure to call release() to avoid
        deadlocks.
        """

        while True:
            # make sure we're observing load maximums
            if self.max_load is not None:
                try:
                    max_load = 8.0
                    load = os.getloadavg()
                    if self.num_running_jobs() > 0 and load[1] > self.max_load:
                        time.sleep(0.01)
                        continue
                except NotImplementedError:
                    pass

            # make sure we're observing memory maximum
            if self.max_mem is not None:
                mem_usage = psutil.phymem_usage()
                if self.num_running_jobs() > 0 and mem_usage.percent > self.max_mem:
                    time.sleep(0.01)
                    continue

            # get a token from the job pipe
            try:
                token = os.read(self.job_pipe[0], 1)
                return token
            except OSError as e:
                if e.errno != errno.EINTR:
                    raise

    def release(self):
        """
        Release a job server token.
        """

        os.write(self.job_pipe[1], b'+')

    def __enter__(self):
        if self.supported:
            self.obtain()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.supported:
            self.release()
        return False
