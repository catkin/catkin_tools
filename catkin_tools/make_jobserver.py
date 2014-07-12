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

from os import pipe, write, close, unlink, read
from multiprocessing import cpu_count
from tempfile import mkstemp
from subprocess import call, PIPE
from termios import FIONREAD

import fcntl
import array

# from .common import log

job_server_instance = None


class MakeJobServer:
    """
    This class implements a GNU make job server.
    """

    @staticmethod
    def get_instance():
        return job_server_instance

    def __init__(self, num_jobs=None):
        global job_server_instance
        assert(not job_server_instance)
        job_server_instance = self

        if not num_jobs:
            try:
                num_jobs = cpu_count()
            except NotImplementedError:
                # log('Failed to determine the cpu_count, falling back to 1 jobs as the default.')
                num_jobs = 1
        else:
            num_jobs = int(num_jobs)

        self.num_jobs = num_jobs
        self.pipe = pipe()

        # Initialize the pipe with num_jobs tokens
        for i in range(num_jobs):
            write(self.pipe[1], b'+')

        self.supported = self._testSupport()

    def _testSupport(self):
        """
        Test if the system 'make' supports the job server implementation
        """

        fd, makefile = mkstemp()
        write(fd, b'''
all:
\techo $(MAKEFLAGS) | grep -- '--jobserver-fds'
''')
        close(fd)

        ret = call(['make', '-f', makefile, '-j2'])

        unlink(makefile)
        return (ret == 0)

    def make_arguments(self):
        """
        Get required arguments for spawning child make processes
        """

        return ["--jobserver-fds=%d,%d" % self.pipe, "-j"]

    def num_running_jobs(self):
        """
        Try to estimate the number of currently running jobs
        """

        try:
            buf = array.array('i', [0])
            if fcntl.ioctl(self.pipe[0], FIONREAD, buf) == 0:
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
            try:
                token = read(self.pipe[0], 1)
                return token
            except OSError as e:
                if e.errno == errno.EINTR:
                    continue
                else:
                    raise

    def release(self):
        """
        Release a job server token.
        """

        write(self.pipe[1], b'+')

    def __enter__(self):
        if self.supported:
            self.obtain()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.supported:
            self.release()
        return False
