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
import errno
import fcntl
import os
import re
import subprocess
import time

from catkin_tools.common import log
from catkin_tools.common import version_tuple

JOBSERVER_SUPPORT_MAKEFILE = b'''
all:
\techo $(MAKEFLAGS) | grep -- '--jobserver-fds'
'''


def memory_usage():
    """
    Get used and total memory usage.

    :returns: Used and total memory in bytes
    :rtype: tuple
    """

    # Handle optional psutil support
    try:
        import psutil

        psutil_version = version_tuple(psutil.__version__)
        if psutil_version < (0, 6, 0):
            usage = psutil.phymem_usage()
            used = usage.used
        else:
            usage = psutil.virtual_memory()
            used = usage.total - usage.available

        return used, usage.total

    except ImportError:
        pass

    return None, None


class _MakeJobServer:
    """
    This class implements a GNU make job server.
    """

    # Singleton jobserver
    _singleton = None

    def __init__(self, num_jobs=None, max_load=None, max_mem=None):
        """
        :param num_jobs: the maximum number of jobs available
        :param max_load: do not dispatch additional jobs if this system load
        value is exceeded
        :param max_mem: do not dispatch additional jobs if system physical
        memory usage exceeds this value (see _set_max_mem for additional
        documentation)
        """

        assert(_MakeJobServer._singleton is None)

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
        self._set_max_mem(max_mem)

        self.job_pipe = os.pipe()

        # Initialize the pipe with num_jobs tokens
        for i in range(num_jobs):
            os.write(self.job_pipe[1], b'+')

    def _set_max_mem(self, max_mem):
        """
        Set the maximum memory to keep instantiating jobs.

        :param max_mem: String describing the maximum memory that can be used
        on the system. It can either describe memory percentage or absolute
        amount.  Use 'P%' for percentage or 'N' for absolute value in bytes,
        'Nk' for kilobytes, 'Nm' for megabytes, and 'Ng' for gigabytes.
        :type max_mem: str
        """

        if max_mem is None:
            self.max_mem = None
            return
        elif type(max_mem) is float or type(max_mem) is int:
            mem_percent = max_mem
        elif type(max_mem) is str:
            m_percent = re.search('([0-9]+)\%', max_mem)
            m_abs = re.search('([0-9]+)([kKmMgG]{0,1})', max_mem)

            if m_percent is None and m_abs is None:
                self.max_mem = None
                return

            if m_percent:
                mem_percent = m_abs.group(1)
            elif m_abs:
                val = float(m_abs.group(1))
                mag_symbol = m_abs.group(2)

                _, total_mem = memory_usage()

                if mag_symbol == '':
                    mag = 1.0
                elif mag_symbol.lower() == 'k':
                    mag = 1024.0
                elif mag_symbol.lower() == 'm':
                    mag = pow(1024.0, 2)
                elif mag_symbol.lower() == 'g':
                    mag = pow(1024.0, 3)

                mem_percent = 100.0 * val * mag / total_mem

        self.max_mem = max(0.0, min(100.0, float(mem_percent)))

    def _obtain(self):
        """
        Obtain a job server token. Be sure to call _release() to avoid
        deadlocks.
        """

        while True:
            # make sure we're observing load maximums
            if self.max_load is not None:
                try:
                    load = os.getloadavg()
                    if jobserver_running_jobs() > 0 and load[1] > self.max_load:
                        time.sleep(0.01)
                        continue
                except NotImplementedError:
                    pass

            # make sure we're observing memory maximum
            if self.max_mem is not None:
                mem_used, mem_total = memory_usage()
                mem_percent_used = 100.0 * float(mem_used) / float(mem_total)
                if jobserver_running_jobs() > 0 and mem_percent_used > self.max_mem:
                    time.sleep(0.01)
                    continue

            # get a token from the job pipe
            try:
                token = os.read(self.job_pipe[0], 1)
                return token
            except OSError as e:
                if e.errno != errno.EINTR:
                    raise

    def _release(self):
        """
        Release a job server token.
        """

        os.write(self.job_pipe[1], b'+')


class _MakeJob:
    """
    Context manager representing a jobserver job.
    """

    def __enter__(self):
        if _MakeJobServer._singleton is not None:
            _MakeJobServer._singleton._obtain()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if _MakeJobServer._singleton is not None:
            _MakeJobServer._singleton._release()
        return False


def _test_support():
    """
    Test if the system 'make' supports the job server implementation.
    """

    fd, makefile = mkstemp()
    os.write(fd, JOBSERVER_SUPPORT_MAKEFILE)
    os.close(fd)

    ret = subprocess.call(['make', '-f', makefile, '-j2'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    os.unlink(makefile)
    return (ret == 0)


def initialize_jobserver(*args, **kwargs):
    """
    Initialize the global GNU Make jobserver.

    :param num_jobs: the maximum number of jobs available
    :param max_load: do not dispatch additional jobs if this system load
    value is exceeded
    :param max_mem: do not dispatch additional jobs if system physical
    memory usage exceeds this value
    """

    assert(_MakeJobServer._singleton is None)

    # Check if the jobserver is supported
    supported = _test_support()

    if not supported:
        _MakeJobServer._singleton = None
        log('@{yf}WARNING: Make job server not supported. The number of Make '
            'jobs may exceed the number of CPU cores.@|')
        return

    # Create the jobserver singleton
    _MakeJobServer._singleton = _MakeJobServer(*args, **kwargs)


def jobserver_job():
    """
    Get a job from the jobserver.

    This is meant to be used with a context manager.
    """
    return _MakeJob()


def jobserver_arguments():
    """
    Get required arguments for spawning child make processes.
    """

    if _MakeJobServer._singleton is not None:
        return ["--jobserver-fds=%d,%d" % _MakeJobServer._singleton.job_pipe, "-j"]
    else:
        return []


def jobserver_running_jobs():
    """
    Try to estimate the number of currently running jobs.
    """

    if _MakeJobServer._singleton is None:
        return '?'

    try:
        buf = array.array('i', [0])
        if fcntl.ioctl(_MakeJobServer._singleton.job_pipe[0], FIONREAD, buf) == 0:
            return _MakeJobServer._singleton.num_jobs - buf[0]
    except NotImplementedError:
        pass
    except OSError:
        pass

    return _MakeJobServer._singleton.num_jobs


def jobserver_max_jobs():
    """
    Get the maximum number of jobs.
    """

    if _MakeJobServer._singleton is not None:
        return _MakeJobServer._singleton.num_jobs
    else:
        return 0


def jobserver_supported():
    """
    Returns true if the jobserver exists.
    """
    return _MakeJobServer._singleton is not None


def set_jobserver_max_mem(max_mem):
    """
    Set the maximum memory to keep instantiating jobs.

    :param max_mem: String describing the maximum memory that can be used on
    the system. It can either describe memory percentage or absolute amount.
    Use 'P%' for percentage or 'N' for absolute value in bytes, 'Nk' for
    kilobytes, 'Nm' for megabytes, and 'Ng' for gigabytes.
    :type max_mem: str
    """

    if _MakeJobServer._singleton:
        _MakeJobServer._singleton._set_max_mem(max_mem)
