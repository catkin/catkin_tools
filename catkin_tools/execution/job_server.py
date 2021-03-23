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

import array
import fcntl
import os
import re
import subprocess
import time
from multiprocessing import cpu_count
from tempfile import mkstemp
from termios import FIONREAD

from catkin_tools.common import log
from catkin_tools.common import version_tuple

from catkin_tools.terminal_color import ColorMapper

mapper = ColorMapper()
clr = mapper.clr


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


JOBSERVER_SUPPORT_MAKEFILE_OLD = b'''
all:
\techo $(MAKEFLAGS) | grep -- '--jobserver-fds'
'''

JOBSERVER_SUPPORT_MAKEFILE = b'''
all:
\techo $(MAKEFLAGS) | grep -- '--jobserver-auth'
'''


def test_gnu_make_support_common(makefile_content):
    """
    Test if "make -f MAKEFILE -j2" runs successfullyn when MAKEFILE
    contains makefile_content.
    """

    fd, makefile = mkstemp()
    os.write(fd, makefile_content)
    os.close(fd)

    ret = subprocess.call(['make', '-f', makefile, '-j2'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    os.unlink(makefile)
    return (ret == 0)


def test_gnu_make_support_old():
    """
    Test if the system 'make' supports the job server implementation.

    This simply checks if the `--jobserver-fds` option is supported by the
    `make` command. It does not tests if the jobserver is actually working
    properly.
    """

    return test_gnu_make_support_common(JOBSERVER_SUPPORT_MAKEFILE_OLD)


def test_gnu_make_support():
    """
    Test if the system 'make' supports the job server implementation.

    This simply checks if the `--jobserver-auth` option is supported by the
    `make` command. It does not tests if the jobserver is actually working
    properly.
    """

    return test_gnu_make_support_common(JOBSERVER_SUPPORT_MAKEFILE)


class GnuMake(object):
    def __init__(self):
        if test_gnu_make_support():
            self.make_args = lambda job_pipe: ["--jobserver-auth=%d,%d" % JobServer._job_pipe]
        elif test_gnu_make_support_old():
            self.make_args = lambda job_pipe: ["--jobserver-fds=%d,%d" % JobServer._job_pipe, "-j"]
        else:
            self.make_args = None

    def is_supported(self):
        return not (self.make_args is None)


class JobServer(object):
    # Whether the job server has been initialized
    _initialized = False

    # Flag designating whether the `make` program supports the GNU Make
    # jobserver interface
    _gnu_make = None

    # Initialize variables
    _load_ok = True
    _mem_ok = True
    _internal_jobs = []
    _max_load = 0
    _max_jobs = 0
    _job_pipe = os.pipe()

    # Setting fd inheritance is required in Python > 3.4
    # This is set by default in Python 2.7
    # For more info see: https://docs.python.org/3.4/library/os.html#fd-inheritance
    if hasattr(os, 'set_inheritable'):
        for fd in _job_pipe:
            os.set_inheritable(fd, True)
            if not os.get_inheritable(fd):
                log(clr('@{yf}@!Warning: jobserver file descriptors are not inheritable.@|'))

    @classmethod
    def _set_max_jobs(cls, max_jobs):
        """Set the maximum number of jobs to be used with the jobserver.

        This will wait for all active jobs to be completed, then re-initialize the job pipe.
        """

        # Read all possible tokens from the pipe
        try:
            os.read(cls._job_pipe[0], cls._max_jobs)
        except (BlockingIOError, InterruptedError):
            pass

        # Update max jobs
        cls._max_jobs = max_jobs

        # Initialize the pipe with max_jobs tokens
        for i in range(cls._max_jobs):
            os.write(cls._job_pipe[1], b'+')

    @classmethod
    def _set_max_mem(cls, max_mem):
        """
        Set the maximum memory to keep instantiating jobs.

        :param max_mem: String describing the maximum memory that can be used
        on the system. It can either describe memory percentage or absolute
        amount.  Use 'P%' for percentage or 'N' for absolute value in bytes,
        'Nk' for kilobytes, 'Nm' for megabytes, and 'Ng' for gigabytes.
        :type max_mem: str
        """

        if max_mem is None:
            cls._max_mem = None
            return
        elif type(max_mem) is float or type(max_mem) is int:
            mem_percent = max_mem
        elif type(max_mem) is str:
            m_percent = re.search(r'([0-9]+)\%', max_mem)
            m_abs = re.search(r'([0-9]+)([kKmMgG]{0,1})', max_mem)

            if m_percent is None and m_abs is None:
                cls._max_mem = None
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

        cls._max_mem = max(0.0, min(100.0, float(mem_percent)))

    @classmethod
    def _check_load(cls):
        if cls._max_load is not None:
            try:
                load = os.getloadavg()
                if load[0] < cls._max_load:
                    cls._load_ok = True
                else:
                    cls._load_ok = False
            except NotImplementedError:
                cls._load_ok = True

        return cls._load_ok

    @classmethod
    def _check_mem(cls):
        if cls._max_mem is not None:
            mem_used, mem_total = memory_usage()
            mem_percent_used = 100.0 * float(mem_used) / float(mem_total)
            if mem_percent_used > cls._max_mem:
                cls._mem_ok = False
            else:
                cls._mem_ok = True

        return cls._mem_ok

    @classmethod
    def _check_conditions(cls):
        return (cls._check_load() and cls._check_mem()) or cls._running_jobs() == 0

    @classmethod
    def _acquire(cls):
        """
        Obtain a job server token. Be sure to call _release() to avoid
        deadlocks.
        """
        try:
            # read a token from the job pipe
            token = os.read(cls._job_pipe[0], 1)
            return token
        except (BlockingIOError, InterruptedError):
            pass

        return None

    @classmethod
    def _release(cls):
        """
        Write a token to the job pipe.
        """
        os.write(cls._job_pipe[1], b'+')

    @classmethod
    def _running_jobs(cls):

        try:
            buf = array.array('i', [0])
            if fcntl.ioctl(cls._job_pipe[0], FIONREAD, buf) == 0:
                return cls._max_jobs - buf[0]
        except (NotImplementedError, OSError):
            pass

        return cls._max_jobs


def initialized():
    """Return True if the job server has been initialized."""
    return JobServer._initialized


def initialize(max_jobs=None, max_load=None, max_mem=None, gnu_make_enabled=False):
    """
    Initialize the global GNU Make jobserver.

    :param max_jobs: the maximum number of jobs available
    :param max_load: do not dispatch additional jobs if this system load
    value is exceeded
    :param max_mem: do not dispatch additional jobs if system physical
    memory usage exceeds this value (see _set_max_mem for additional
    documentation)
    """

    # Check initialization
    if JobServer._initialized is True:
        return

    # Check if the jobserver is supported
    if JobServer._gnu_make is None:
        JobServer._gnu_make = GnuMake()

    if not JobServer._gnu_make.is_supported():
        log(clr('@!@{yf}WARNING:@| Make job server not supported. The number of Make '
                'jobs may exceed the number of CPU cores.@|'))

    # Set gnu make compatibilty enabled
    JobServer._gnu_make_enabled = gnu_make_enabled

    # Set the maximum number of jobs
    if max_jobs is None:
        try:
            max_jobs = cpu_count()
        except NotImplementedError:
            log('@{yf}WARNING: Failed to determine the cpu_count, falling back to 1 jobs as the default.@|')
            max_jobs = 1
    else:
        max_jobs = int(max_jobs)

    JobServer._set_max_jobs(max_jobs)
    JobServer._max_load = max_load
    JobServer._set_max_mem(max_mem)

    JobServer._initialized = True


def load_ok():
    return JobServer._load_ok


def mem_ok():
    return JobServer._mem_ok


def set_max_mem(max_mem):
    """
    Set the maximum memory to keep instantiating jobs.

    :param max_mem: String describing the maximum memory that can be used on
    the system. It can either describe memory percentage or absolute amount.
    Use 'P%' for percentage or 'N' for absolute value in bytes, 'Nk' for
    kilobytes, 'Nm' for megabytes, and 'Ng' for gigabytes.
    :type max_mem: str
    """

    JobServer._set_max_mem(max_mem)


def wait_acquire():
    """
    Block until a job server token is acquired, then return it.
    """

    token = None

    while token is None:
        # make sure we're observing load and memory maximums
        if not JobServer._check_conditions():
            time.sleep(0.01)
            continue

        # try to get a job token
        token = JobServer._acquire()

    return token


def acquire():
    """
    Block until a job server token is acquired, then return it.
    """

    token = None

    # make sure we're observing load and memory maximums
    if JobServer._check_conditions():
        # try to get a job token
        token = JobServer._acquire()

    return token


def add_label(label):
    JobServer._internal_jobs.append(label)


def del_label(label):
    JobServer._internal_jobs.remove(label)


def try_acquire_gen():
    """
    Yield None until a job server token is acquired, then yield it.
    """
    while True:
        # make sure we're observing load and memory maximums
        if JobServer._check_conditions() and running_jobs() < max_jobs():
            # try to get a job token
            token = JobServer._acquire()
            yield token
        else:
            yield None


def try_acquire():
    """
    Try to acquire a job token, return None if not available.
    """
    # make sure we're observing load and memory maximums
    if JobServer._check_conditions() and running_jobs() < max_jobs():
        # try to get a job token
        token = JobServer._acquire()
        return token

    return None


def release(label=None):
    """
    Release a job server token.
    """
    JobServer._release()
    if label is not None:
        del_label(label)


def gnu_make_enabled():
    return JobServer._gnu_make.is_supported() and JobServer._gnu_make_enabled


def gnu_make_args():
    """
    Get required arguments for spawning child gnu Make processes.
    """

    if JobServer._gnu_make_enabled:
        return JobServer._gnu_make.make_args(JobServer._job_pipe)
    else:
        return []


def max_jobs():
    """
    Get the maximum number of jobs.
    """

    return JobServer._max_jobs


def running_jobs():
    """
    Try to estimate the number of currently running jobs.
    """

    if not gnu_make_enabled():
        return 0

    return JobServer._running_jobs()


def internal_jobs():
    return JobServer._internal_jobs


class JobGuard:

    """
    Context manager representing a jobserver job.
    """

    def __enter__(self):
        wait_acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        release()
        return False
