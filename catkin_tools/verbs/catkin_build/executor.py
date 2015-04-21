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

"""Executor implementation, these objects create threads and process jobs in them"""

from __future__ import print_function

from threading import Thread

from .color import colorize_cmake

from catkin_tools.common import remove_ansi_escape
from catkin_tools.runner import run_command

from catkin_tools.make_jobserver import jobserver_job


class ExecutorEvent(object):

    """This is returned by the Executor when an event occurs

    Events can be jobs starting/finishing, commands starting/failing/finishing,
    commands producing output (each line is an event), or when the executor
    quits or failes.
    """

    def __init__(self, executor_id, event_type, data, package):
        self.executor_id = executor_id
        self.event_type = event_type
        self.data = data
        self.package = package


class Executor(Thread):

    """Threaded executor for the parallel catkin build jobs"""
    name_prefix = 'build'

    def __init__(self, executor_id, context, comm_queue, job_queue, install_lock, continue_on_failure=False):
        super(Executor, self).__init__()
        self.name = self.name_prefix + '-' + str(executor_id + 1)
        self.executor_id = executor_id
        self.c = context
        self.queue = comm_queue
        self.jobs = job_queue
        self.current_job = None
        self.install_space_lock = install_lock
        self.shutdown_on_failure = not continue_on_failure
        self.should_shutdown = False

    def job_started(self, job):
        self.queue.put(ExecutorEvent(self.executor_id, 'job_started', {}, job.package.name))

    def command_started(self, cmd, location):
        package_name = '' if self.current_job is None else self.current_job.package.name
        data = {
            'cmd': cmd,
            'location': location
        }
        self.queue.put(ExecutorEvent(self.executor_id, 'command_started', data, package_name))

    def command_log(self, msg):
        package_name = '' if self.current_job is None else self.current_job.package.name
        data = {'message': msg}
        self.queue.put(ExecutorEvent(self.executor_id, 'command_log', data, package_name))

    def command_failed(self, cmd, location, retcode):
        package_name = '' if self.current_job is None else self.current_job.package.name
        data = {
            'cmd': cmd,
            'location': location,
            'retcode': retcode
        }
        self.queue.put(ExecutorEvent(self.executor_id, 'command_failed', data, package_name))

    def command_finished(self, cmd, location, retcode):
        package_name = '' if self.current_job is None else self.current_job.package.name
        data = {
            'cmd': cmd,
            'location': location,
            'retcode': retcode
        }
        self.queue.put(ExecutorEvent(self.executor_id, 'command_finished', data, package_name))

    def job_finished(self, job):
        self.queue.put(ExecutorEvent(self.executor_id, 'job_finished', {}, job.package.name))

    def job_failed(self, job):
        self.queue.put(ExecutorEvent(self.executor_id, 'job_failed', {}, job.package.name))

    def quit(self, exc=None):
        package_name = '' if self.current_job is None else self.current_job.package.name
        data = {
            'reason': 'normal' if exc is None else 'exception',
            'exc': str(exc)
        }
        self.queue.put(ExecutorEvent(self.executor_id, 'exit', data, package_name))

    def run(self):
        try:
            # Until exit
            while True:
                # Get a job off the queue
                self.current_job = self.jobs.get()
                # If the job is None, then we should shutdown
                if self.current_job is None:
                    # Notify shutdown
                    self.quit()
                    break
                # Notify that a new job was started
                self.job_started(self.current_job)

                # Track if the job has failed
                job_has_failed = False

                # Execute each command in the job
                with jobserver_job():
                    # Check here for externally set shutdown condition.
                    # This prevents trailing jobs when using the job server.
                    if self.should_shutdown:
                        self.quit()
                        return
                    for command in self.current_job:
                        install_space_locked = False
                        if command.lock_install_space:
                            self.install_space_lock.acquire()
                            install_space_locked = True
                        try:
                            # don't run further commands if previous one of this job failed
                            if job_has_failed:
                                break
                            # Log that the command being run
                            self.command_started(command, command.location)
                            # Receive lines from the running command
                            for line in run_command(command.cmd, cwd=command.location):
                                # If it is an integer, it corresponds to the command's return code
                                if isinstance(line, int):
                                    retcode = line
                                    # If the return code is not zero
                                    if retcode != 0:
                                        # Log the failure (the build loop will dispatch None's)
                                        self.command_failed(command, command.location, retcode)
                                        job_has_failed = True
                                        break
                                    else:
                                        self.command_finished(command, command.location, retcode)
                                else:
                                    # Otherwise it is some sort of string data
                                    # Ensure that the data is not just ansi escape characters
                                    if remove_ansi_escape(line).strip():
                                        for sub_line in line.splitlines(True):  # keepends=True
                                            if sub_line:
                                                if command.stage_name == 'cmake':
                                                    sub_line = colorize_cmake(sub_line)
                                                self.command_log(sub_line)
                        finally:
                            if install_space_locked:
                                self.install_space_lock.release()

                # Check if the job has failed
                if job_has_failed:
                    self.job_failed(self.current_job)
                    if self.shutdown_on_failure:
                        # Try to consume and throw away any and all remaining jobs in the queue
                        while self.jobs.get() is not None:
                            pass
                        # Once we get our None, quit
                        self.quit()
                        return
                else:
                    self.job_finished(self.current_job)

        except KeyboardInterrupt:
            self.quit()
        except Exception as exc:
            import traceback
            self.quit(traceback.format_exc() + str(exc))
            raise
