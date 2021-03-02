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

import os
import traceback
from shlex import quote as cmd_quote

from .io import IOBufferLogger
from .io import IOBufferProtocol


class Stage(object):

    """A description of one of the serially-executed stages of a Job.

    Like Jobs, Stages are stateless, and simply describe what needs to be done
    and how to do it.
    """

    def __init__(
            self,
            label,
            logger_factory=IOBufferProtocol.factory,
            occupy_job=True,
            locked_resource=None,
            repro=None):
        self.label = str(label)
        self.logger_factory = logger_factory
        self.occupy_job = occupy_job
        self.repro = repro
        self.locked_resource = locked_resource

    def get_reproduction_cmd(self, verb, jid):
        """Get a command line to reproduce this stage with the proper environment."""
        return self.repro


class CommandStage(Stage):

    """Job stage that describes a system command.
    """

    def __init__(
            self,
            label,
            cmd,
            cwd=None,
            env=None,
            env_overrides=None,
            shell=False,
            emulate_tty=True,
            stderr_to_stdout=False,
            occupy_job=True,
            locked_resource=None,
            logger_factory=IOBufferProtocol.factory):
        """
        :param label: The label for the stage
        :param command: A list of strings composing a system command
        :param protocol: A protocol class to use for this stage

        :parma cmd: The command to run
        :param cwd: The directory in which to run the command (default: os.getcwd())
        :param env: The base environment. (default: {})
        :param env_overrides: The variables that override the base environment. (default: {})
        :param shell: Whether to run the command in "shell" mode. (default: False)
        :param emulate_tty: Support colored output (default: True)
        :param stderr_to_stdout: Pipe stderr to stdout (default: False)

        :param occupy_job: Whether this stage should wait for a worker from the job server (default: True)
        :param logger_factory: The factory to use to construct a logger (default: IOBufferProtocol.factory)
        """

        if not type(cmd) in [list, tuple] or not all([isinstance(s, str) for s in cmd]):
            raise ValueError('Command stage must be a list of strings: {}'.format(cmd))
        super(CommandStage, self).__init__(label, logger_factory, occupy_job, locked_resource)

        # Store environment overrides
        self.env_overrides = env_overrides or {}

        # Override base environment
        env = env or {}
        env.update(self.env_overrides)

        self.async_execute_process_kwargs = {
            'cmd': cmd,
            'cwd': cwd or os.getcwd(),
            'env': env,
            'shell': shell,
            # Emulate tty for cli colors
            'emulate_tty': emulate_tty,
            # Capture stderr and stdout separately
            'stderr_to_stdout': stderr_to_stdout,
        }

    def update_env(self, base_env):
        """Update the environment for this stage with the env args passed to the constructor."""
        self.async_execute_process_kwargs['env'].update(base_env)
        self.async_execute_process_kwargs['env'].update(self.env_overrides)

    def get_reproduction_cmd(self, verb, jid):
        """Get a command line to reproduce this stage with the proper environment."""

        # Define the base env command
        get_env_cmd = 'catkin {} --get-env {}'.format(verb, jid)

        # Add additional env args
        env_overrides_formatted = ' '.join([
            '{}={}'.format(k, cmd_quote(v))
            for k, v in self.env_overrides.items()
        ])

        # Define the actual command to reproduce
        cmd_str = ' '.join([cmd_quote(t) for t in self.async_execute_process_kwargs['cmd']])

        # Define the command to run the subcommand
        env_cmd = 'catkin env -si {} {}'.format(
            env_overrides_formatted,
            cmd_str)

        # Return the full command
        return 'cd {}; {} | {}; cd -'.format(
            self.async_execute_process_kwargs['cwd'],
            get_env_cmd,
            env_cmd)


class FunctionStage(Stage):

    """Job stage that describes a python function.

    :param label: The label for the stage
    :param function: A python function which returns 0 on success

    Functions must take the arguments:
        - logger
        - event_queue
    """

    def __init__(
            self,
            label,
            function,
            logger_factory=IOBufferLogger.factory,
            occupy_job=True,
            locked_resource=None,
            *args,
            **kwargs):
        if not callable(function):
            raise ValueError('Function stage must be callable.')
        super(FunctionStage, self).__init__(label, logger_factory, occupy_job, locked_resource)

        self.args = args
        self.kwargs = kwargs

        def function_proxy(logger, event_queue):
            try:
                return function(logger, event_queue, *args, **kwargs)
            except:  # noqa: E722
                # Silencing E722 here since we immediately re-raise the exception.
                logger.err(str(traceback.format_exc()))
                raise
        self.function = function_proxy
