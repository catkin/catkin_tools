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

import os

from catkin_tools.terminal_color import ansi

from catkin_tools.common import remove_ansi_escape
from catkin_tools.common import wide_log

from .color import clr


class FileBackedLogCache(object):

    def __init__(self, package_name, log_dir, color):
        self.package = package_name
        self.log_dir = log_dir
        self.color = color
        self.log_path = os.path.join(log_dir, self.package + '.log')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        self.file_handle = open(self.log_path, 'w')
        self.current_cmd = None
        self.last_command_line = None
        self.current_line = 0

    def start_command(self, cmd, msg):
        self.last_command_line = self.current_line
        self.current_cmd = cmd
        self.append(msg.rstrip('\n') + '\n')

    def append(self, msg):
        self.file_handle.write(msg)
        self.file_handle.flush()
        self.current_line += 1

    def finish_command(self, msg):
        self.current_cmd = None
        self.append(msg.rstrip('\n') + '\n')

    def close(self):
        self.file_handle.close()
        self.file_handle = None

    def print_last_command_log(self):
        wide_log("")
        command_output = ""
        with open(self.log_path, 'r') as f:
            line_number = 0
            for line in f:
                if line_number is not None and line_number != self.last_command_line:
                    line_number += 1
                    continue
                line_number = None
                command_output += line
        if not self.color:
            wide_log(remove_ansi_escape(command_output), end='')
        else:
            wide_log(command_output, end='')
        wide_log("")


class OutputController(object):

    def __init__(self, log_dir, quiet, interleave_output, color, max_package_name_length, prefix_output=False):
        self.log_dir = log_dir
        self.quiet = quiet
        self.interleave = interleave_output
        self.color = color
        self.max_package_name_length = max_package_name_length
        self.prefix_output = prefix_output
        self.__command_log = {}

    def job_started(self, package):
        self.__command_log[package] = FileBackedLogCache(package, self.log_dir, self.color)
        wide_log(clr("Starting ==> {package}").format(**locals()))

    def command_started(self, package, cmd, location):
        if package not in self.__command_log:
            raise RuntimeError("Command started received for package '{0}' before package job started: '{1}' in '{2}'"
                               .format(package, cmd.pretty, location))
        msg = clr("[{package}] ==> '{cmd.cmd_str}' in '{location}'").format(**locals())
        self.__command_log[package].start_command(cmd, msg)
        if not self.quiet and self.interleave:
            wide_log(msg)

    def command_log(self, package, msg):
        if package not in self.__command_log:
            raise RuntimeError("Command log received for package '{0}' before package job started: '{1}'"
                               .format(package, msg))
        if self.__command_log[package].current_cmd is None:
            raise RuntimeError("Command log received for package '{0}' before command started: '{1}'"
                               .format(package, msg))
        self.__command_log[package].append(msg)
        if not self.color:
            msg = remove_ansi_escape(msg)
        if not self.quiet and self.interleave:
            msg = msg.rstrip(ansi('reset'))
            msg = msg.rstrip()
            if self.interleave and self.prefix_output:
                wide_log(clr("[{package}] {msg}").format(**locals()))
            else:
                wide_log(msg)

    def command_failed(self, package, cmd, location, retcode):
        if package not in self.__command_log:
            raise RuntimeError(
                "Command failed received for package '{0}' before package job started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd.pretty, location, retcode))
        if self.__command_log[package].current_cmd is None:
            raise RuntimeError(
                "Command failed received for package '{0}' before command started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd.pretty, location, retcode))
        msg = clr("[{package}] <== '{cmd.cmd_str}' failed with return code '{retcode}'").format(**locals())
        self.__command_log[package].finish_command(msg)
        if not self.interleave:
            self.__command_log[package].print_last_command_log()
        elif not self.quiet:
            wide_log(msg)

    def command_finished(self, package, cmd, location, retcode):
        if package not in self.__command_log:
            raise RuntimeError(
                "Command finished received for package '{0}' before package job started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd.pretty, location, retcode))
        if self.__command_log[package].current_cmd is None:
            raise RuntimeError(
                "Command finished received for package '{0}' before command started: '{1}' in '{2}' returned '{3}'"
                .format(package, cmd.pretty, location, retcode))
        msg = clr("[{package}] <== '{cmd.cmd_str}' finished with return code '{retcode}'").format(**locals())
        self.__command_log[package].finish_command(msg)
        if not self.quiet and not self.interleave:
            self.__command_log[package].print_last_command_log()
        elif not self.quiet:
            wide_log(msg)

    def job_finished(self, package, time):
        self.__command_log[package].close()
        del self.__command_log[package]
        msg = clr("Finished <== {package:<") + str(self.max_package_name_length) + clr("} [ {time} ]")
        wide_log(msg.format(**locals()))

    def job_failed(self, package, time):
        self.__command_log[package].close()
        del self.__command_log[package]
        msg = clr("Failed <== {package:<") + str(self.max_package_name_length) + clr("} [ {time} ]")
        wide_log(msg.format(**locals()))
