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

"""This modules provides a portable, failsafe notification function"""

import os
import platform
import subprocess

from catkin_tools.utils import which

this_dir = os.path.dirname(__file__)


def _notify_osx(title, msg, icon_image):
    # Note: icon_image is unused on OS X. Maybe it would make more sense
    # to pass a boolean `success' and then let the platform specific
    # notification implementations decide how that translates to icons
    # or what not.
    app_path = os.path.join(this_dir, 'resources', 'osx', 'catkin build.app')
    open_exec = which('open')
    if open_exec is None:
        return
    command = [open_exec, app_path, '--args', title, msg]
    terminal = os.environ.get('TERM_PROGRAM', None)
    if terminal == "Apple_Terminal":
        command += ["-activate", "com.apple.Terminal"]
    elif terminal == "iTerm.app":
        command += ["-activate", "com.googlecode.iterm2"]
    subprocess.Popen(command,
                     stdout=subprocess.PIPE,
                     stderr=subprocess.PIPE)


def _notify_linux(title, msg, icon_image):
    icon_path = os.path.join(this_dir, 'resources', 'linux', icon_image)
    notify_send_exec = which('notify-send')
    if notify_send_exec is None:
        return
    subprocess.Popen([notify_send_exec, '-i', icon_path, '-t', '2000', '--hint', 'int:transient:1', title, msg],
                     stdout=subprocess.PIPE,
                     stderr=subprocess.PIPE)


def notify(title, msg, icon_image='catkin_icon.png'):
    if platform.system() == 'Darwin':
        return _notify_osx(title, msg, icon_image=icon_image)
    if platform.system() == 'Linux':
        return _notify_linux(title, msg, icon_image=icon_image)
