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

import os
import stat
import subprocess
import sys
import tempfile

from multiprocessing import cpu_count

from catkin_tools.common import get_cached_recursive_build_depends_in_workspace

from catkin_tools.runner import run_command

# Due to portability issues, it uses only POSIX-compliant shell features.
# This means that there is no support for BASH-like arrays, and special
# care needs to be taken in order to preserve argument atomicity when
# passing along to the `exec` instruction at the end.
#
# This involves forming a string called `_ARGS` which is composed of
# tokens like `"$_Ai"` for i=0..N-1 for N arguments so that with N=3
# arguments, for example, `_ARGS` would look like `"$_A0" "$_A1" "$_A2"`.
# The double-quotes are necessary because they define the argument
# boundaries when the variables are expanded by calling `eval`.

env_file_template = """\
#!/usr/bin/env sh
# generated from within catkin_tools/verbs/catkin_build/common.py

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: build_env.sh COMMANDS"
  /bin/echo "Calling build_env.sh without arguments is not supported anymore."
  /bin/echo "Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# save original args for later
_ARGS=
_ARGI=0
for arg in "$@"; do
  # Define placeholder variable
  eval "_A$_ARGI=\$arg"
  # Add placeholder variable to arg list
  _ARGS="$_ARGS \\"\$_A$_ARGI\\""
  # Increment arg index
  _ARGI=`expr $_ARGI + 1`

  #######################
  ## Uncomment for debug:
  #_escaped="$(echo "$arg" | sed -e 's@ @ @g')"
  #echo "$_escaped"
  #eval "echo '$_ARGI \$_A$_ARGI'"
  #######################
done

#######################
## Uncomment for debug:
#echo "exec args:"
#echo "$_ARGS"
#for arg in $_ARGS; do eval echo $arg; done
#echo "-----------"
#####################

# remove all passed in args, resetting $@, $*, $#, $n
shift $#
# set the args for the sourced scripts
set -- $@ "--extend"
# source setup.sh with implicit --extend argument for each direct build depend in the workspace
{sources}

# execute given args
eval exec $_ARGS
"""


def generate_env_file(sources, env_file_path):
    env_file = env_file_template.format(sources='\n'.join(sources))
    with open(env_file_path, 'w') as f:
        f.write(env_file)
    # Make this file executable
    os.chmod(env_file_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)
    return env_file_path


def create_env_file(package, context):
    sources = []
    source_snippet = '. "{source_path}"'
    # If installing to isolated folders or not installing, but devel spaces are not merged
    if (context.install and context.isolate_install) or (not context.install and context.isolate_devel):
        # Source each package's install or devel space
        space = context.install_space_abs if context.install else context.devel_space_abs
        # Get the recursive dependcies
        depends = get_cached_recursive_build_depends_in_workspace(package, context.packages)
        # For each dep add a line to source its setup file
        for dep_pth, dep in depends:
            source_path = os.path.join(space, dep.name, 'setup.sh')
            sources.append(source_snippet.format(source_path=source_path))
    else:
        # Just source common install or devel space
        source_path = os.path.join(
            context.install_space_abs if context.install else context.devel_space_abs,
            'setup.sh')
        sources = [source_snippet.format(source_path=source_path)] if os.path.exists(source_path) else []
    # Build the env_file
    env_file_path = os.path.abspath(os.path.join(context.build_space_abs, package.name, 'build_env.sh'))
    generate_env_file(sources, env_file_path)
    return env_file_path


def create_build_space(buildspace, package_name):
    """Creates a build space, if it does not already exist, in the build space

    :param buildspace: folder in which packages are built
    :type buildspace: str
    :param package_name: name of the package this build space is for
    :type package_name: str
    :returns: package specific build directory
    :rtype: str
    """
    package_build_dir = os.path.join(buildspace, package_name)
    if not os.path.exists(package_build_dir):
        os.makedirs(package_build_dir)
    return package_build_dir


def get_build_type(package):
    """Returns the build type for a given package

    :param package: package object
    :type package: :py:class:`catkin_pkg.package.Package`
    :returns: build type of the package, e.g. 'catkin' or 'cmake'
    :rtype: str
    """
    export_tags = [e.tagname for e in package.exports]
    if 'build_type' in export_tags:
        build_type_tag = [e.content for e in package.exports if e.tagname == 'build_type'][0]
    else:
        build_type_tag = 'catkin'
    return build_type_tag


class Job(object):

    """Encapsulates a job which executes a series of commands"""

    def __init__(self, context):
        self.context = context
        self.commands = []
        self.__command_index = 0

    def get_commands(self):
        raise NotImplementedError('get_commands')

    def __iter__(self):
        return self

    def __next__(self):
        return self.next()

    def next(self):
        if self.__command_index >= len(self.commands):
            raise StopIteration()
        self.__command_index += 1
        return self.commands[self.__command_index - 1]


class BuildJob(Job):

    """Encapsulates a job which builds a package"""

    def __init__(self, context,  package, package_path, force_cmake):
        super(BuildJob, self).__init__(context)
        self.package = package
        self.package_name = package.name
        self.package_path = package_path
        self.force_cmake = force_cmake


class CleanJob(Job):

    """Encapsulates a job which cleans a package"""

    def __init__(self, context, package_name):
        super(CleanJob, self).__init__(context)
        self.package_name = package_name
