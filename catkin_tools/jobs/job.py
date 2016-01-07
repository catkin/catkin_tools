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

from catkin_tools.common import mkdir_p
from catkin_tools.common import get_cached_recursive_build_depends_in_workspace

from catkin_tools.resultspace import get_resultspace_environment

from catkin_tools.execution.jobs import Job
from catkin_tools.execution.stages import CommandStage

from .commands.cmake import CMAKE_EXEC

# Build Environment File
# =============
#
# The Build Environment file is used to create environments to packages built
# in an isolated build scenario. This enables packages to build against other
# packages without sourcing the main workspace setup.sh file.
#
# Due to portability issues, it uses only POSIX-compliant shell features. This
# means that there is no support for BASH-like arrays, and special care needs
# to be taken in order to preserve argument atomicity when passing along to the
# `exec` instruction at the end.
#
# This involves forming a string called `_ARGS` which is composed of tokens
# like `"$_Ai"` for i=0..N-1 for N arguments so that with N=3 arguments, for
# example, `_ARGS` would look like `"$_A0" "$_A1" "$_A2"`.  The double-quotes
# are necessary because they define the argument boundaries when the variables
# are expanded by calling `eval`.

ENV_FILE_NAME = 'build_env.sh'

ENV_FILE_TEMPLATE = """\
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


def get_env_file_path(package, context):
    """Get the path to a package's build environment file."""

    return os.path.abspath(os.path.join(context.build_space_abs, package.name, ENV_FILE_NAME))


def get_env_loaders(package, context):
    """Get a list of env loaders required to build this package."""

    sources = []
    # If installing to isolated folders or not installing, but devel spaces are not merged
    if (context.install and context.isolate_install) or (not context.install and context.isolate_devel):
        # Source each package's install or devel space
        space = context.install_space_abs if context.install else context.devel_space_abs
        # Get the recursive dependcies
        depends = get_cached_recursive_build_depends_in_workspace(package, context.packages)
        # For each dep add a line to source its setup file
        for dep_pth, dep in depends:
            source_path = os.path.join(space, dep.name, 'env.sh')
            sources.append(source_path)
    else:
        # Get the actual destination of this package
        source_path = os.path.join(context.package_dest_path(package), 'env.sh')
        sources = [source_path]

    return sources


def get_env_loader(package, context):
    """This function returns a function object which extends a base environment
    based on a set of environments to load."""

    def load_env(base_env):
        # Copy the base environment to extend
        job_env = dict(base_env)
        # Get the paths to the env loaders
        env_loader_paths = get_env_loaders(package, context)
        # If DESTDIR is set, set _CATKIN_SETUP_DIR as well
        if context.destdir is not None:
            job_env['_CATKIN_SETUP_DIR'] = context.package_dest_path(package)

        for env_loader_path in env_loader_paths:
            # print(' - Loading resultspace env from: {}'.format(env_loader_path))
            resultspace_env = get_resultspace_environment(
                os.path.split(env_loader_path)[0],
                base_env=job_env,
                quiet=True,
                cached=True,
                strict=False)
            job_env.update(resultspace_env)

        return job_env

    return load_env


def create_env_file(logger, event_queue, package, context, env_file_path):
    """FunctionStage functor for creating a build environment file."""

    source_paths = get_env_loaders(package, context)

    source_snippet = '. "{source_path}"'
    sources = [source_snippet.format(source_path=source_path) for source_path in source_paths]

    # Populate the build env file template and write it out
    env_file = ENV_FILE_TEMPLATE.format(sources='\n'.join(sources))
    if os.path.exists(env_file_path):
        with open(env_file_path, 'r') as f:
            if env_file == f.read():
                return 0
    with open(env_file_path, 'w') as f:
        f.write(env_file)

    # Make the env file executable
    os.chmod(env_file_path, stat.S_IXUSR | stat.S_IWUSR | stat.S_IRUSR)

    return 0


def get_package_build_space_path(buildspace, package_name):
    """Generates a build space path, does not modify the filesystem.

    TODO: Move to common.py
    TODO: Get buildspace from context
    TODO: Make arguments the same order as get_env_file_path

    :param buildspace: folder in which packages are built
    :type buildspace: str
    :param package_name: name of the package this build space is for
    :type package_name: str
    :returns: package specific build directory
    :rtype: str
    """
    return os.path.join(buildspace, package_name)


def create_build_space(logger, event_queue, buildspace, package_name):
    """Creates a build space, if it does not already exist, in the build space

    :param buildspace: folder in which packages are built
    :type buildspace: str
    :param package_name: name of the package this build space is for
    :type package_name: str
    :returns: package specific build directory
    :rtype: str
    """
    package_build_dir = get_package_build_space_path(buildspace, package_name)
    if not os.path.exists(package_build_dir):
        os.makedirs(package_build_dir)
    return package_build_dir


def makedirs(logger, event_queue, path):
    """FunctionStage functor that makes a path of directories."""
    mkdir_p(path)
    return 0


def get_build_type(package):
    """Returns the build type for a given package.

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


def create_clean_buildspace_job(context, package_name, dependencies):

    build_space = get_package_build_space_path(context.build_space_abs, package_name)
    if not os.path.exists(build_space):
        # No-op
        return Job(jid=package_name, deps=dependencies, stages=[])

    stages = []

    stages.append(CommandStage(
        'rmbuild',
        [CMAKE_EXEC, '-E', 'remove_directory', build_space],
        cwd=context.build_space_abs))

    return Job(
        jid=package_name,
        deps=dependencies,
        stages=stages)
