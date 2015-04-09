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

"""This module implements a class for representing a catkin workspace context"""

from __future__ import print_function

import os
import re
import sys

from . import metadata

from .common import getcwd
from .common import printed_fill
from .common import remove_ansi_escape
from .common import terminal_width

from .metadata import find_enclosing_workspace

from .resultspace import get_resultspace_environment

from .terminal_color import ColorMapper

color_mapper = ColorMapper()
clr = color_mapper.clr


class Context(object):

    """Encapsulates a catkin workspace's settings which affect build results.

    This class will validate some of the settings on assignment using the
    filesystem, but it will never modify the filesystem. For instance, it will
    raise an exception if the source space does not exist, but it will not
    create a folder for the build space if it does not already exist.

    This context can be locked, so that changing the members is prevented.
    """

    DEFAULT_SOURCE_SPACE = 'src'
    DEFAULT_BUILD_SPACE = 'build'
    DEFAULT_DEVEL_SPACE = 'devel'
    DEFAULT_INSTALL_SPACE = 'install'

    STORED_KEYS = ['extend_path',
                   'source_space',
                   'build_space',
                   'devel_space',
                   'install_space',
                   'isolate_devel',
                   'install',
                   'isolate_install',
                   'cmake_args',
                   'make_args',
                   'use_internal_make_jobserver',
                   'catkin_make_args',
                   'whitelist',
                   'blacklist']

    KEYS = STORED_KEYS + [
        'workspace',
        'profile',
        'space_suffix']

    @classmethod
    def load(
        cls,
        workspace_hint=None,
        profile=None,
        opts=None,
        strict=False,
        append=False,
        remove=False,
        load_env=True
    ):
        """Load a context from a given workspace and profile with optional
        modifications.

        This function will try to load a given context from the specified
        workspace with the following resolution strategy:
         - existing workspace enclosing given workspace path
         - existing workspace enclosing "."
         - given workspace path
         - "."

        If a workspace cannot be found, it will assume that the user is
        specifying a new workspace, unless `strict=True` is given. In this
        latter case, this function will return None.

        :param workspace_hint: The hint used to find a workspace (see description for more details)
        :type workspace_hint: str
        :param profile: The profile to load the context from, if the profile is None, the active profile is used
        :type profile: str
        :param opts: An argparse options namespace containing context keys to override stored context keys
        :type opts: namespace
        :param strict: Causes this function to return None if a workspace isn't found
        :type strict: bool
        :param append: Appends any list-type opts to existing opts
        :type append: bool
        :param remove: Removes any list-type opts from existing opts
        :type remove: bool
        :param load_env: Control whether the context loads the resultspace
        environment for the full build context
        :type load_env: bool

        :returns: A potentially valid Context object constructed from the given arguments
        :rtype: Context
        """

        # Initialize dictionary version of opts namespace
        opts_vars = vars(opts) if opts else {}

        # Get the workspace (either the given directory or the enclosing ws)
        workspace_hint = workspace_hint or opts_vars.get('workspace', None) or getcwd()
        workspace = find_enclosing_workspace(workspace_hint)
        if not workspace:
            if strict or not workspace_hint:
                return None
            else:
                workspace = workspace_hint
        opts_vars['workspace'] = workspace

        # Get the active profile
        profile = profile or opts_vars.get('profile', None) or metadata.get_active_profile(workspace)
        opts_vars['profile'] = profile

        # Initialize empty metadata/args
        config_metadata = {}
        context_args = {}

        # Get the metadata stored in the workspace if it was found
        if workspace:
            config_metadata = metadata.get_metadata(workspace, profile, 'config')
            context_args.update(config_metadata)

        # User-supplied args are used to update stored args
        # Only update context args with given opts which are not none
        for (k, v) in opts_vars.items():
            if k in Context.KEYS and v is not None:
                # Handle list-type arguments with append/remove functionality
                if type(context_args.get(k, None)) is list and type(v) is list:
                    if append:
                        context_args[k] += v
                    elif remove:
                        context_args[k] = [w for w in context_args[k] if w not in v]
                    else:
                        context_args[k] = v
                else:
                    context_args[k] = v

        # Create the build context
        ctx = Context(**context_args)

        # Don't load the cmake config if it's not needed
        if load_env:
            ctx.load_env()

        return ctx

    @classmethod
    def save(cls, context):
        """Save a context in the associated workspace and profile."""
        metadata.update_metadata(
            context.workspace,
            context.profile,
            'config',
            context.get_stored_dict())

    def get_stored_dict(self):
        """Get the context parameters which should be stored persistently."""
        return dict([(k, getattr(self, k)) for k in Context.STORED_KEYS])

    def __init__(
        self,
        workspace=None,
        profile=None,
        extend_path=None,
        source_space=None,
        build_space=None,
        devel_space=None,
        install_space=None,
        isolate_devel=False,
        install=False,
        isolate_install=False,
        cmake_args=None,
        make_args=None,
        use_internal_make_jobserver=True,
        catkin_make_args=None,
        space_suffix=None,
        whitelist=None,
        blacklist=None
    ):
        """Creates a new Context object, optionally initializing with parameters

        :param workspace: root of the workspace, defaults to the enclosing workspace
        :type workspace: str
        :param profile: profile name, defaults to the default profile
        :type profile: str
        :param extend_path: catkin result-space to extend
        :type extend_path: str
        :param source_space: relative location of source space, defaults to '<workspace>/src'
        :type source_space: str
        :param build_space: relativetarget location of build space, defaults to '<workspace>/build'
        :type build_space: str
        :param devel_space: relative target location of devel space, defaults to '<workspace>/devel'
        :type devel_space: str
        :param install_space: relative target location of install space, defaults to '<workspace>/install'
        :type install_space: str
        :param isolate_devel: each package will have its own develspace if True, default is False
        :type isolate_devel: bool
        :param install: packages will be installed by invoking ``make install``, defaults to False
        :type install: bool
        :param isolate_install: packages will be installed to separate folders if True, defaults to False
        :type isolate_install: bool
        :param cmake_args: extra cmake arguments to be passed to cmake for each package
        :type cmake_args: list
        :param make_args: extra make arguments to be passed to make for each package
        :type make_args: list
        :param use_internal_make_jobserver: true if this configuration should use an internal make jobserv
        :type use_internal_make_jobserver: bool
        :param catkin_make_args: extra make arguments to be passed to make for each catkin package
        :type catkin_make_args: list
        :param space_suffix: suffix for build, devel, and install spaces which are not explicitly set.
        :type space_suffix: str
        :param whitelist: a list of packages to build by default
        :type whitelist: list
        :param blacklist: a list of packages to ignore by default
        :type blacklist: list
        :raises: ValueError if workspace or source space does not exist
        """
        self.__locked = False

        # Validation is done on assignment
        # Handle *space assignment and defaults
        self.workspace = workspace

        self.extend_path = extend_path if extend_path else None
        ss = '' if space_suffix is None else space_suffix

        self.profile = profile

        self.source_space = Context.DEFAULT_SOURCE_SPACE if source_space is None else source_space
        self.build_space = Context.DEFAULT_BUILD_SPACE + ss if ss or build_space is None else build_space
        self.devel_space = Context.DEFAULT_DEVEL_SPACE + ss if ss or devel_space is None else devel_space
        self.install_space = Context.DEFAULT_INSTALL_SPACE + ss if ss or install_space is None else install_space
        self.destdir = os.environ['DESTDIR'] if 'DESTDIR' in os.environ else None

        # Handle package whitelist/blacklist
        self.whitelist = whitelist or []
        self.blacklist = blacklist or []

        # Handle build options
        self.isolate_devel = isolate_devel
        self.install = install
        self.isolate_install = isolate_install

        # Handle additional cmake and make arguments
        self.cmake_args = cmake_args or []
        self.make_args = make_args or []
        self.use_internal_make_jobserver = use_internal_make_jobserver
        self.catkin_make_args = catkin_make_args or []

        # List of packages in the workspace is set externally
        self.packages = []

        # List of warnings about the workspace is set internally
        self.warnings = []

        # Initialize environment settings set by load_env
        self.manual_cmake_prefix_path = None
        self.cached_cmake_prefix_path = None
        self.env_cmake_prefix_path = None
        self.cmake_prefix_path = None

    def load_env(self):

        # Check for CMAKE_PREFIX_PATH in manual cmake args
        self.manual_cmake_prefix_path = ''
        for cmake_arg in self.cmake_args:
            prefix_path_match = re.findall('-DCMAKE_PREFIX_PATH.*?=(.+)', cmake_arg)
            if len(prefix_path_match) > 0:
                self.manual_cmake_prefix_path = prefix_path_match[0]

        # Load and update mirror of 'sticky' CMake information
        if self.install:
            sticky_env = get_resultspace_environment(self.install_space_abs, quiet=True)
        else:
            sticky_env = get_resultspace_environment(self.devel_space_abs, quiet=True)

        self.cached_cmake_prefix_path = ''
        if 'CMAKE_PREFIX_PATH' in sticky_env:
            split_result_cmake_prefix_path = sticky_env.get('CMAKE_PREFIX_PATH', '').split(':')
            if len(split_result_cmake_prefix_path) > 1:
                self.cached_cmake_prefix_path = ':'.join(split_result_cmake_prefix_path[1:])

        # Either load an explicit environment or get it from the current environment
        self.env_cmake_prefix_path = ''
        if self.extend_path:
            extended_env = get_resultspace_environment(self.extend_path, quiet=False)
            self.env_cmake_prefix_path = extended_env.get('CMAKE_PREFIX_PATH', '')
            if not self.env_cmake_prefix_path:
                print(clr("@!@{rf}Error:@| Could not load environment from workspace: '%s', "
                          "target environment (env.sh) does not provide 'CMAKE_PREFIX_PATH'" % self.extend_path))
                print(extended_env)
                sys.exit(1)
        else:
            # Get the current CMAKE_PREFIX_PATH
            if 'CMAKE_PREFIX_PATH' in os.environ:
                split_result_cmake_prefix_path = os.environ['CMAKE_PREFIX_PATH'].split(':')
                if len(split_result_cmake_prefix_path) > 1 and (
                        (not self.install and split_result_cmake_prefix_path[0] == self.devel_space_abs) or
                        (self.install and split_result_cmake_prefix_path[0] == self.install_space_abs)):

                    self.env_cmake_prefix_path = ':'.join(split_result_cmake_prefix_path[1:])
                else:
                    self.env_cmake_prefix_path = os.environ.get('CMAKE_PREFIX_PATH', '')

        # Add warnings based on conflicing CMAKE_PREFIX_PATH
        if self.cached_cmake_prefix_path and self.extend_path:
            ep_not_in_lcpp = any([self.extend_path in p for p in self.cached_cmake_prefix_path.split(':')])
            if not ep_not_in_lcpp:
                self.warnings += [clr(
                    "Your workspace is configured to explicitly extend a "
                    "workspace which yields a CMAKE_PREFIX_PATH which is "
                    "different from the cached CMAKE_PREFIX_PATH used last time "
                    "this workspace was built.\\n\\n"
                    "If you want to use a different CMAKE_PREFIX_PATH you "
                    "should call @{yf}`catkin clean --all`@| to remove all "
                    "references to the previous CMAKE_PREFIX_PATH.\\n\\n"
                    "@{cf}Cached CMAKE_PREFIX_PATH:@|\\n\\t@{yf}%s@|\\n"
                    "@{cf}Other workspace to extend:@|\\n\\t@{yf}{_Context__extend_path}@|\\n"
                    "@{cf}Other workspace's CMAKE_PREFIX_PATH:@|\\n\\t@{yf}%s@|"
                    % (self.cached_cmake_prefix_path, self.env_cmake_prefix_path))]

        elif self.env_cmake_prefix_path and\
                self.cached_cmake_prefix_path and\
                self.env_cmake_prefix_path != self.cached_cmake_prefix_path:
            self.warnings += [clr(
                "Your current environment's CMAKE_PREFIX_PATH is different "
                "from the cached CMAKE_PREFIX_PATH used the last time this "
                "workspace was built.\\n\\n"
                "If you want to use a different CMAKE_PREFIX_PATH you should "
                "call @{yf}`catkin clean --all`@| to remove all references to "
                "the previous CMAKE_PREFIX_PATH.\\n\\n"
                "@{cf}Cached CMAKE_PREFIX_PATH:@|\\n\\t@{yf}%s@|\\n"
                "@{cf}Current CMAKE_PREFIX_PATH:@|\\n\\t@{yf}%s@|" %
                (self.cached_cmake_prefix_path, self.env_cmake_prefix_path))]

        # Check if prefix path is different from the environment prefix path
        if self.manual_cmake_prefix_path:
            self.cmake_prefix_path = self.manual_cmake_prefix_path
        elif self.cached_cmake_prefix_path:
            self.cmake_prefix_path = self.cached_cmake_prefix_path
        else:
            self.cmake_prefix_path = self.env_cmake_prefix_path

    def summary(self, notes=[]):
        # Add warnings (missing dirs in CMAKE_PREFIX_PATH, etc)
        summary_warnings = self.warnings
        if not self.initialized():
            summary_warnings += [clr(
                "Workspace `@{yf}{_Context__workspace}@|` is not yet "
                "initialized. Use the `catkin init` or run `catkin config "
                "--init`.")]
        if not self.source_space_exists():
            summary_warnings += [clr(
                "Source space `@{yf}{_Context__source_space_abs}@|` does not yet exist.")]
        if self.corrupted_by_catkin_make():
            summary_warnings += [clr(
                "Build space `@{yf}{_Context__build_space_abs}@|` exists but "
                "appears to have previously been created by the `catkin_make` "
                "or `catkin_make_isolated` tool. Please choose a different "
                "directory to use with `catkin_tools` or clean the build "
                "space.")]

        summary = [
            [
                clr("@{cf}Profile:@|                     @{yf}{profile}@|"),
                clr("@{cf}Extending:@|        {extend_mode} @{yf}{extend}@|"),
                clr("@{cf}Workspace:@|                   @{yf}{_Context__workspace}@|"),
                clr("@{cf}Source Space:@|      {source_missing} @{yf}{_Context__source_space_abs}@|"),
                clr("@{cf}Build Space:@|       {build_missing} @{yf}{_Context__build_space_abs}@|"),
                clr("@{cf}Devel Space:@|       {devel_missing} @{yf}{_Context__devel_space_abs}@|"),
                clr("@{cf}Install Space:@|     {install_missing} @{yf}{_Context__install_space_abs}@|"),
                clr("@{cf}DESTDIR:@|                     @{yf}{_Context__destdir}@|"),
            ],
            [
                clr("@{cf}Isolate Develspaces:@|         @{yf}{_Context__isolate_devel}@|"),
                clr("@{cf}Install Packages:@|            @{yf}{_Context__install}@|"),
                clr("@{cf}Isolate Installs:@|            @{yf}{_Context__isolate_install}@|"),
            ],
            [
                clr("@{cf}Additional CMake Args:@|       @{yf}{cmake_args}@|"),
                clr("@{cf}Additional Make Args:@|        @{yf}{make_args}@|"),
                clr("@{cf}Additional catkin Make Args:@| @{yf}{catkin_make_args}@|"),
                clr("@{cf}Internal Make Job Server:@|    @{yf}{_Context__use_internal_make_jobserver}@|"),
            ],
            [
                clr("@{cf}Whitelisted Packages:@|        @{yf}{whitelisted_packages}@|"),
                clr("@{cf}Blacklisted Packages:@|        @{yf}{blacklisted_packages}@|"),
            ]
        ]

        # Construct string for extend value
        if self.extend_path:
            extend_value = self.extend_path
            extend_mode = clr('@{gf}[explicit]@|')
        elif self.cached_cmake_prefix_path:
            extend_value = self.cmake_prefix_path
            extend_mode = clr('  @{gf}[cached]@|')
        elif (self.env_cmake_prefix_path and
                self.env_cmake_prefix_path != self.devel_space_abs and
                self.env_cmake_prefix_path != self.install_space_abs):
            extend_value = self.cmake_prefix_path
            extend_mode = clr('     @{gf}[env]@|')
        else:
            extend_value = 'None'
            extend_mode = clr('          ')

        def existence_str(path):
            return clr(' @{gf}[exists]@|' if os.path.exists(path) else '@{rf}[missing]@|')

        subs = {
            'profile': self.profile,
            'extend_mode': extend_mode,
            'extend': extend_value,
            'cmake_prefix_path': (self.cmake_prefix_path or ['Empty']),
            'cmake_args': ' '.join(self.__cmake_args or ['None']),
            'make_args': ' '.join(self.__make_args or ['None']),
            'catkin_make_args': ', '.join(self.__catkin_make_args or ['None']),
            'source_missing': existence_str(self.source_space_abs),
            'build_missing': existence_str(self.build_space_abs),
            'devel_missing': existence_str(self.devel_space_abs),
            'install_missing': existence_str(self.install_space_abs),
            'whitelisted_packages': ' '.join(self.__whitelist or ['None']),
            'blacklisted_packages': ' '.join(self.__blacklist or ['None']),
        }
        subs.update(**self.__dict__)
        # Get the width of the shell
        width = terminal_width()
        max_length = 0
        groups = []
        for group in summary:
            for index, line in enumerate(group):
                group[index] = line.format(**subs)
                max_length = min(width, max(max_length, len(remove_ansi_escape(group[index]))))
            groups.append("\n".join(group))
        divider = clr('@{pf}' + ('-' * max_length) + '@|')
        warning_divider = clr('@{rf}' + ('-' * max_length) + '@|')

        # Format warnings
        if len(summary_warnings) == 0:
            notes = [clr("@!@{cf}Workspace configuration appears valid.@|")] + notes
            warnings_joined = ''
        else:
            warnings_formatted = [
                printed_fill(clr('@!@{rf}WARNING:@| ') + sw.format(**subs), max_length)
                for sw in summary_warnings]
            warnings_joined = (
                "\n\n" + warning_divider + "\n" +
                ("\n" + warning_divider + "\n").join(warnings_formatted) +
                "\n" + warning_divider + "\n")

        return (divider + "\n" +
                ("\n" + divider + "\n").join(groups) + "\n" + divider + "\n" +
                ((("\n\n").join(notes) + "\n" + divider) if notes else '') +
                warnings_joined)

    @property
    def workspace(self):
        return self.__workspace

    @workspace.setter
    def workspace(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # Validate Workspace
        if not os.path.exists(value):
            raise ValueError("Workspace path '{0}' does not exist.".format(value))
        self.__workspace = os.path.abspath(value)

    @property
    def extend_path(self):
        return self.__extend_path

    @extend_path.setter
    def extend_path(self, value):
        if value is not None:
            if not os.path.isabs(value):
                value = os.path.join(self.workspace, value)
            if not os.path.exists(value):
                raise ValueError("Resultspace path '{0}' does not exist.".format(value))
        self.__extend_path = value

    @property
    def source_space_abs(self):
        return self.__source_space_abs

    @property
    def source_space(self):
        return self.__source_space

    @source_space.setter
    def source_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__source_space = value
        self.__source_space_abs = os.path.join(self.__workspace, value)

    def source_space_exists(self):
        "Returns true if the source space exists"
        return os.path.exists(self.source_space_abs) and os.path.isdir(self.source_space_abs)

    def initialized(self):
        """Check if this context is initialized."""
        return self.workspace == find_enclosing_workspace(self.workspace)

    @property
    def build_space_abs(self):
        return self.__build_space_abs

    @property
    def build_space(self):
        return self.__build_space

    @build_space.setter
    def build_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # TODO: check that build space was not run with a different context before
        self.__build_space = value
        self.__build_space_abs = os.path.join(self.__workspace, value)

    @property
    def devel_space_abs(self):
        return self.__devel_space_abs

    @property
    def devel_space(self):
        return self.__devel_space

    @devel_space.setter
    def devel_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # TODO: check that devel space was not run with a different context before
        self.__devel_space = value
        self.__devel_space_abs = os.path.join(self.__workspace, value)

    @property
    def install_space_abs(self):
        return self.__install_space_abs

    @property
    def install_space(self):
        return self.__install_space

    @install_space.setter
    def install_space(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        # TODO: check that install space was not run with a different context before
        self.__install_space = value
        self.__install_space_abs = os.path.join(self.__workspace, value)

    @property
    def destdir(self):
        return self.__destdir

    @destdir.setter
    def destdir(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__destdir = value

    @property
    def isolate_devel(self):
        return self.__isolate_devel

    @isolate_devel.setter
    def isolate_devel(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__isolate_devel = value

    @property
    def install(self):
        return self.__install

    @install.setter
    def install(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__install = value

    @property
    def isolate_install(self):
        return self.__isolate_install

    @isolate_install.setter
    def isolate_install(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__isolate_install = value

    @property
    def cmake_args(self):
        return self.__cmake_args

    @cmake_args.setter
    def cmake_args(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__cmake_args = value

    @property
    def make_args(self):
        return self.__make_args

    @make_args.setter
    def make_args(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__make_args = value

    @property
    def use_internal_make_jobserver(self):
        return self.__use_internal_make_jobserver

    @use_internal_make_jobserver.setter
    def use_internal_make_jobserver(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__use_internal_make_jobserver = value

    @property
    def catkin_make_args(self):
        return self.__catkin_make_args

    @catkin_make_args.setter
    def catkin_make_args(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__catkin_make_args = value

    @property
    def packages(self):
        return self.__packages

    @packages.setter
    def packages(self, value):
        if self.__locked:
            raise RuntimeError("Setting of context members is not allowed while locked.")
        self.__packages = value

    @property
    def whitelist(self):
        return self.__whitelist

    @whitelist.setter
    def whitelist(self, value):
        self.__whitelist = value

    @property
    def blacklist(self):
        return self.__blacklist

    @blacklist.setter
    def blacklist(self, value):
        self.__blacklist = value

    def corrupted_by_catkin_make(self):
        for other_tool_name in ['catkin_make_isolated', 'catkin_make']:
            if os.path.exists(os.path.join(self.build_space_abs, '%s.cache' % other_tool_name)):
                return True
        return False
