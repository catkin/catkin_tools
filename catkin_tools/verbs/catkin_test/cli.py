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
import argparse
import os
import sys

from catkin_pkg.package import InvalidPackage

from catkin_tools.argument_parsing import add_context_args
from catkin_tools.argument_parsing import configure_make_args
from catkin_tools.common import find_enclosing_package
from catkin_tools.common import getcwd
from catkin_tools.common import is_tty
from catkin_tools.context import Context
from catkin_tools.context import clr
from catkin_tools.metadata import find_enclosing_workspace
from catkin_tools.resultspace import load_resultspace_environment
from catkin_tools.terminal_color import set_color

from .test import test_workspace


def prepare_arguments(parser):
    parser.description = """\
Test one or more packages in a catkin workspace.
This invokes `make run_tests` or `make test` for either all or the specified
packages in a catkin workspace.\
"""

    # Workspace / profile args
    add_context_args(parser)
    # Sub-commands
    # What packages to test
    pkg_group = parser.add_argument_group('Packages', 'Control which packages get tested.')
    add = pkg_group.add_argument
    add('packages', metavar='PKGNAME', nargs='*',
        help='Workspace packages to test. If no packages are given, then all the packages are tested.')
    add('--this', dest='build_this', action='store_true', default=False,
        help='Test the package containing the current working directory.')
    add('--continue-on-failure', '-c', action='store_true', default=False,
        help='Continue testing packages even if the tests for other requested packages fail.')

    config_group = parser.add_argument_group('Config', 'Parameters for the underlying build system.')
    add = config_group.add_argument
    add('-p', '--parallel-packages', metavar='PACKAGE_JOBS', dest='parallel_jobs', default=None, type=int,
        help='Maximum number of packages allowed to be built in parallel (default is cpu count)')
    add('-t', '--test-target', metavar='TARGET', default=None, type=str,
        help='Make target to run for tests (default is "run_tests" for catkin and "test" for cmake)')
    add('--catkin-test-target', metavar='TARGET', default=None, type=str,
        help='Make target to run for tests for catkin packages, overwrites --test-target (default is "run_tests")')
    add('--make-args', metavar='ARG', dest='make_args', nargs='+', required=False, type=str, default=None,
        help='Arbitrary arguments which are passed to make. '
             'It collects all of following arguments until a "--" is read.')

    interface_group = parser.add_argument_group('Interface', 'The behavior of the command-line interface.')
    add = interface_group.add_argument
    add('--verbose', '-v', action='store_true', default=False,
        help='Print output from commands in ordered blocks once the command finishes.')
    add('--interleave-output', '-i', action='store_true', default=False,
        help='Prevents ordering of command output when multiple commands are running at the same time.')
    add('--summarize', '--summary', '-s', action='store_true', default=None,
        help='Adds a summary to the end of the log')
    add('--no-status', action='store_true', default=False,
        help='Suppresses status line, useful in situations where carriage return is not properly supported.')

    def status_rate_type(rate):
        rate = float(rate)
        if rate < 0:
            raise argparse.ArgumentTypeError("must be greater than or equal to zero.")
        return rate

    add('--limit-status-rate', '--status-rate', type=status_rate_type, default=10.0,
        help='Limit the update rate of the status bar to this frequency. Zero means unlimited. '
             'Must be positive, default is 10 Hz.')
    add('--no-notify', action='store_true', default=False,
        help='Suppresses system pop-up notification.')

    # Deprecated arguments
    # colors are handled by the main catkin command
    add('--no-color', action='store_true', help=argparse.SUPPRESS)
    add('--force-color', action='store_true', help=argparse.SUPPRESS)
    # no-deps became the default
    add('--no-deps', action='store_true', help=argparse.SUPPRESS)

    return parser


def main(opts):
    # Set color options
    opts.force_color = os.environ.get('CATKIN_TOOLS_FORCE_COLOR', opts.force_color)
    if (opts.force_color or is_tty(sys.stdout)) and not opts.no_color:
        set_color(True)
    else:
        set_color(False)

    # Context-aware args
    if opts.build_this:
        # Determine the enclosing package
        try:
            ws_path = find_enclosing_workspace(getcwd())
            # Suppress warnings since this won't necessarily find all packages
            # in the workspace (it stops when it finds one package), and
            # relying on it for warnings could mislead people.
            this_package = find_enclosing_package(
                search_start_path=getcwd(),
                ws_path=ws_path,
                warnings=[])
        except InvalidPackage as ex:
            sys.exit(clr("[test] @!@{rf}Error:@| The file {} is an invalid package.xml file."
                         " See below for details:\n\n{}").format(ex.package_path, ex.msg))

        # Handle context-based package building
        if this_package:
            opts.packages += [this_package]
        else:
            sys.exit(clr("[test] @!@{rf}Error:@| In order to use --this, "
                         "the current directory must be part of a catkin package."))

    # Load the context
    ctx = Context.load(opts.workspace, opts.profile, opts, append=True)

    # Load the environment of the workspace to extend
    if ctx.extend_path is not None:
        try:
            load_resultspace_environment(ctx.extend_path)
        except IOError as exc:
            sys.exit(clr("[test] @!@{rf}Error:@| Unable to extend workspace from \"{}\": {}").format(
                         ctx.extend_path, str(exc)))

    # Check if the context is valid before writing any metadata
    if not ctx.source_space_exists():
        sys.exit(clr("[test] @!@{rf}Error:@| Unable to find source space `{}`").format(ctx.source_space_abs))

    # Extract make arguments
    make_args, _, _, _ = configure_make_args(ctx.make_args, ctx.jobs_args, ctx.use_internal_make_jobserver)
    ctx.make_args = make_args

    # Get parallel toplevel jobs
    try:
        parallel_jobs = int(opts.parallel_jobs)
    except TypeError:
        parallel_jobs = None

    # Set VERBOSE environment variable
    if opts.verbose and 'VERBOSE' not in os.environ:
        os.environ['VERBOSE'] = '1'

    # Get test targets
    catkin_test_target = 'run_tests'
    cmake_test_target = 'test'
    if opts.test_target:
        catkin_test_target = opts.test_target
        cmake_test_target = opts.test_target
    if opts.catkin_test_target:
        catkin_test_target = opts.catkin_test_target

    return test_workspace(
        ctx,
        packages=opts.packages,
        n_jobs=parallel_jobs,
        quiet=not opts.verbose,
        interleave_output=opts.interleave_output,
        no_status=opts.no_status,
        limit_status_rate=opts.limit_status_rate,
        no_notify=opts.no_notify,
        continue_on_failure=opts.continue_on_failure,
        summarize_build=opts.summarize,
        catkin_test_target=catkin_test_target,
        cmake_test_target=cmake_test_target,
    )
