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


from catkin_tools.argument_parsing import add_context_args
from catkin_tools.context import Context
from catkin_tools.verbs.catkin_test.test import test_workspace


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

    return parser


def main(opts):
    ctx = Context.load(opts.workspace, opts.profile, opts, append=True)

    return test_workspace(
        ctx,
        packages=opts.packages,
    )