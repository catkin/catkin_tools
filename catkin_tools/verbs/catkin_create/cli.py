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

from catkin_tools.argument_parsing import add_context_args
from catkin_tools.context import Context
from catkin_pkg.package_templates import create_package_files, PackageTemplate

# Exempt build directories
# See https://github.com/catkin/catkin_tools/issues/82


def prepare_arguments(parser):
    # Workspace / profile args
    add_context_args(parser)

    subparsers = parser.add_subparsers(dest='subcommand', help='sub-command help')

    parser_pkg = subparsers.add_parser('pkg', help='Create a new catkin package.')

    parser_pkg.description = (
        "Create a new Catkin package. Note that while the "
        "default options used by this command are sufficient for prototyping and "
        "local usage, it is important that any publically-available packages have "
        "a valid license and a valid maintainer e-mail address.")

    add = parser_pkg.add_argument

    add('name', metavar='PKG_NAME', nargs='+',
        help='The name of one or more packages to create. This name should be '
        'completely lower-case with individual words separated by undercores.')

    add('-p', '--path', action='store', default=os.getcwd(),
        help='The path into which the package should be generated.')

    # TODO: Make this possible
    # add('--manifest-only', action='store_true', default=False,
    #     help='Only create a package.xml manifest file and do not generate a CMakeLists.txt')

    # TODO: Make this possible
    # add('--build-type', type=str, choices=['catkin', 'cmake'],
    #     nargs=1,
    #     default='catkin',
    #     help='The buildtool to use to build the package. (default: catkin)')

    rosdistro_name = os.environ['ROS_DISTRO'] if 'ROS_DISTRO' in os.environ else None
    add('--rosdistro', required=rosdistro_name is None, default=rosdistro_name,
        help='The ROS distro (default: environment variable ROS_DISTRO if defined)')

    basic_group = parser_pkg.add_argument_group('Package Metadata')
    add = basic_group.add_argument

    add('-v', '--version',
        metavar='MAJOR.MINOR.PATCH',
        action='store',
        help='Initial package version. (default 0.0.0)')
    add('-l', '--license',
        action='append',
        help='The software license under which the code is distributed, such as '
        'BSD, MIT, GPLv3, or others. (default: "TODO")')
    add('-m', '--maintainer',
        metavar=('NAME', 'EMAIL'),
        dest='maintainers',
        action='append',
        nargs=2,
        help='A maintainer who is responsible for the package. (default: '
        '[username, username@todo.todo]) (multiple allowed)')
    add('-a', '--author',
        metavar=('NAME', 'EMAIL'),
        dest='authors',
        action='append',
        nargs=2,
        help='An author who contributed to the package. (default: no additional '
        'authors) (multiple allowed)')
    add('-d', '--description',
        action='store',
        help='Description of the package. (default: empty)')

    deps_group = parser_pkg.add_argument_group('Package Dependencies')
    add = deps_group.add_argument
    add('--catkin-deps', '-c', metavar='DEP', nargs="*",
        help='The names of one or more Catkin dependencies. These are '
        'Catkin-based packages which are either built as source or installed '
        'by your system\'s package manager.')
    add('--system-deps', '-s', metavar='DEP', nargs="*",
        help='The names of one or more system dependencies. These are other '
        'packages installed by your operating system\'s package manager.')

    cpp_group = parser_pkg.add_argument_group('C++ Options')
    add = cpp_group.add_argument
    add('--boost-components',
        metavar='COMP',
        nargs='*',
        help='One or more boost components used by the package.')

    # py_group = parser_pkg.add_argument_group('Python Options')
    # add('--python-setup', action='store_true', default=False,
    #     help='Add a default python setup file.')

    return parser


def main(opts):

    # Load the context
    ctx = Context.load(opts.workspace, opts.profile, opts, append=True)
    try:
        # Get absolute path to directory containing package
        package_dest_path = os.path.abspath(opts.path)

        # Sort list of maintainers and authors (it will also be sorted inside
        # PackageTemplate so by sorting it here, we ensure that the same order
        # is used.  This is important later when email addresses are assigned.
        if not opts.maintainers:
            maintainers = []
            for x in ctx.maintainers:
                email = x.split()[-1]
                name = ' '.join(x.split()[:-1])
                maintainers += [[name, email]]
            opts.maintainers = maintainers
        if opts.maintainers:
            opts.maintainers.sort(key=lambda x: x[0])
        if not opts.authors:
            authors = []
            for x in ctx.authors:
                email = x.split()[-1]
                name = ' '.join(x.split()[:-1])
                authors += [[name, email]]
            opts.authors = authors
        if opts.authors:
            opts.authors.sort(key=lambda x: x[0])
        if not opts.license:
            opts.license = ctx.licenses or []

        for package_name in opts.name:
            print('Creating package "%s" in "%s"...' % (package_name, package_dest_path))
            target_path = os.path.join(package_dest_path, package_name)
            package_template = PackageTemplate._create_package_template(
                package_name=package_name,
                description=opts.description,
                licenses=opts.license,
                maintainer_names=[m[0] for m in opts.maintainers] if opts.maintainers else [],
                author_names=[a[0] for a in opts.authors] if opts.authors else [],
                version=opts.version,
                catkin_deps=opts.catkin_deps,
                system_deps=opts.system_deps,
                boost_comps=opts.boost_components)

            # Add maintainer and author e-mails
            if opts.maintainers:
                for (pm, om) in zip(package_template.maintainers, opts.maintainers):
                    pm.email = om[1]
            if opts.authors:
                for (pa, oa) in zip(package_template.authors, opts.authors):
                    pa.email = oa[1]

            # Add build type export
            # if opts.build_type and opts.build_type != 'catkin':
            #     build_type = Export('build_type', content=opts.build_type)
            #     package_template.exports.append(build_type)

            create_package_files(target_path=target_path,
                                 package_template=package_template,
                                 rosdistro=opts.rosdistro,
                                 newfiles={})
            print('Successfully created package files in %s.' % target_path)
    except ValueError as vae:
        print(str(vae))
        return 1

    return 0
