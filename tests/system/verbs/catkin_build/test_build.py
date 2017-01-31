from __future__ import print_function

import os
import re
import shutil

from math import floor

from ...workspace_factory import workspace_factory

from ....utils import in_temporary_directory
from ....utils import assert_cmd_success
from ....utils import assert_cmd_failure
from ....utils import assert_files_exist
from ....utils import catkin_success
from ....utils import catkin_failure
from ....utils import redirected_stdio


from ....workspace_assertions import assert_workspace_initialized
from ....workspace_assertions import assert_no_warnings

TEST_DIR = os.path.dirname(__file__)
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'resources')

BUILD = ['build', '--no-notify', '--no-status']
CLEAN = ['clean', '--yes']

BUILD_TYPES = ['cmake', 'catkin']


def create_flat_workspace(wf, build_type, n_pkgs):
    """Create a bunch of packages with no interdependencies"""
    for i in range(n_pkgs):
        wf.create_package('pkg_{}'.format(i))


def create_chain_workspace(wf, build_type, n_pkgs):
    """Create a bunch of packages, each of which depends on one other in the
    workspace except for the root."""
    for i in range(n_pkgs):
        wf.create_package(
            'pkg_{}'.format(i),
            depends=(['pkg_{}'.format(i - 1)] if i > 0 else []))


def create_tree_workspace(wf, build_type, n_pkg_layers, n_children=2):
    """Create a bunch of packages which form a balanced dependency tree"""
    n_pkgs = pow(n_children, n_pkg_layers + 1) - 1
    for i in range(n_pkgs):
        wf.create_package(
            'pkg_{}'.format(i),
            depends=(['pkg_{}'.format(floor(i - 1) / n_children)] if i > 0 else []))
    return n_pkgs


@in_temporary_directory
def test_build_no_src():
    """Calling catkin build without a source space should fail."""
    assert catkin_failure(BUILD)


def test_build_auto_init_no_pkgs():
    """Test automatically initializing a workspace with no packages."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            wf.build()
            assert catkin_success(BUILD)
            assert_workspace_initialized('.')
    assert_no_warnings(out)


def test_build_auto_init_with_pkg():
    """Test automatically initializing a workspace."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            wf.create_package('pkg_a')
            wf.build()
            assert catkin_success(BUILD)
            assert_workspace_initialized('.')
    assert_no_warnings(out)


def test_build_dry_run():
    """Test showing the build jobs without doing anything."""
    with redirected_stdio() as (out, err):
        for build_type in BUILD_TYPES:
            with workspace_factory() as wf:
                create_tree_workspace(wf, build_type, 3)
                wf.build()
                assert catkin_success(BUILD + ['--dry-run'])
                assert not os.path.exists('build')
                assert not os.path.exists('devel')


def test_build_all_isolated():
    """Test building all packages in an isolated workspace"""
    pass  # TODO: Implement test


def test_build_all_merged():
    """Test building all packages in a merged workspace"""
    pass  # TODO: Implement test


def test_build_pkg():
    """Test building a package by name.
    """
    with redirected_stdio() as (out, err):
        for build_type in BUILD_TYPES:
            with workspace_factory() as wf:
                create_chain_workspace(wf, build_type, 4)
                wf.build()
                assert catkin_failure(BUILD + ['pkg_nil'])
                assert catkin_success(BUILD + ['pkg_2'])
                assert os.path.exists(os.path.join('build', 'pkg_0'))
                assert os.path.exists(os.path.join('build', 'pkg_1'))
                assert os.path.exists(os.path.join('build', 'pkg_2'))
                assert not os.path.exists(os.path.join('build', 'pkg_3'))


def test_build_no_deps():
    """Test building a package by name without deps."""
    with redirected_stdio() as (out, err):
        for build_type in BUILD_TYPES:
            with workspace_factory() as wf:
                create_chain_workspace(wf, build_type, 3)
                wf.build()

                # --no-deps needs an argument
                assert catkin_failure(BUILD + ['--no-deps'])
                # only pkg_2 shuold be built
                assert catkin_success(BUILD + ['pkg_2', '--no-deps'])
                assert os.path.exists(os.path.join('build', 'pkg_2'))
                assert not os.path.exists(os.path.join('build', 'pkg_1'))
                assert not os.path.exists(os.path.join('build', 'pkg_0'))


def test_build_start_with():
    """Test building all packages starting with a specific one."""
    with redirected_stdio() as (out, err):
        for build_type in BUILD_TYPES:
            with workspace_factory() as wf:
                create_chain_workspace(wf, build_type, 4)
                wf.build()

                # --start-with needs an argument
                assert catkin_failure(BUILD + ['--start-with'])

                # --start-with needs a valid package
                assert catkin_failure(BUILD + ['--start-with', 'pkg_nil'])

                # this should build all packages
                assert catkin_success(BUILD + ['--start-with', 'pkg_0'])
                for i in range(4):
                    assert os.path.exists(os.path.join('build', 'pkg_{}'.format(i)))
                assert catkin_success(CLEAN)

                # this should skip pkg_2's deps
                assert catkin_success(BUILD + ['--start-with', 'pkg_2'])
                assert not os.path.exists(os.path.join('build', 'pkg_0'))
                assert not os.path.exists(os.path.join('build', 'pkg_1'))
                assert os.path.exists(os.path.join('build', 'pkg_2'))
                assert os.path.exists(os.path.join('build', 'pkg_3'))
                assert catkin_success(CLEAN)


def test_unbuilt_linked():
    """Test building packages which have yet to be built"""
    with redirected_stdio() as (out, err):
        for build_type in BUILD_TYPES:
            with workspace_factory() as wf:
                create_chain_workspace(wf, build_type, 2)
                wf.build()

                # only pkg_0 shuold be built
                assert catkin_success(BUILD + ['pkg_0', '--no-deps'])
                # the rest should be built, but pkg_0 shouldn't be rebuilt
                assert os.path.exists(os.path.join('build', 'pkg_0'))
                assert not os.path.exists(os.path.join('build', 'pkg_1'))

                pkg_0_log_path = os.path.join('logs', 'pkg_0')

                # build the unbuilt packages (rebuild deps)
                pkg_0_log_files = os.listdir(pkg_0_log_path)
                assert catkin_success(BUILD + ['--unbuilt'])
                assert os.path.exists(os.path.join('build', 'pkg_0'))
                assert os.path.exists(os.path.join('build', 'pkg_1'))
                # make sure pkg_0 has been rebuilt
                assert pkg_0_log_files != os.listdir(pkg_0_log_path)

                # build the unbuilt packages (don't rebuild deps)
                pkg_0_log_files = os.listdir(pkg_0_log_path)
                assert catkin_success(['clean', 'pkg_1'])
                assert catkin_success(BUILD + ['--unbuilt', '--no-deps'])
                assert os.path.exists(os.path.join('build', 'pkg_0'))
                assert os.path.exists(os.path.join('build', 'pkg_1'))
                # make sure pkg_0 hasn't been rebuilt
                assert pkg_0_log_files == os.listdir(pkg_0_log_path)


def test_unbuilt_isolated():
    """Test building unbuilt packages with an isolated develspace."""
    pass  # TODO: This should succeed, but isn't implemented for isolated develspaces


def test_unbuilt_merged():
    """Test building unbuilt packages with a merged develspace."""
    pass  # TODO: This should fail, but the check hsan't been tested


def test_continue_on_failure():
    """Test behavior when some packages fail to build."""
    pass  # TODO: Write test


def test_preclean():
    """Test pre-cleaning packages in a workspace."""
    pass  # TODO: Write test


def test_force_cmake():
    """Test forcing cmake to run on packages in a workspace."""
    pass  # TODO: Write test


def test_install():
    """Test building and installing catkin packages without DESTDIR"""
    with redirected_stdio() as (out, err):
        for build_type in BUILD_TYPES:
            with workspace_factory() as wf:
                create_chain_workspace(wf, build_type, 2)
                wf.build()

                assert catkin_success(['config', '--install'])
                assert catkin_success(BUILD)
                assert os.path.exists(os.path.join('install'))


def test_install_cmake():
    """Test building and installing cmake packages without DESTDIR."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            print(os.getcwd)
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'cmake_pkgs'),
                os.path.join('src/cmake_pkgs'))

            assert catkin_success(['config', '--install'])
            assert catkin_success(BUILD)
            assert os.path.exists(os.path.join('install'))


def test_install_cmake_destdir():
    """Test building and installing cmake packages with DESTDIR."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            print(os.getcwd)
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'cmake_pkgs'),
                os.path.join('src/cmake_pkgs'))

            tmpinstall_path = os.path.join(os.getcwd(), 'tmpinstall')
            env = {'DESTDIR': tmpinstall_path}

            assert catkin_success(['config', '--install', '--install-space', '/opt/foo'], env)
            assert catkin_success(BUILD, env)
            assert os.path.exists(tmpinstall_path)
            assert not os.path.exists(os.path.join('install'))


def test_install_catkin_destdir():
    """Test building and installing catkin packages with DESTDIR."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            print(os.getcwd)
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'products_0'),
                os.path.join('src', 'products_0'))

            tmpinstall_path = os.path.join(os.getcwd(), 'tmpinstall')
            env = {'DESTDIR': tmpinstall_path}
            install_space = os.path.abspath(os.path.join('opt', 'foo'))

            assert catkin_success(['config', '--install', '--install-space', install_space], env)
            assert catkin_success(BUILD, env)
            assert os.path.exists(tmpinstall_path)
            assert not os.path.exists(os.path.join('install'))

            # check for _CATKIN_SETUP_DIR
            setup_sh_path = os.path.join(tmpinstall_path, install_space.lstrip(os.sep), 'setup.sh')
            print(setup_sh_path)
            assert os.path.exists(setup_sh_path)
            setup_dir_correct = False
            with open(setup_sh_path, "r") as setup_sh:
                for line in setup_sh:
                    if re.search('_CATKIN_SETUP_DIR:={}'.format(install_space), line):
                        setup_dir_correct = True
                        break
            assert setup_dir_correct is True


def test_pkg_with_unicode_names():
    """Test building a package with unicode file names."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            print(os.getcwd)
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'products_unicode'),
                os.path.join('src/cmake_pkgs'))

            assert catkin_success(['config', '--link-devel'])
            assert catkin_success(BUILD)
