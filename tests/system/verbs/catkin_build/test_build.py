import os
import re
import shutil

from ...workspace_factory import workspace_factory

from ....utils import in_temporary_directory, temporary_directory
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
            build_type=build_type,
            depends=(['pkg_{}'.format(int((i - 1) / n_children))] if i > 0 else []))
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


def test_build_all_isolate_install():
    """Test building dependent catkin packages with isolated installspace."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            n_pkgs = create_tree_workspace(wf, 'catkin', 2)
            wf.create_package('pkg_dep', build_type='catkin',
                              build_depends=['pkg_{}'.format(n) for n in range(n_pkgs)])
            wf.build()

            assert catkin_success(['config', '--isolate-install', '--install'])
            assert catkin_success(BUILD)
            assert os.path.exists('install/pkg_dep')
    assert_no_warnings(out)


def test_build_all_isolate_devel():
    """Test building dependent catkin packages with isolated develspace."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            n_pkgs = create_tree_workspace(wf, 'catkin', 2)
            wf.create_package('pkg_dep', build_type='catkin',
                              build_depends=['pkg_{}'.format(n) for n in range(n_pkgs)])
            wf.build()

            assert catkin_success(['config', '--isolate-devel'])
            assert catkin_success(BUILD)
            assert os.path.exists('devel/pkg_dep')
            assert not os.path.exists('install')
    assert_no_warnings(out)


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
            print(os.getcwd())
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
            print(os.getcwd())
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
            print(os.getcwd())
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
            print(os.getcwd())
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'products_unicode'),
                os.path.join('src/products_unicode'))

            assert catkin_success(['config', '--link-devel'])
            assert catkin_success(BUILD)


def test_glob_pattern_build():
    """Test building multiple packages given as glob pattern"""
    with redirected_stdio() as (out, err):
        for build_type in BUILD_TYPES:
            with workspace_factory() as wf:
                create_flat_workspace(wf, build_type, 11)
                wf.build()
                assert catkin_success(BUILD + ['pkg_1*'])
                assert not os.path.exists(os.path.join('build', 'pkg_0'))
                assert os.path.exists(os.path.join('build', 'pkg_1'))
                assert os.path.exists(os.path.join('build', 'pkg_10'))
                assert not os.path.exists(os.path.join('build', 'pkg_2'))
                assert not os.path.exists(os.path.join('build', 'pkg_3'))
                assert not os.path.exists(os.path.join('build', 'pkg_4'))
                assert not os.path.exists(os.path.join('build', 'pkg_5'))
                assert not os.path.exists(os.path.join('build', 'pkg_6'))
                assert not os.path.exists(os.path.join('build', 'pkg_7'))
                assert not os.path.exists(os.path.join('build', 'pkg_8'))
                assert not os.path.exists(os.path.join('build', 'pkg_9'))


def test_pkg_with_conditional_build_type():
    """Test building a dual catkin/ament package."""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            print(os.getcwd())
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'build_type_condition'),
                os.path.join('src/build_type_condition'))

            assert catkin_success(['config', '--merge-devel'])
            assert catkin_success(BUILD)

            # Currently the build verb skips over packages it doesn't know how to build.
            # So we have to infer this skipping by checking the build directory.
            msg = "Package with ROS 2 conditional build_type was skipped."
            assert os.path.exists(os.path.join('build', 'build_type_condition')), msg


def test_pkg_with_conditional_depend():
    """Test building a package with a condition attribute in the depend tag"""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            wf.create_package('ros1_pkg')
            wf.create_package('ros2_pkg')
            wf.build()
            shutil.copytree(
                os.path.join(RESOURCES_DIR, 'catkin_pkgs', 'depend_condition'),
                os.path.join('src/depend_condition'))
            assert catkin_success(BUILD + ['depend_condition'], env={'ROS_VERSION': '1'})
            assert os.path.exists(os.path.join('build', 'depend_condition'))
            assert os.path.exists(os.path.join('build', 'ros1_pkg'))
            assert not os.path.exists(os.path.join('build', 'ros2_pkg'))


def test_symlinked_workspace():
    """Test building from a symlinked workspace"""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            wf.create_package('pkg')
            wf.build()
            assert catkin_success(BUILD)
            with temporary_directory() as t:
                os.symlink(wf.workspace, os.path.join(t, 'ws'))
                assert catkin_success(BUILD + ['-w', os.path.join(t, 'ws')])


def test_generate_setup_util():
    """Test generation of setup utilities in a linked devel space"""
    with redirected_stdio() as (out, err):
        with workspace_factory() as wf:
            wf.create_package('pkg')
            wf.build()
            # Test that the files are generated in a clean workspace
            assert catkin_success(['config', '--install'])
            assert catkin_success(BUILD)
            assert os.path.exists(os.path.join(wf.workspace, 'devel', '_setup_util.py'))
            assert os.path.exists(os.path.join(wf.workspace, 'install', '_setup_util.py'))

            # Test that the files are regenerated after clean
            assert catkin_success(['clean', '--yes'])
            assert catkin_success(BUILD)
            assert os.path.exists(os.path.join(wf.workspace, 'devel', '_setup_util.py'))
            assert os.path.exists(os.path.join(wf.workspace, 'install', '_setup_util.py'))

            # Test that the files are regenerated after cleaning the install space
            assert catkin_success(['clean', '--yes', '--install'])
            assert catkin_success(BUILD)
            assert os.path.exists(os.path.join(wf.workspace, 'devel', '_setup_util.py'))
            assert os.path.exists(os.path.join(wf.workspace, 'install', '_setup_util.py'))
