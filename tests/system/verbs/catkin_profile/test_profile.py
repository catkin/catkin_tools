import os

import yaml

from ....utils import assert_cmd_success
from ....utils import catkin_success
from ....utils import in_temporary_directory
from ....utils import redirected_stdio
from ....workspace_assertions import assert_in_config
from ...workspace_factory import workspace_factory

BUILD = ['build', '--no-notify', '--no-status']

BUILD = ['build', '--no-notify', '--no-status']


def assert_active_profile(workspace, profile):
    profile_file = os.path.join(workspace, '.catkin_tools', 'profiles', 'profiles.yml')
    if not os.path.exists(profile_file):
        assert profile == 'default'
    else:
        with open(profile_file) as f:
            profiles = yaml.safe_load(f)
        assert profiles.get('active', 'default') == profile


@in_temporary_directory
def test_profile_list():
    assert_cmd_success(['mkdir', 'src'])
    with redirected_stdio():
        assert catkin_success(['init'])
        assert catkin_success(BUILD)
        assert catkin_success(['profile', 'list'])
    assert_active_profile('.', 'default')


@in_temporary_directory
def test_profile_set():
    assert_cmd_success(['mkdir', 'src'])
    with redirected_stdio():
        assert catkin_success(['init'])
        assert catkin_success(BUILD)
        assert catkin_success(['profile', 'set', 'default'])
        assert catkin_success(['profile', 'add', 'second'])
        assert_active_profile('.', 'default')
        assert catkin_success(['profile', 'set', 'second'])
        assert_active_profile('.', 'second')


def test_profile_copy():
    with workspace_factory() as wf:
        wf.build()
        with redirected_stdio():
            assert catkin_success(['config', '--make-args', 'test'])
            assert catkin_success(['profile', 'add', '--copy', 'default', 'mycopy'])
        assert_in_config('.', 'mycopy', 'make_args', ['test'])
        assert_active_profile('.', 'default')


def test_profile_extend():
    with workspace_factory() as wf:
        wf.build()
        with redirected_stdio():
            assert catkin_success(['config', '--make-args', 'test'])
            assert catkin_success(['profile', 'add', '--extend', 'default', 'myextend'])
            assert catkin_success(['config', '--profile', 'myextend', '--skiplist', 'mypackage'])
        assert_in_config('.', 'default', 'make_args', ['test'])
        assert_in_config('.', 'myextend', 'blacklist', ['mypackage'])


def test_different_first_profile():
    with workspace_factory() as wf:
        wf.build()
        assert catkin_success(BUILD + ['--profile', 'release'])
        assert_active_profile(wf.workspace, 'release')
