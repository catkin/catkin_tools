import os
import re
import yaml

from .utils import assert_files_exist


def assert_workspace_initialized(path):
    assert_files_exist(path, ['.catkin_tools'])


def assert_warning_message(out_str, pattern=''):
    """
    Assert that the stdout returned from a call contains a catkin_tools
    warning.
    """
    out_str_stripped = ' '.join(str(out_str).splitlines())
    found = re.findall('WARNING:', out_str_stripped)
    assert len(found) > 0

    if pattern:
        found = re.findall(pattern, out_str_stripped)
        assert len(found) > 0


def assert_no_warnings(out_str):
    """
    Assert that the stdout returned from a call contains a catkin_tools
    warning.
    """
    out_str_stripped = ' '.join(str(out_str).splitlines())
    found = re.findall('WARNING:', out_str_stripped)
    assert len(found) == 0


def assert_in_config(workspace, profile, key, value):
    with open(os.path.join(workspace, '.catkin_tools', 'profiles', profile, 'config.yaml')) as f:
        config = yaml.safe_load(f)

    assert config.get(key, None) == value
