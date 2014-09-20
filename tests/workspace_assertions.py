
from __future__ import print_function

import functools
import os
import re
import shutil
import sys
import tempfile

import subprocess
import unittest

from .utils import assert_raises_regex
from .utils import in_temporary_directory
from .utils import redirected_stdio
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
