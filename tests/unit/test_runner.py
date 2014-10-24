from __future__ import print_function
from __future__ import unicode_literals

import os
import mock

from catkin_tools.runner.run_unix import run_command

from nose.tools import eq_ as assert_eq

TEST_DIR = os.path.dirname(__file__)


@mock.patch('catkin_tools.runner.run_unix.run_command')
def test_runner_ascii(patched_func):
    cmd = ['cat', os.path.join(TEST_DIR, 'ascii_text.txt')]
    for line in run_command(cmd):
        if type(line) == int:
            assert_eq(line, 0)
        else:
            assert_eq(line.rstrip(), 'Hello ASCII!')


@mock.patch('catkin_tools.runner.run_unix.run_command')
def test_runner_unicode(patched_func):
    cmd = ['cat', os.path.join(TEST_DIR, 'unicode_text.txt')]
    for line in run_command(cmd):
        if type(line) == int:
            assert_eq(line, 0)
        else:
            if line.rstrip() != 'Hello Unicode\u203d':
                print('WARNING: Unicode reading not supported!')
