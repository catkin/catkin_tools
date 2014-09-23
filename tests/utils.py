
from __future__ import print_function

import functools
import os
import re
import shutil
import sys
import tempfile

import subprocess
import unittest

try:
    # Python2
    from StringIO import StringIO
except ImportError:
    # Python3
    from io import StringIO

try:
    from subprocess import TimeoutExpired
except ImportError:
    class TimeoutExpired(object):
        pass

TESTS_DIR = os.path.dirname(__file__)
MOCK_DIR = os.path.join(TESTS_DIR, 'mock_resources')

def assert_raises(exception_classes, callable_obj=None, *args, **kwargs):
    context = AssertRaisesContext(exception_classes)
    if callable_obj is None:
        return context
    with context:
        callable_obj(*args, **kwargs)


def assert_raises_regex(exception_classes, expected_regex, callable_obj=None, *args, **kwargs):
    context = AssertRaisesContext(exception_classes, expected_regex)
    if callable_obj is None:
        return context
    with context:
        callable_obj(*args, **kwargs)


class AssertRaisesContext(object):

    def __init__(self, expected, expected_regex=None):
        self.expected = expected
        self.expected_regex = expected_regex

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if self.expected is None:
            if exc_type is None:
                return True
            else:
                raise
        if exc_type is None:
            try:
                exc_name = self.expected.__name__
            except AttributeError:
                exc_name = str(self.expected)
            raise AssertionError("{0} not raised".format(exc_name))
        if not issubclass(exc_type, self.expected):
            raise
        if self.expected_regex is None:
            return True
        expected_regex = self.expected_regex
        expected_regex = re.compile(expected_regex)
        if not expected_regex.search(str(exc_value)):
            raise AssertionError("'{0}' does not match '{1}'".format(expected_regex.pattern, str(exc_value)))
        return True


class redirected_stdio(object):

    def __enter__(self):
        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr
        sys.stdout = out = StringIO()
        sys.stderr = err = StringIO()
        return out, err

    def __exit__(self, exc_type, exc_value, traceback):
        sys.stdout = self.original_stdout
        sys.stderr = self.original_stderr


class temporary_directory(object):

    def __init__(self, prefix=''):
        self.prefix = prefix

    def __enter__(self):
        self.original_cwd = os.getcwd()
        self.temp_path = tempfile.mkdtemp(prefix=self.prefix)
        os.chdir(self.temp_path)
        return self.temp_path

    def __exit__(self, exc_type, exc_value, traceback):
        if self.temp_path and os.path.exists(self.temp_path):
            shutil.rmtree(self.temp_path)
        if self.original_cwd and os.path.exists(self.original_cwd):
            os.chdir(self.original_cwd)


def in_temporary_directory(f):
    @functools.wraps(f)
    def decorated(*args, **kwds):
        with temporary_directory() as directory:
            from inspect import getargspec
            # If it takes directory of kwargs and kwds does already have
            # directory, inject it
            if 'directory' not in kwds and 'directory' in getargspec(f)[0]:
                kwds['directory'] = directory
            return f(*args, **kwds)
    decorated.__name__ = f.__name__
    return decorated


def rosinstall(pth, specfile):
    '''
    calls rosinstall in pth with given specfile,
    then replaces CMakelists with catkin's toplevel.cmake'
    '''
    assert os.path.exists(specfile), specfile
    # to save testing time, we do not invoke rosinstall when we
    # already have a .rosinstall file
    if not os.path.exists(os.path.join(pth, '.rosinstall')):
        succeed(["rosinstall", "-j8", "--catkin", "-n",
                 pth, specfile, '--continue-on-error'], cwd=TESTS_DIR)


def run(args, **kwargs):
    """
    Call to Popen, returns (errcode, stdout, stderr)
    """
    print("run:", args)
    with tempfile.NamedTemporaryFile(mode='w+') as temp_buffer:
        p = subprocess.Popen(
            args,
            stdout=temp_buffer,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            cwd=kwargs.get('cwd', os.getcwd()))
        print("P==", p.__dict__)
        print("Dumping stdout to: "+ temp_buffer.name)
        p.wait()
        temp_buffer.seek(0)
        stdout = temp_buffer.read()

        return (p.returncode, stdout, '')

    return (None, None, None)


def assert_cmd_success(cmd, **kwargs):
    """
    Asserts that running a command returns zero.

    returns: stdout
    """
    print(">>>", cmd, kwargs)
    (r, out, err) = run(cmd, **kwargs)
    print("<<<", str(out))
    assert r == 0, "cmd failed with result %s:\n %s " % (r, str(cmd))
    return out


def assert_cmd_failure(cmd, **kwargs):
    """
    Asserts that running a command returns non-zero.

    returns: stdout
    """
    print(">>>", cmd, kwargs)
    (r, out, err) = run(cmd, withexitstatus=True, **kwargs)
    print("<<<", str(out))
    assert 0 != r, """cmd succeeded, though should fail: %s result=%u\noutput=\n%s""" % (cmd, r, out)
    return out

def assert_files_exist(prefix, files):
    """
    Assert that all files exist in the prefix.
    """
    for f in files:
        p = os.path.join(prefix, f)
        print("Checking for", p)
        assert os.path.exists(p), "%s doesn't exist" % p
