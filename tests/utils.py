from __future__ import print_function

import functools
import os
import re
import shutil
import sys
import tempfile

import subprocess

from catkin_tools.commands.catkin import main as catkin_main

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


def catkin_success(args, env={}):
    orig_environ = dict(os.environ)
    try:
        os.environ.update(env)
        catkin_main(args)
    except SystemExit as exc:
        ret = exc.code
        if ret != 0:
            import traceback
            traceback.print_exc()
    finally:
        os.environ = orig_environ
    return ret == 0


def catkin_failure(args, env={}):
    orig_environ = dict(os.environ)
    try:
        os.environ.update(env)
        catkin_main(args)
    except SystemExit as exc:
        ret = exc.code
    finally:
        os.environ = orig_environ
    return ret != 0


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
        self.out = StringIO()
        self.err = StringIO()
        sys.stdout = self.out
        sys.stderr = self.err
        return self.out, self.err

    def __exit__(self, exc_type, exc_value, traceback):
        sys.stdout = self.original_stdout
        sys.stderr = self.original_stderr

        print(self.out.getvalue())


class temporary_directory(object):

    def __init__(self, prefix=''):
        self.prefix = prefix
        self.delete = False

    def __enter__(self):
        self.original_cwd = os.getcwd()
        self.temp_path = tempfile.mkdtemp(prefix=self.prefix)
        os.chdir(self.temp_path)
        return self.temp_path

    def __exit__(self, exc_type, exc_value, traceback):
        if self.delete and self.temp_path and os.path.exists(self.temp_path):
            print('Deleting temporary testind directory: %s' % self.temp_path)
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


def run(args, **kwargs):
    """
    Call to Popen, returns (errcode, stdout, stderr)
    """
    print("run:", args)
    p = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        cwd=kwargs.get('cwd', os.getcwd()))
    print("P==", p.__dict__)
    (stdout, stderr) = p.communicate()

    return (p.returncode, stdout, stderr)


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
    assert 0 != r, "cmd succeeded, but it should fail: %s result=%u\noutput=\n%s" % (cmd, r, out)
    return out


def assert_files_exist(prefix, files):
    """
    Assert that all files exist in the prefix.
    """
    for f in files:
        p = os.path.join(prefix, f)
        print("Checking for", p)
        assert os.path.exists(p), "%s doesn't exist" % p
