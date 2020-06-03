from catkin_tools.argument_parsing import extract_jobs_flags
from catkin_tools.argument_parsing import extract_jobs_flags_values


JOB_FLAG_ERR = "`extract_job_flags` failed to correctly extract flags from %s"


def check_only_strings_in_list_util(mflags, args_list):
    """Utility function for testing regular expressions used in argument parsing

    Tests if all space-separated values in mflags are in args_list and
    that there are no extra entries in args_list. If either of these
    tests fail, we return false.

    :param mflags: space separated string of arguments
    :type mflags: str
    :param args_list: list of strings to test against
    :type args_list: list
    :returns: if tests pass
    :rtype: bool
    """
    split_flags = mflags.split()
    if len(args_list) != len(split_flags):
        return False
    for arg in args_list:
        if arg not in split_flags:
            return False
        else:
            first_index = next(
                i for i, val in enumerate(split_flags) if val == arg)
            split_flags.pop(first_index)
    if not split_flags:
        return True
    else:
        return False


def test_job_flag_filtering_jl_packages():
    """Test ensures packages that start with `j` or `l` are not converted to make job args:"""
    args = "jpkg lpkg -j1 -l1"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "-j1 -l1", filtered), JOB_FLAG_ERR % args
    args = "--jobs=1 j2pkg l2pkg --load-average=1"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "--jobs=1 --load-average=1", filtered), JOB_FLAG_ERR % args
    args = "--jobs=1 j2pkg --verbose --no-deps l2pkg --load-average=1 --dry-run --start-with j2pkg"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "--jobs=1 --load-average=1", filtered), JOB_FLAG_ERR % args


def test_job_flag_filtering_empty_jl():
    """Test ensures proper capture of -j/-l args without a value (accepted by GNU Make)"""
    args = "pkg1 pkg2 -j"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "-j", filtered), JOB_FLAG_ERR % args
    args = "pkg1 pkg2 -l"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "-l", filtered), JOB_FLAG_ERR % args
    args = "--jobs pkg1 --verbose --no-deps pkg2 --load-average --dry-run --start-with pkg1"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "--jobs --load-average", filtered), JOB_FLAG_ERR % args


def test_no_job_flag():
    args = "jpkg1 lpkg2"
    filtered = extract_jobs_flags(args)
    assert filtered is None, JOB_FLAG_ERR % args


def test_job_flag_load_float():
    """Test ensures floating point values are handled correctly for load average"""
    args = "pkg1 pkg2 --load-average=1.23"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "--load-average=1.23", filtered), JOB_FLAG_ERR % args
    args = "-l2 pkg1 pkg2 --load-average=1.23 --no-deps --load-average=1 --load-average -l"
    filtered = extract_jobs_flags(args)
    assert check_only_strings_in_list_util(
        "-l2 --load-average=1.23 --load-average=1 --load-average -l", filtered), JOB_FLAG_ERR % args


def test_job_flag_values():
    """Test to ensure values are properly extracted from list of job flags"""
    flags = "-j1 -j2 --jobs=3 --jobs 4"
    valdict = extract_jobs_flags_values(flags)
    assert valdict['jobs'] == 4
    flags = "-l1 -l2 --load-average=3 --load-average 4"
    valdict = extract_jobs_flags_values(flags)
    assert abs(valdict['load-average'] - 4.0) < 1e-12
    flags = "-l -j"
    valdict = extract_jobs_flags_values(flags)
    assert 'load-average' in valdict and 'jobs' in valdict
    assert valdict['load-average'] is None and valdict['jobs'] is None
    flags = "--load-average --jobs"
    valdict = extract_jobs_flags_values(flags)
    assert 'load-average' in valdict and 'jobs' in valdict
    assert valdict['load-average'] is None and valdict['jobs'] is None
    flags = '-j -l2 -j -l --load-average=4'
    valdict = extract_jobs_flags_values(flags)
    assert 'load-average' in valdict and 'jobs' in valdict
    assert valdict['jobs'] is None and abs(
        valdict['load-average'] - 4.0) < 1e-12
