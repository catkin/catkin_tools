import flake8.engine
import os


def test_flake8():
    """Test source code for pyFlakes and PEP8 conformance"""
    flake8style = flake8.engine.StyleGuide(max_line_length=120)
    report = flake8style.options.report
    report.start()
    this_dir = os.path.dirname(os.path.abspath(__file__))
    try:
        input_dir = flake8style.input_dir
    except AttributeError:
        input_dir = flake8style._styleguide.input_dir
    input_dir(os.path.join(this_dir, '..', 'catkin_tools'))
    report.stop()
    assert report.total_errors == 0, \
        ("Found '{0}' code style errors (and warnings)."
         .format(report.total_errors))
