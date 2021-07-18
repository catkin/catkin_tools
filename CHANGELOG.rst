^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package catkin_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Upcoming
--------

* Add possibility to clean individual package with isolated devel space (#683)
* Fix regeneration of setup file when the install space was cleaned (#682)
* Fix workspace generation with catkin build --this and --start-with-this (#685)

0.7.1 (2021-07-17)
------------------
* Fixes in the build system requiring a version increase

0.7.0 (2021-03-01)
------------------
* Fix placeholders for cmake warning and error coloring (#678)
* Fix catkin clean --all-profiles when not at workspace root (#673)
* Fix `catkin create pkg` without license parameter (#671) 
* Support building from a symlinked workspace (#669)
* Use loadavg over the last 1 minute (#668)
* Fix shell completion install locations (#652)
* Fix blank lines in build output (#666)
* Use standard python function to determine terminal width (#653)
* Fix handling of invalid package.xml files (#660)
* Fixes for extending profiles (#658)
* escape whitespaces in `catkin config` printout (#657)
* updates to zsh completion (#609)
* Ignore catkin_tools_prebuild package in build space (#650)
* fix 'catkin locate' for symlinked pkgs inside workspace (#624)
* Report circular dependencies detected by topological_order_packages() (#617)
* Add `--this` option to `clean` verb (#623)
* In catkin build, preserve original job list topological ordering (#626)
* Fail build if jobs were abandoned (#644)
* Fix installation of new cmake files (#615)
* Abort with error message on circular dependency. (#641)
* Changed yield from lock to await for Python 3.9 compat (#635)
* Remove older py35+xenial config and add py39+focal (#637)
* Install python2 before travis runs on Focal. (#639)
* Bump cmake min ver to 2.8.12 (#634)
* Fix byte decoding for python 3 (Issue #625) (#627)
* Cleanup of jobs flag parsing (#610, #656, #655)
* Fix get_python_install_dir for Python 2/3 (#601)
* Minor cleanup:
  - import cleanup (#651)
  - remove hack (#659)
  - Add missing space in devel layout error message
  - fix TypeError on executing catkin env (#649)
  - Put a space between 'workspace' and 'and' (#619)
  - Remove redundant 'configuration' in mechanics.rst (#646)
  - Use PYTHONASYNCIODEBUG instead of TROLLIUSDEBUG (#661)
* Contributors: Akash Patel, Guglielmo Gemignani, Ivor Wanders, Kevin Jaget, Lucas Walter, Mathias Lüdtke, Matthijs van der Burgh, Mike Purvis, Robert Haschke, Simon Schmeisser, Tim Rakowski, Timon Engelke, Vojtech Spurny, ckurtz22, mobangjack, pseyfert, xiaxi, zig-for

0.6.0 (2020-06-03)
------------------
* This release restores the 0.4.5 state due to an accident with the 0.5.0 release where we pushed it to Python2 users which it doesn't support.

0.5.0 (2020-06-02)
------------------
* Revert "jobs: Fixing environment required to run catkin test targets on pre-indigo catkin" (`#600 <https://github.com/catkin/catkin_tools/issues/600>`_)
  This reverts commit 050de443da4a65be23b6f62a954efea60db53d09 and partially c5daf4a6d4cb02ab7c22a553a565bddadbaa2a45.
* Doc updates: usage of ``-d`` on cheat sheet, updated verb .txt files (`#604 <https://github.com/catkin/catkin_tools/issues/604>`_)
  - The ``-d|--directory`` feature was introduced in `#572 <https://github.com/catkin/catkin_tools/issues/572>`_
  - Also ran ``dump_cli`` to update help literally included on verb documentation
  pages. This updated more than just ``docs/verbs/cli/catkin_list.txt``, but they
  were in need of an update
* Add ``--this`` option to ``locate`` verb (addresses `#552 <https://github.com/catkin/catkin_tools/issues/552>`_) (`#605 <https://github.com/catkin/catkin_tools/issues/605>`_)
* Merge pull request `#608 <https://github.com/catkin/catkin_tools/issues/608>`_ from mikepurvis/fix-build-type
  Check build_type with function from catkin_pkg.
* Flake8 fixes for new warnings.
* Switch to get_build_type from catkin_pkg.
* Add failing test for an ament/catkin package.
* Allow extending profiles and merging their contents. (`#592 <https://github.com/catkin/catkin_tools/issues/592>`_)
* Updated regex for extracting '-l' and '-j' args for make (`#573 <https://github.com/catkin/catkin_tools/issues/573>`_)
  * Updated regex for extracting '-l' and '-j' args for make
  Summary of improvements:
  - No longer consuming args that happen to begin ``jX`` or ``lX`` (where ``X`` is a
  sequence if integers.
  - Fixed bug where an empty 'j' arg would throw a ``ValueError`` when trying to
  convert an empty string (``''``) to an int
  - Both ``-l/--load-average`` and ``-j/--jobs`` can now be empty (which is
  supported by GNU Make
  - The ``-l/--load-average`` arg will now accept a float value (e.g. ``catkin build -l4.0``) which is supported by GNU Make
  * Added test cases for make args regular expressions
  Test cases also helped to make regexes slightly more robust.
* Do not ignore metapackages (fix `#418 <https://github.com/catkin/catkin_tools/issues/418>`_) (`#596 <https://github.com/catkin/catkin_tools/issues/596>`_)
  Metapackages must not be ignored by catkin_tools. The fact that other non-metapackages
  cannot depend on them according to REP-140 does not imply that they do not need to be
  built and installed at all.
* Catch BlockingIOError in job_server. (`#599 <https://github.com/catkin/catkin_tools/issues/599>`_)
* Drop parentheses from yields and returns (`#595 <https://github.com/catkin/catkin_tools/issues/595>`_)
* Use distutils to handle install path (`#569 <https://github.com/catkin/catkin_tools/issues/569>`_)
* Update release suites. (`#564 <https://github.com/catkin/catkin_tools/issues/564>`_)
  * Add cosmic, disco, and eoan suites.
  * Remove old suites.
  * Bump Python3 version to 3.5 in stdeb.
  * Drop jessie as a supported target.
  * Add focal.
  Co-authored-by: Mike Purvis <mpurvis@clearpathrobotics.com>
* Also run tests with Python 3.7 and 3.8 (`#590 <https://github.com/catkin/catkin_tools/issues/590>`_)
  Signed-off-by: Shane Loretz <sloretz@osrfoundation.org>
* Added option to past list of directories to ``list`` verb (`#572 <https://github.com/catkin/catkin_tools/issues/572>`_)
* Fix permissions of created setup files (`#563 <https://github.com/catkin/catkin_tools/issues/563>`_)
* Handle empty file in ``get_metadata`` correctly (`#568 <https://github.com/catkin/catkin_tools/issues/568>`_)
  If the ``metadata_file_path`` exists but the ``metadata_file`` is empty, ``yaml.safe_load(metadata_file)`` will return None. But the caller expects a dictionary which can lead to a crash. (e.g. in https://github.com/catkin/catkin_tools/blob/master/catkin_tools/verbs/catkin_build/cli.py#L371)
* add authors/maintainers/licenses to context, which are the default values on package creation (`#548 <https://github.com/catkin/catkin_tools/issues/548>`_)
* argument parser: use None instead of [] for empty jobs args (`#575 <https://github.com/catkin/catkin_tools/issues/575>`_)
* Fix space-suffix option (`#560 <https://github.com/catkin/catkin_tools/issues/560>`_)
* Bash completion for run_tests verb (`#562 <https://github.com/catkin/catkin_tools/issues/562>`_)
  * bash completion for run_tests and options
  * also completing packages
* Revert "Set posix=False when parsing environment variables (`#578 <https://github.com/catkin/catkin_tools/issues/578>`_)" (`#584 <https://github.com/catkin/catkin_tools/issues/584>`_)
  This reverts commit 5b121260a75ffcda863d4588a8e9efc7b4945477.
* Add Travis CI badge to README.md (`#585 <https://github.com/catkin/catkin_tools/issues/585>`_)
* Support globs in build/blacklist/whitelist (`#576 <https://github.com/catkin/catkin_tools/issues/576>`_)
  * Add glob patterns to build, blacklist and whitelist
  * Add glob patterns to clean
  * Add test for glob pattern build
* Set posix=False when parsing environment variables (`#578 <https://github.com/catkin/catkin_tools/issues/578>`_)
* Revert `#561 <https://github.com/catkin/catkin_tools/issues/561>`_ (`#582 <https://github.com/catkin/catkin_tools/issues/582>`_)
  Now that the project is only targetting python 3.5+, `#561 <https://github.com/catkin/catkin_tools/issues/561>`_ is no longer required and we can use the most up-to-date version of ``sphinx`` and ``sphinxcontrib-spelling``.
* Do not load the environment when calling profile to allow to change the profile if the current profile is invalid (`#566 <https://github.com/catkin/catkin_tools/issues/566>`_)
* Set Travis to check Python 3.5 and 3.6. (`#580 <https://github.com/catkin/catkin_tools/issues/580>`_)
* Python 3 only, migrate from trollius to native asyncio (`#574 <https://github.com/catkin/catkin_tools/issues/574>`_)
* fix(controllers): python3 fix for the controllers.py file (`#565 <https://github.com/catkin/catkin_tools/issues/565>`_)
* Force sphinxcontrib-spelling less than 4.3.0 (`#561 <https://github.com/catkin/catkin_tools/issues/561>`_)
* fix(docs) mechanics to linked develspace (`#546 <https://github.com/catkin/catkin_tools/issues/546>`_)
* Contributors: Alexis Paques, Christian Henkel, Fan Jiang, Jan Strohbeck, Jarvis Schultz, Johannes Meyer, Leroy Rügemer, Matt Reynolds, Matthijs van der Burgh, Mike Purvis, Moritz Münst, Ramon Wijnands, Shane Loretz, Steven! Ragnarök, Tim de Jager, Timon Engelke, Yuki Furuta

0.4.5 (2019-04-02)
------------------
* Fixed jobserver not working with GNU make >= 4.2 (`#480 <https://github.com/catkin/catkin_tools/issues/480>`_)
* Use yaml.safe_load everywhere. (`#542 <https://github.com/catkin/catkin_tools/issues/542>`_)
* Fixed unhandled config context options warning (`#489 <https://github.com/catkin/catkin_tools/issues/489>`_)
* Added some exception handling for io (`#529 <https://github.com/catkin/catkin_tools/issues/529>`_)
* Added opt --active to profile list (`#513 <https://github.com/catkin/catkin_tools/issues/513>`_)
* Added build type switch to cheat sheet. (`#522 <https://github.com/catkin/catkin_tools/issues/522>`_)
* Fix issue when empty install prefix specified (`#533 <https://github.com/catkin/catkin_tools/issues/533>`_)
* Fix bug when workspace contains special characters (`#536 <https://github.com/catkin/catkin_tools/issues/536>`_)
  Simply sanitize the workspace name when printing it out
* Add local_setup.* to DEVEL_LINK_BLACKLIST (`#539 <https://github.com/catkin/catkin_tools/issues/539>`_)
* Fix notification typo (`#527 <https://github.com/catkin/catkin_tools/issues/527>`_)
* When using a non-standard shell fix the usage of DEFAULT_SHELL. (`#511 <https://github.com/catkin/catkin_tools/issues/511>`_)
  Previously, using a non-standard shell meant using that non-standard shell's path (shell_path) whereas the shell_name would be bash. Not only is there this discrepancy, but then the command is actually run with the non-standard shell, which is liable to cause runtime errors (as it does if e.g. SHELL=/usr/bin/fish).
* Sort authors/maintainers for correct email assignment (`#492 <https://github.com/catkin/catkin_tools/issues/492>`_)
* Fixed 'pip install --user catkin_tools' (`#488 <https://github.com/catkin/catkin_tools/issues/488>`_)
* Fixed 'catkin profile set ...' error message (`#487 <https://github.com/catkin/catkin_tools/issues/487>`_)
* Respect VERBOSE environment variable if already set before in verbose mode (`#506 <https://github.com/catkin/catkin_tools/issues/506>`_)
* Re-enable Sphinx spell check. (`#461 <https://github.com/catkin/catkin_tools/issues/461>`_)" (`#462 <https://github.com/catkin/catkin_tools/issues/462>`_)
* Silence E722 flake8 warnings, fix OSX builds. (`#509 <https://github.com/catkin/catkin_tools/issues/509>`_)
* setup.py: Exclude all tests* from install (`#499 <https://github.com/catkin/catkin_tools/issues/499>`_)
* Fixed typo: relateive -> relative (`#484 <https://github.com/catkin/catkin_tools/issues/484>`_)
* Sanitizes log message input (`#479 <https://github.com/catkin/catkin_tools/issues/479>`_)
* Enable arguments with spaces in alias definition (`#476 <https://github.com/catkin/catkin_tools/issues/476>`_)
* Fixed zsh completion to actually use zsh caching for package list (`#459 <https://github.com/catkin/catkin_tools/issues/459>`_) (`#475 <https://github.com/catkin/catkin_tools/issues/475>`_)
* Fixed doc and completion based on current CLI for interleave output (`#467 <https://github.com/catkin/catkin_tools/issues/467>`_)
  The current CLI for interleave output proposes to use
  --interleave-output instead of --interleave.
* Add trollius to setup.py `install_requires` list (`#474 <https://github.com/catkin/catkin_tools/issues/474>`_)
  closes `#445 <https://github.com/catkin/catkin_tools/issues/445>`_
  This patch can be viewed as continuing the work of pull request `#282 <https://github.com/catkin/catkin_tools/issues/282>`_.
* Fix typo in documentation of option env-cache (`#466 <https://github.com/catkin/catkin_tools/issues/466>`_)
* Implementation of pluggable spaces. (`#458 <https://github.com/catkin/catkin_tools/issues/458>`_)
* Disable Sphinx spell check for now. (`#461 <https://github.com/catkin/catkin_tools/issues/461>`_)
* Better message when missing a required command line tool. (`#455 <https://github.com/catkin/catkin_tools/issues/455>`_)
* Specify return code when build interrupted. (`#452 <https://github.com/catkin/catkin_tools/issues/452>`_)
* Drop utf-8 encoding to compute file hashes in symlink stage (`#399 <https://github.com/catkin/catkin_tools/issues/399>`_)
* Fix logic which merges environment PATH variables. (`#449 <https://github.com/catkin/catkin_tools/issues/449>`_)
* Ignore vim swap files. (`#450 <https://github.com/catkin/catkin_tools/issues/450>`_)
* Add tests for isolated builds. (`#444 <https://github.com/catkin/catkin_tools/issues/444>`_)
* Correctly merge envvars from isolated workspaces. (`#443 <https://github.com/catkin/catkin_tools/issues/443>`_)
* Fix hanging on circular run depend. (`#440 <https://github.com/catkin/catkin_tools/issues/440>`_)
* Contributors: Chris Lalancette, Christian Muck, Felix Widmaier, Florian Tschopp, Hervé Audren, Ian Taylor, JD Yamokoski, Jeremie Deray, Johannes Meyer, Jonathan Bohren, Manuel Binna, Mikael Arguedas, Mike Purvis, Robert Haschke, Scott C. Livingston, Simon Deleersnijder, Tim Rakowski, Tommi, William Woodall, Xfel, luisrayas3

0.4.4 (2017-02-08)
------------------
* Removed unused dependency on ``sphinxcontrib-ansi`` (`#432 <https://github.com/catkin/catkin_tools/issues/432>`_)
* Fixed a small bug in a log message (`#428 <https://github.com/catkin/catkin_tools/issues/428>`_)
* Changed the way symlinks from the private devel spaces were made to better support Python development (`#377 <https://github.com/catkin/catkin_tools/issues/377>`_)
* Fixed a unicode error which occurred when there was unicode output from the compiler (`#368 <https://github.com/catkin/catkin_tools/issues/368>`_)
* Fixed race condition in build related to reading of install space (fixes `#378 <https://github.com/catkin/catkin_tools/issues/378>`_) (`#391 <https://github.com/catkin/catkin_tools/issues/391>`_)
* stderr output from the compiler is now output to stderr by catkin tools to better support integration with IDE's (`#400 <https://github.com/catkin/catkin_tools/issues/400>`_) (`#424 <https://github.com/catkin/catkin_tools/issues/424>`_)
* Improved handling of situation where SHELL environment variable does not exist (`#414 <https://github.com/catkin/catkin_tools/issues/414>`_) (`#421 <https://github.com/catkin/catkin_tools/issues/421>`_)
* Contributors: Jonathan Bohren, Robert Haschke, @dominiquehunziker, Timothee Cour, Mike Purvis

0.4.3 (2017-01-05)
------------------
* Deprecated ``catkin --locate-extra-shell-verbs`` in favor of ``catkin locate --shell-verbs`` (`#352 <https://github.com/catkin/catkin_tools/issues/352>`_)
* Fixed regression in red catkin icon on error feature (`#346 <https://github.com/catkin/catkin_tools/issues/346>`_)
* Fixed a bug in the execution of jobs and display of active status (`#351 <https://github.com/catkin/catkin_tools/issues/351>`_)
* Fixed a bug in environment cache checking (`#353 <https://github.com/catkin/catkin_tools/issues/353>`_)
* Fixed a bug in display of build times over one hour (`#357 <https://github.com/catkin/catkin_tools/issues/357>`_)
* Notifications are now coalesced into a single notification (`#358 <https://github.com/catkin/catkin_tools/issues/358>`_)
* Improvements to shell completion and zsh specific completions (`#365 <https://github.com/catkin/catkin_tools/issues/365>`_)
* Various typos fixed.
* Now uses ``ioctl()`` to determine the terminal width on some platforms (`#415 <https://github.com/catkin/catkin_tools/issues/415>`_) (`#416 <https://github.com/catkin/catkin_tools/issues/416>`_)
* Contributors: Jonathan Bohren, Robert Haschke, Claudio Bandera, Kei Okada, Andreas Hertle, David V. Lu!!, Timo Röhling, G.A. vd. Hoorn

0.4.2 (2016-04-19)
------------------
* Revert `#344 <https://github.com/catkin/catkin_tools/issues/344>`_ until a better fix can be made.

0.4.1 (2016-04-19)
------------------
* Add test for unicode in env (`#345 <https://github.com/catkin/catkin_tools/issues/345>`_)
  Regression test for issue `#338 <https://github.com/catkin/catkin_tools/issues/338>`_.
* Fixed mishandling of environments with unicode values (`#342 <https://github.com/catkin/catkin_tools/issues/342>`_)
* Fixed bug where a long delay occurred when using a slow status rate (`#344 <https://github.com/catkin/catkin_tools/issues/344>`_)
* Contributors: Jonathan Bohren, Steven Peters

0.4.0 (2016-04-18)
------------------
* Major refactor of the job execution engine to use Trollius/Asyncio.
  * Changed the way build environments are generated (no more ``build.sh``).
* Added new "Linked-devel" space option, where the ``devel`` space for each package is isolated, but are symlinked to a single merged ``devel`` space afterwards.
* Added support for cleaning and partial cleaning of the workspace with ``catkin clean``.
* Added "shell verbs" like ``catkin cd`` and ``catkin source`` (requires sourcing of shell files).
* Added support for (and testing for) ``DESTDIR``.
* Warnings are now captured and reported to the console even without ``--verbose``.
* Fixed ``setup.py`` installation when using ``--user``.
* Fixed an issue where CMake was always rerun even when the settings didn't change.
* Added support for the ``.built_by`` marker file to detect when being used at the same time as ``catkin_make[_isolated]``.
* Fixed ``catkin create -p``.
* Improved error message when a circular dependency in the packages is detected.
* Fixed a problem where ``catkin config`` could incorrectly clear the make arguments.
* Fixed a bug where the UI could get stuck on "calculating new jobs".
* Fixed a bug where the ``--isolated-devel`` option would crash when building a subset of the workspace.
* Fixed the "leaf_sources out of bounds" error.
* Moved log files out of ``build/logs`` into "log space" in the workspace root.
* Added `env` utility verb for querying environment and running commands in a modified environment.
* Build types (i.e. cmake, catkin) are now supplied through ``entry_points``.
* Added "environemnt caching" for build jobs to speed up building in some cases.
* Contributors: Alexander Schaefer, Dave Coleman, Dirk Thomas, Esteve Fernandez, Ivor Wanders, Jonathan Bohren, Kartik Mohta, Kei Okada, Kentaro Wada, Robert Haschke, Steven Peters, William Woodall

0.3.1 (2015-12-20)
------------------
* Added some new shell based verbs, i.e. ``catkin cd`` and ``catkin source``.
  `#244 <https://github.com/catkin/catkin_tools/pull/244>`_
  `#192 <https://github.com/catkin/catkin_tools/pull/192>`_
* Use a red icon when a build fails in the notifications.
  `#246 <https://github.com/catkin/catkin_tools/pull/246>`_
* Changed how and where shell completion files are installed.
* Improvements to support ``DESTDIR``.
  `#240 <https://github.com/catkin/catkin_tools/pull/240>`_
* Added a cross tool check to warn users when they are using ``catkin_tools`` in conjunction with either ``catkin_make`` or ``catkin_make_isolated``.
  `#214 <https://github.com/catkin/catkin_tools/pull/214>`_
* Use ``/bin/bash`` as a fallback when the ``SHELL`` environment variable is not set.
  `#239 <https://github.com/catkin/catkin_tools/pull/239>`_
  `#243 <https://github.com/catkin/catkin_tools/pull/243>`_
* Fix error when ``TERM`` doesn't match (through ``ssh`` for example).
  `#232 <https://github.com/catkin/catkin_tools/pull/232>`_

0.3.0 (2015-04-21)
------------------
* Added support for architecture specific libraries directories, a la ``GNUInstallDirs``.
  `#156 <https://github.com/catkin/catkin_tools/pull/156>`_
* Fixed a bug in the implementation of the ``--this`` option of the ``catkin build`` verb.
  `#162 <https://github.com/catkin/catkin_tools/pull/162>`_
* Fixed parsing of and added options that append, remove, or clear arugments which are actually lists, e.g. ``--cmake-args``.
  `#147 <https://github.com/catkin/catkin_tools/pull/147>`_
  `#179 <https://github.com/catkin/catkin_tools/pull/179>`_
* Moved the ANSI color related options to the ``catkin`` command and out of the ``catkin build`` verb.
  `#158 <https://github.com/catkin/catkin_tools/pull/158>`_
* Fixed a bug where the ``--this`` command could look outside of the workspace.
  `#169 <https://github.com/catkin/catkin_tools/pull/169>`_
* Improved the perfomance of listing the result spaces by only loading the environement when asked and caching when needed.
  `#174 <https://github.com/catkin/catkin_tools/pull/174>`_
  `#185 <https://github.com/catkin/catkin_tools/pull/185>`_
  `#190 <https://github.com/catkin/catkin_tools/pull/190>`_
* Added support for blacklisting and whitelisting packages.
  `#175 <https://github.com/catkin/catkin_tools/pull/175>`_
* Some warnings from ``catkin_pkg`` are now suppressed in some verbs. Requires ``catkin_pkg`` >= 0.2.8.
  `#163 <https://github.com/catkin/catkin_tools/pull/163>`_
* Added an internal implementation of the GNU Make server which consolidates jobs amoungst multiple runs of ``make``.
  This has the affect of limiting the total number of jobs make is running even when using a large ``-p`` value.
  This changes the default behavior of the tool, to get the old behavior simply add ``--no-jobserver`` to ``catkin build``.
  This can be set in your build profile with ``catkin config``, or you could use a verb alias to always pass it.
  In general this new default behavior should prevent systems from being brought to their knees by ``catkin build``.
  `#155 <https://github.com/catkin/catkin_tools/pull/155>`_
* Added the ``catkin locate`` verb.
  `#165 <https://github.com/catkin/catkin_tools/pull/165>`_
* Added bash and zsh shell completion.
  `#168 <https://github.com/catkin/catkin_tools/pull/168>`_

0.2.2 (2015-03-09)
------------------
* Added the ``--no-color`` option to the build verb which forces ``catkin build`` to not output color.
* Fixed a bug in a console message.

0.2.1 (2015-02-23)
------------------
* Added options ``--continue-on-error`` and ``--summarize`` (`#138 <https://github.com/catkin/catkin_tools/pull/138>`_)
* Added option for limiting status line updates, ``--status-rate`` (`#141 <https://github.com/catkin/catkin_tools/pull/141>`_)
* Made small fixes to the generated documentation.
* Fixed a bug where ``run_depends`` were not considered in topological ordering.
* Consolidated functions to calculate terminal width.
* Improved failure condition of missing ``cmake`` and ``make`` cli tools.
