^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package catkin_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
