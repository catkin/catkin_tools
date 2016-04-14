Documentation Examples
======================

This document explains how to run examples and generate all static text and
asciinema videos.

## Prerequisites

* [perl](http://perl.org)
* [asciinema](http://asciinema.org)
* [rosinstall\_generator](https://github.com/vcstools/wstool)
* [wstool](https://github.com/vcstools/wstool)
* [catkin](https://github.com/ros/catkin)

## Generating All Examples

All examples must be run from the examples directory.

```bash
./quickstart_ws/all.bash
./failure_ws/all.bash
./ros_tutorials_ws/all.bash
```

## Scripts

### slowrun

The `slowrun` script executes a script line by line, echoing characters to the
console with a delay, as if they were being typed.

Optional arguments:

* `--buffer` -- buffer and delay printing of each line from the output from subcommands

### slowrecord

The `slowrecord` script executes a script line by line with `slowrun`, but also
spawns a `urxvt` terminal with a specific size, and records the commands with
`asciinema`.

Optional arguments:

* `--check` -- check interactively before uploading
* `--tall` -- use a taller window for recording
