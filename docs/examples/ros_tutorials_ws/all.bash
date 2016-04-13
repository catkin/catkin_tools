#!/usr/bin/env bash

SLOWRECORD=$(pwd)/slowrecord
WS=/tmp/failure_ws

pushd `dirname $0`

rm -rf $WS

source /opt/ros/indigo/setup.bash

$SLOWRECORD --check --tall 0_checkout.bash
$SLOWRECORD --check --tall --buffer 1_init.bash
$SLOWRECORD --check --tall --buffer 2_dry_run.bash
$SLOWRECORD --check --tall 3_build.bash
$SLOWRECORD --check --tall 4_build_v.bash
$SLOWRECORD --check --tall 5_build_i.bash
$SLOWRECORD --check --tall 6_build_partial.bash
$SLOWRECORD --check --tall 7_build_this.bash
$SLOWRECORD --check --tall 8_build_start_with.bash
$SLOWRECORD --check --tall 9_build_no_deps.bash

popd
