#!/usr/bin/env bash

SLOWRECORD=$(pwd)/slowrecord
WS=/tmp/failure_ws

pushd `dirname $0`

rm -rf $WS

source /opt/ros/indigo/setup.bash
bash 0_init.bash
$SLOWRECORD --check --tall --buffer 1_build_warning.bash
$SLOWRECORD --check --tall --buffer 2_build_err.bash

popd
