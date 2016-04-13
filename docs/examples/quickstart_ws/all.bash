#!/usr/bin/env bash

SLOWRECORD=$(pwd)/slowrecord
WS=/tmp/quickstart_ws

pushd `dirname $0`

rm -rf $WS
bash 0_quickstart.bash
pushd $WS; catkin clean -y; popd
bash 1_prebuild.bash > 1_prebuild.out
rm -rf $WS
$SLOWRECORD --check --tall --buffer 0_quickstart.bash
bash 2_postbuild.bash > 2_postbuild.out

popd
