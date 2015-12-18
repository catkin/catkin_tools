#!/usr/bin/env bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  sudo apt-get install cmake build-essential
  #sudo apt-get install cmake libgtest-dev build-essential
elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
  echo "All OSX-specific deps satisfied."
fi
