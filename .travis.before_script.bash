#!/usr/bin/env bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  sudo apt-get install cmake libgtest-dev build-essential python-setuptools python-pip
  pip install trollius
elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
  # noop?
  pip install setuptools
  pip install trollius
fi
