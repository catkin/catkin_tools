#!/usr/bin/env bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  sudo apt-get install cmake build-essential
elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
  echo "No OS X-specific before_script steps."
fi
