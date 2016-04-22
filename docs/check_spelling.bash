#!/usr/bin/env bash

if [ ! "$TRAVIS_OS_NAME" == "osx" ]; then
  sphinx-build -b spelling . build -t use_spelling
fi

