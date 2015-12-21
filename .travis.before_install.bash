#!/usr/bin/env bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  echo "No Linux-specific before_install steps."
elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
  if [ "$PYTHON" == "/usr/local/bin/python3" ]; then
    brew install python3
  fi
  sudo pip install virtualenv
  virtualenv -p $PYTHON venv
  source venv/bin/activate
fi
