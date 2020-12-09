#!/usr/bin/env bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  echo "deb http://archive.ubuntu.com/ubuntu $(lsb_release -cs) main universe restricted" > /etc/apt/sources.list
  echo "deb http://archive.ubuntu.com/ubuntu $(lsb_release -cs)-updates main universe restricted" >> /etc/apt/sources.list
  sudo apt update
  sudo apt install enchant -y
  sudo apt install python2 -y || true
elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
  brew upgrade python
  $PYTHON -m pip install virtualenv
  $PYTHON -m virtualenv -p $PYTHON venv
  brew install enchant
  source venv/bin/activate
fi
