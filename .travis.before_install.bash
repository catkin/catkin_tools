#!/usr/bin/env bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  echo "deb http://archive.ubuntu.com/ubuntu $(lsb_release -cs) main universe restricted" > /etc/apt/sources.list
  echo "deb http://archive.ubuntu.com/ubuntu $(lsb_release -cs)-updates main universe restricted" >> /etc/apt/sources.list
  sudo apt update
  sudo apt install enchant -y
elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
  if [ "$PYTHON" == "/usr/local/bin/python3" ]; then
    # Brewed Python 3.
    brew upgrade python
  elif [ "$PYTHON" == "/usr/local/bin/python2" ]; then
    # Brewed Python 2.
    brew install python@2
  fi
  $PYTHON -m pip install virtualenv
  $PYTHON -m virtualenv -p $PYTHON venv
  brew install enchant
  source venv/bin/activate
fi
