#!/usr/bin/env bash

if [ "$TRAVIS_OS_NAME" == "linux" ]; then
  echo "AOK"
  #if [ "$PYTHON" == "/usr/bin/python3.4" ]; then
    #sudo add-apt-repository ppa:fkrull/deadsnakes -y
    #sudo apt-get update
    #sudo apt-get install python3.4 python3-dev
  #fi
elif [ "$TRAVIS_OS_NAME" == "osx" ]; then

  # Install python if necessary
  if [ "$PYTHON" == "/usr/local/bin/python" ]; then
    brew install python
    if ! hash virtualenv 2>/dev/null; then sudo pip install virtualenv; fi
  elif [ "$PYTHON" == "/usr/local/bin/python3" ]; then
    brew install python3
    if ! hash virtualenv 2>/dev/null; then sudo pip3 install virtualenv; fi
  fi

  # Install CMake if necessary
  if ! hash cmake 2>/dev/null; then brew install cmake; fi

  virtualenv -p $PYTHON venv
  source venv/bin/activate
fi
