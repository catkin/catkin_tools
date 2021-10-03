# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from shlex import split as cmd_split

catkin_config_path = os.path.join(os.path.expanduser('~'), '.config', 'catkin')

builtin_verb_aliases_content = """\
### DO NOT EDIT, generated automatically and updated automatically by catkin_tools

### If you want to add your own verb aliases, add additional files to this directory
### If you want to override an alias in this file, put the same alias in a new file
### If you want to disable an alias in this file, set the alias to null in a new file
### Files in this folder which end with `.yaml` are evaluated in sorted order

b: build
bt: b --this
ls: list
install: config --install
p: create pkg
run_tests: test
"""


def initialize_verb_aliases(path=catkin_config_path):
    if not os.path.isdir(path):
        raise RuntimeError(
            "Cannot initialize verb aliases because catkin configuration path ('{0}') does not exist or is a file."
            .format(path))
    verb_aliases_path = os.path.join(path, 'verb_aliases')
    if not os.path.isdir(verb_aliases_path):
        if os.path.isfile(verb_aliases_path):
            raise RuntimeError("The catkin verb aliases config directory ('{0}') exists, but is a file."
                               .format(verb_aliases_path))
        os.makedirs(verb_aliases_path)
    builtin_verb_aliases = os.path.join(verb_aliases_path, '00-default-aliases.yaml')
    write_defaults = True
    if os.path.exists(builtin_verb_aliases):
        with open(builtin_verb_aliases, 'r') as f:
            if f.read() != builtin_verb_aliases_content:
                print("Warning, builtin verb aliases at '{0}' differ from builtin, overwriting"
                      .format(builtin_verb_aliases))
            else:
                write_defaults = False
    if write_defaults:
        with open(builtin_verb_aliases, 'w') as f:
            f.write(builtin_verb_aliases_content)


def initialize_config(path=catkin_config_path):
    # Assert that the config path exists, otherwise try to create it
    if not os.path.isdir(path):
        if os.path.isfile(path):
            raise RuntimeError("The catkin config directory ('{0}') exists, but is a file."
                               .format(path))
        os.makedirs(path)
    # Initialize the verbs
    initialize_verb_aliases(path)


def get_verb_aliases(path=catkin_config_path):
    if not os.path.isdir(path):
        raise RuntimeError(
            "Cannot get verb aliases because the catkin config path ('{0}') does not exist or is a file."
            .format(path))
    verb_aliases_path = os.path.join(path, 'verb_aliases')
    if not os.path.isdir(verb_aliases_path):
        raise RuntimeError(
            "Cannot get verb aliases because the verb aliases config path ('{0}') does not exist or is a file."
            .format(verb_aliases_path))
    verb_aliases = {}
    for file_name in sorted(os.listdir(verb_aliases_path)):
        if file_name.endswith('.yaml'):
            full_path = os.path.join(verb_aliases_path, file_name)
            with open(full_path, 'r') as f:
                yaml_dict = yaml.safe_load(f)
            if yaml_dict is None:
                continue
            if not isinstance(yaml_dict, dict):
                raise RuntimeError("Invalid alias file ('{0}'), expected a dict but got a {1}"
                                   .format(full_path, type(yaml_dict)))
            for key, value in yaml_dict.items():
                if not isinstance(key, str):
                    raise RuntimeError("Invalid alias in file ('{0}'), expected a string but got '{1}' of type {2}"
                                       .format(full_path, key, type(key)))
                parsed_value = None
                if isinstance(value, str):
                    # Parse using shlex
                    parsed_value = cmd_split(value)
                elif isinstance(value, list) or isinstance(value, type(None)):
                    # Take plainly
                    parsed_value = value
                else:
                    raise RuntimeError(
                        "Invalid alias expansion in file ('{0}'), expected a string or a list but got '{1}' of type {2}"
                        .format(full_path, value, type(value)))
                verb_aliases[key] = parsed_value
    for alias, value in dict(verb_aliases).items():
        if not value:
            del verb_aliases[alias]
    return verb_aliases
