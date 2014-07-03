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

# This set of functions define the interactions with the catkin_tools metadata
# file, `.catkin_tools.yml`. This file can be used by each verb to store
# verb-specific information 

from __future__ import print_function

import os
import yaml

CATKIN_TOOLS_METADATA_FILE = '.catkin_tools.yml'

metadata_file_header = """\
### DO NOT EDIT, generated automatically and updated automatically by catkin_tools
### If you delete this file, it will cause `catkin build` to re-build from scratch. 
### If you modify this file, it may have unintended side-effects on your workspace.
"""

def find_metadata_file(search_path):
    """Find the catkin_tools metadata file starting in the path given by
    search_path and traversing each parent directory until either finding such
    a file or getting to the root of the filesystem.
    
    search_path: Directory which either is a catkin workspace or is contained
                 in a catkin workspace

    returns: Path to the file if found, `None` if not found.
    """
    while True:
        # Check if marker file exists
        candidate_path = os.path.join(search_path,CATKIN_TOOLS_METADATA_FILE)
        if os.path.exists(candidate_path):
            return candidate_path

        # Update search path or end
        (search_path, child_dir) = os.path.split(search_path)
        if len(child_dir) == 0:
            break

    return None

def write_header(stream):
    """Write the metadata file header to a given stream.
    """
    stream.write(metadata_file_header)

def init_metadata_file(workspace_path, force=False):
    """Create a catkin_tools metadata file with no content in a given path.

    workspace_path: The path to the root of a catkin workspace
    force: If true, overwrite an existing metadata file
    """
    
    # Make sure the directory
    if not os.path.exists(workspace_path):
        raise IOError("Error: Can't initialize Catkin workspace in path %s because it does not exist." % (workspace_path))
        
    # Construct the full path to the metadata file
    metadata_file_path = os.path.join(workspace_path, CATKIN_TOOLS_METADATA_FILE)

    # Check if a metadata file already exists
    if os.path.exists(metadata_file_path):
        if force:
            print("Warning: Overwriting existing catkin metadata file: %s" % (metadata_file_path))
        else:
            raise IOError("Catkin metadata file already exists, not overwriting: %s" % (metadata_file_path))

    # Write a metadata file which only contains a header
    with open(metadata_file_path,'w') as metadata_file:
        write_header(metadata_file)

    return metadata_file_path

def get_metadata(metadata_file_path):
    """Get a python structure representing the information stored in the
    catkin_tools metadata file.

    metadata_file_path: The path to a catkin_tools metadata file
    """

    with open(metadata_file_path,'r') as metadata_file:
        return yaml.load(metadata_file)

def update_metadata(metadata_file_path,key,new_data):
    """Update the catkin_tools metadata file located either in search_path or
    in one of the parent directories of search_path.

    metadata_file_path: The path to a catkin_tools metadata file
    key: The top-level key in the metadata file (usually the name of the
         associated verb)
    new_data: A python dictionary or array to write to the metadata file.
    """

    # Get the curent metadata
    data = get_metadata(metadata_file_path) or dict()

    # Update the given key
    data[key] = new_data
    with open(metadata_file_path,'w') as metadata_file:
        write_header(metadata_file)
        yaml.dump(data, metadata_file, default_flow_style=False)

