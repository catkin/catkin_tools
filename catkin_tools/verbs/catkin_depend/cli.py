#!/usr/bin/env python
# ********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2014, University of Colorado, Boulder
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the University of Colorado Boulder
#     nor the names of its contributors may be
#     used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# ********************************************************************/

#   Author: Dave Coleman
#   Desc:   Simple tool to add a package dependency to the 4 locations in a catkin package
#
#   Current Limitations:
#
# - You don't mind formatting of your package.xml being messed up.
# - You don't mind the deps being added to the bottom of package.xml
# - 'find_package(catkin REQUIRED COMPONENTS' is on the same line

from __future__ import print_function

import os
import sys
import re
from lxml import etree

def prepare_arguments(parser):
   add = parser.add_argument

   add('dependencies', nargs='*',
      help="one or more dependencies to add to a package")

   return parser

def main(opts):
   deps = opts.dependencies
   for dep_name in deps:
      print('Adding dependency: ' + dep_name)
 
   # package.xml ----------------------------------------------------------------------------

   # Add dep to package manifest
   parser = etree.XMLParser(remove_blank_text=False)
   package_file_name = os.getcwd() + "/package.xml"
   #print ' - Found package.xml at '+ package_file_name
   package_xml = etree.parse(package_file_name, parser)

   # Make sure at least all required dependencies are in the depends lists
   build_deps = deps
   run_deps   = deps

   def update_deps(reqd_deps, req_type, e_parent):
      curr_deps = [e.text for e in e_parent.findall(req_type)]
      missing_deps = set(reqd_deps) - set(curr_deps)
      for d in missing_deps:
         etree.SubElement(e_parent, req_type).text = d
      return missing_deps

   # empty sets evaluate to false
   modified_pkg  = update_deps(build_deps, "build_depend", package_xml.getroot())
   modified_pkg |= update_deps(run_deps, "run_depend", package_xml.getroot())

   if modified_pkg:
      with open(package_file_name,"w") as f:
         package_xml.write(f, xml_declaration=True, pretty_print=True)      
      print(" - Modified package.xml")


   # CMakeLists.txt ----------------------------------------------------------------------------

   # Check if CMakeLists file exists
   cmake_file = os.getcwd() + "/CMakeLists.txt"
   if not os.path.exists(cmake_file):
      print( '\nERROR: can\'t find CMakeLists file at \'' + cmake_file + '\'\n')
      sys.exit(-1)

   # Add dep to CMakeLists.txt
   template_file_data = open(cmake_file, 'r')
   template_text = template_file_data.read() 

   def search_cmake(search_string,replace_string,template_text):         
      (template_text, num_subs_made) = re.subn(search_string, replace_string, template_text)
                                           
      if not num_subs_made:
      	print( '\nERROR: can\'t find the string "' + search_string + '"\n')
        print( 'Unable to modify CMakeLists.txt')
        sys.exit(-1)
        
      return template_text

   for dep_name in deps:
      # run two modifications to CMakeLists
      template_text = search_cmake('find_package\(catkin REQUIRED COMPONENTS',
                                   'find_package(catkin REQUIRED COMPONENTS\n  '+dep_name,
                                   template_text)
      template_text = search_cmake('CATKIN_DEPENDS', 
                                   'CATKIN_DEPENDS\n    '+dep_name, 
                                   template_text)

   # Save changes
   with open(cmake_file,'w') as f:
      f.write(template_text)
   print( ' - Modified CMakeLists file at \'' + cmake_file + '\'')

   print( '\n')

