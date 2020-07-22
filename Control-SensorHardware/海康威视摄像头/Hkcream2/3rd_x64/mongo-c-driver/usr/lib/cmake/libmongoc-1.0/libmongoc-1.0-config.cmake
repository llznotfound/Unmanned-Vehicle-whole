# Copyright 2017 MongoDB Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

message(WARNING "This CMake target is deprecated.  Use 'mongo::mongoc_shared' instead.  Consult the example projects for further details.")

set (MONGOC_MAJOR_VERSION 1)
set (MONGOC_MINOR_VERSION 16)
set (MONGOC_MICRO_VERSION 0)
set (MONGOC_VERSION 1.16.0-pre)

find_package (libbson-1.0 "1.16" REQUIRED)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was libmongoc-1.0-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set_and_check (MONGOC_INCLUDE_DIRS "${PACKAGE_PREFIX_DIR}/include/libmongoc-1.0")
list (APPEND MONGOC_INCLUDE_DIRS ${BSON_INCLUDE_DIRS})

# We want to provide an absolute path to the library and we know the
# directory and the base name, but not the suffix, so we use CMake's
# find_library () to pick that up.  Users can override this by configuring
# MONGOC_LIBRARY themselves.
find_library (MONGOC_LIBRARY mongoc-1.0 PATHS "${PACKAGE_PREFIX_DIR}/lib" NO_DEFAULT_PATH)
set (MONGOC_LIBRARIES ${MONGOC_LIBRARY} ${BSON_LIBRARIES})

# If this file is generated by the Autotools on Mac, SSL_LIBRARIES might be
# "-framework CoreFoundation -framework Security". Split into a CMake array
# like "-framework CoreFoundation;-framework Security".
set (IS_FRAMEWORK_VAR 0)
foreach (LIB
   secur32.lib;crypt32.lib;Shlwapi.lib secur32.lib;crypt32.lib;Bcrypt.lib  Dnsapi
   D:/Programming_Software/anaconda/Library/lib/snappy.lib D:/Programming_Software/anaconda/Library/lib/icuuc.lib  
)
   if (LIB STREQUAL "-framework")
      set (IS_FRAMEWORK_VAR 1)
      continue ()
   elseif (IS_FRAMEWORK_VAR)
      list (APPEND MONGOC_LIBRARIES "-framework ${LIB}")
      set (IS_FRAMEWORK_VAR 0)
   else ()
      list (APPEND MONGOC_LIBRARIES ${LIB})
   endif ()
endforeach ()

set (MONGOC_DEFINITIONS ${BSON_DEFINITIONS})
