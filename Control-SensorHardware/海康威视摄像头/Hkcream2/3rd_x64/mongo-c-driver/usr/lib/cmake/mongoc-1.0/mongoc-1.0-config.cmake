include(CMakeFindDependencyMacro)
find_dependency(bson-1.0 1.16.0)
include("${CMAKE_CURRENT_LIST_DIR}/mongoc-targets.cmake")
