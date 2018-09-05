# - Try to find CDD
# Once done this will define
#  CDD_FOUND - System has CDD
#  CDD_INCLUDE_DIRS - The CDD include directories
#  CDD_LIBRARIES - The libraries needed to use CDD

find_path(CDD_INCLUDE_DIR NAMES cdd/cdd.h PATHS "/usr/include" "~/local/include" "/usr/local/include")

find_library(CDD_LIBRARY NAMES CDD cdd PATHS "${CDD_INCLUDE_DIR}/lib")

set(CDD_INCLUDE_DIRS ${CDD_INCLUDE_DIR})
set(CDD_LIBRARIES ${CDD_LIBRARY})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CDD_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CDD DEFAULT_MSG CDD_LIBRARY CDD_INCLUDE_DIR)

mark_as_advanced(CDD_INCLUDE_DIR CDD_LIBRARY CDD_CXX_LIBRARY)
