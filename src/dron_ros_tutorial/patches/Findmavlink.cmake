# - Try to find Mavlink
# Once done this will define
#  mavlink_FOUND - System has mavlink
#  mavlink_INCLUDE_DIRS - The mavlink include directories

find_package(PkgConfig)
pkg_check_modules(PC_MAVLINK QUIET mavlink)

find_path(mavlink_INCLUDE_DIR mavlink/config.h
          HINTS ${PC_MAVLINK_INCLUDEDIR} ${PC_MAVLINK_INCLUDE_DIRS}
          PATH_SUFFIXES libxml2 )

set(mavlink_INCLUDE_DIRS ${mavlink_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(mavlink DEFAULT_MSG mavlink_INCLUDE_DIR)

mark_as_advanced(mavlink_INCLUDE_DIR)
