#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "despot" for configuration "Debug"
set_property(TARGET despot APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(despot PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdespot.so"
  IMPORTED_SONAME_DEBUG "libdespot.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS despot )
list(APPEND _IMPORT_CHECK_FILES_FOR_despot "${_IMPORT_PREFIX}/lib/libdespot.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
