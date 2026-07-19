#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "science_control::science_2026" for configuration ""
set_property(TARGET science_control::science_2026 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(science_control::science_2026 PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libscience_2026.so"
  IMPORTED_SONAME_NOCONFIG "libscience_2026.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS science_control::science_2026 )
list(APPEND _IMPORT_CHECK_FILES_FOR_science_control::science_2026 "${_IMPORT_PREFIX}/lib/libscience_2026.so" )

# Import target "science_control::keyboard_teleop" for configuration ""
set_property(TARGET science_control::keyboard_teleop APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(science_control::keyboard_teleop PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/science_control/keyboard_teleop"
  )

list(APPEND _IMPORT_CHECK_TARGETS science_control::keyboard_teleop )
list(APPEND _IMPORT_CHECK_FILES_FOR_science_control::keyboard_teleop "${_IMPORT_PREFIX}/lib/science_control/keyboard_teleop" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
