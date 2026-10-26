# Shared linter setup, include() it in a package's BUILD_TESTING block before
# ament_lint_auto_find_test_dependencies(), so all packages lint the same way.
# The other ament_lint_common linters (lint_cmake, flake8, pep257) run
# unchanged.
# The package.xml needs <test_depend> on ament_cmake_clang_format and
# ament_cmake_xmllint.

# The house style is clang-format's LLVM style, not the ROS uncrustify style:
# check it with clang-format and the repo's .clang-format instead (#5)
list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
set(ament_cmake_clang_format_CONFIG_FILE
  "${CMAKE_CURRENT_LIST_DIR}/../.clang-format")

# xmllint fetches the package.xml schema from download.ros.org, which fails
# without network. Run it here instead of through ament_lint_auto, with a
# catalog that points xmllint at the local copy of the schema. schema/*.xsd
# are copied unchanged from github.com/ros-infrastructure/rep/tree/master/xsd
# (commit 3486e23), which is what download.ros.org/schema serves
list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_xmllint)
find_package(ament_cmake_xmllint REQUIRED)
ament_xmllint()
set_tests_properties(xmllint PROPERTIES
  ENVIRONMENT "XML_CATALOG_FILES=${CMAKE_CURRENT_LIST_DIR}/xml_catalog.xml")

# cppcheck runs through ament_lint_auto, with two changes (#57):
# - ament_cppcheck refuses cppcheck 1.88 and 2.x ("known performance issues")
#   and then reports every file as skipped. Ubuntu 24.04 ships 2.13, which
#   takes a few seconds on these packages, so allow it. The cppcheck test only
#   exists after ament_package(), so this is deferred to the end of the
#   CMakeLists.txt. Packages without C/C++ files get no cppcheck test.
# - Findings in test/ are suppressed: cppcheck doesn't know the gtest macros
#   (TEST, TEST_P) and parses test/arduino_stub/Arduino.h as C, which gives
#   syntaxError false positives. ament_cppcheck passes the pattern as
#   --suppress=*:test/*, so it has to be relative (absolute paths have no
#   effect). The test/ files are still parsed and listed as checked in the
#   results, they just can't fail the test.
set(ament_cmake_cppcheck_ADDITIONAL_EXCLUDE "test/*")
function(_mobile_robot_lint_cppcheck_env)
  if(TEST cppcheck)
    set_property(TEST cppcheck APPEND PROPERTY
      ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
  endif()
endfunction()
cmake_language(DEFER CALL _mobile_robot_lint_cppcheck_env)
