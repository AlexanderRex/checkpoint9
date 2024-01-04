# CMake generated Testfile for 
# Source directory: /home/user/ros2_ws/src/checkpoint9/attach_shelf
# Build directory: /home/user/ros2_ws/src/checkpoint9/build/attach_shelf
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/copyright.xunit.xml" "--package-name" "attach_shelf" "--output-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/ament_copyright/copyright.txt" "--command" "/opt/ros/galactic/bin/ament_copyright" "--xunit-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/user/ros2_ws/src/checkpoint9/attach_shelf" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_copyright.cmake;46;ament_add_test;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;18;ament_copyright;/opt/ros/galactic/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;40;ament_package;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/cppcheck.xunit.xml" "--package-name" "attach_shelf" "--output-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/ament_cppcheck/cppcheck.txt" "--command" "/opt/ros/galactic/bin/ament_cppcheck" "--xunit-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "300" WORKING_DIRECTORY "/home/user/ros2_ws/src/checkpoint9/attach_shelf" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;61;ament_add_test;/opt/ros/galactic/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;83;ament_cppcheck;/opt/ros/galactic/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;40;ament_package;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;0;")
add_test(cpplint "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/cpplint.xunit.xml" "--package-name" "attach_shelf" "--output-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/ament_cpplint/cpplint.txt" "--command" "/opt/ros/galactic/bin/ament_cpplint" "--xunit-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/user/ros2_ws/src/checkpoint9/attach_shelf" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;68;ament_add_test;/opt/ros/galactic/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;35;ament_cpplint;/opt/ros/galactic/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;40;ament_package;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/lint_cmake.xunit.xml" "--package-name" "attach_shelf" "--output-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/ament_lint_cmake/lint_cmake.txt" "--command" "/opt/ros/galactic/bin/ament_lint_cmake" "--xunit-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/src/checkpoint9/attach_shelf" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/opt/ros/galactic/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;40;ament_package;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/uncrustify.xunit.xml" "--package-name" "attach_shelf" "--output-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/ament_uncrustify/uncrustify.txt" "--command" "/opt/ros/galactic/bin/ament_uncrustify" "--xunit-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/src/checkpoint9/attach_shelf" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;65;ament_add_test;/opt/ros/galactic/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;34;ament_uncrustify;/opt/ros/galactic/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;40;ament_package;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/xmllint.xunit.xml" "--package-name" "attach_shelf" "--output-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/ament_xmllint/xmllint.txt" "--command" "/opt/ros/galactic/bin/ament_xmllint" "--xunit-file" "/home/user/ros2_ws/src/checkpoint9/build/attach_shelf/test_results/attach_shelf/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/user/ros2_ws/src/checkpoint9/attach_shelf" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/opt/ros/galactic/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/opt/ros/galactic/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/opt/ros/galactic/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;40;ament_package;/home/user/ros2_ws/src/checkpoint9/attach_shelf/CMakeLists.txt;0;")
