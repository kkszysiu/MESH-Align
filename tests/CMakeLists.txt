# Headless core tests — link ONLY meshalign_core (proves the core has no GL/ImGui).
add_executable(core_tests test_mesh.cpp)
target_link_libraries(core_tests PRIVATE meshalign_core)
add_test(NAME core_tests COMMAND core_tests)

add_executable(io_tests io_tests.cpp)
target_link_libraries(io_tests PRIVATE meshalign_core)
add_test(NAME io_tests COMMAND io_tests)

add_executable(analysis_tests analysis_tests.cpp)
target_link_libraries(analysis_tests PRIVATE meshalign_core)
add_test(NAME analysis_tests COMMAND analysis_tests)

add_executable(picking_tests picking_tests.cpp)
target_link_libraries(picking_tests PRIVATE meshalign_core)
add_test(NAME picking_tests COMMAND picking_tests)

add_executable(deviation_tests deviation_tests.cpp)
target_link_libraries(deviation_tests PRIVATE meshalign_core)
add_test(NAME deviation_tests COMMAND deviation_tests)
