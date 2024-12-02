# Enable testing

project(AML_Test) 

add_executable(${PROJECT_NAME}
    test_main.cpp
    vector3_test.cpp
    matrix3_test.cpp
    dcm_test.cpp
    euler_test.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    AML
)

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_BINARY_DIR}/bin
)