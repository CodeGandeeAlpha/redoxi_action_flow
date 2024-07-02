cmake_minimum_required(VERSION 3.10)
project(vineyard_test_demo)

set(CMAKE_CXX_STANDARD 11)

# 寻找Vineyard库
find_package(vineyard REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(vineyard_test_demo client_cpp.cpp)
add_executable(img_test img_test.cpp)
add_executable(array_two_clients_test array_two_clients_test.cpp)
add_executable(tensor_test tensor_test.cpp)

# 链接Vineyard库
target_include_directories(vineyard_test_demo PRIVATE ${VINEYARD_INCLUDE_DIRS})
target_link_libraries(vineyard_test_demo PRIVATE ${VINEYARD_LIBRARIES})

target_include_directories(array_two_clients_test PRIVATE ${VINEYARD_INCLUDE_DIRS})
target_link_libraries(array_two_clients_test PRIVATE ${VINEYARD_LIBRARIES})

target_include_directories(tensor_test PRIVATE ${VINEYARD_INCLUDE_DIRS})
target_link_libraries(tensor_test PRIVATE ${VINEYARD_LIBRARIES})

target_include_directories(img_test PRIVATE ${VINEYARD_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(img_test PRIVATE ${VINEYARD_LIBRARIES} ${OpenCV_LIBS})