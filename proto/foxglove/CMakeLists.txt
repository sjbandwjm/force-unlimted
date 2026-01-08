file(GLOB_RECURSE PROTO_FILES "*.proto")

set(PROTO_EXE ${HOLO_ROOT}/3rdparty/bin/protoc)
message(STATUS "protoc exec: ${PROTO_EXE}")

execute_process(
    COMMAND ${CMAKE_COMMAND} -E env ${PROTO_EXE} --version
    RESULT_VARIABLE PROTOC_RESULT
    OUTPUT_VARIABLE PROTOC_OUTPUT
)
message(STATUS "protoc version: ${PROTOC_OUTPUT}")


if(NEED_GENERATE_PROTO)
    message(STATUS "protoc execute ouput_dir: ${CMAKE_CURRENT_BINARY_DIR}")

    execute_process(
        COMMAND export LD_LIBRARY_PATH=${HOLO_ROOT}/3rdparty/lib:${LD_LIBRARY_PATH}
        RESULT_VARIABLE ret
    )

    foreach(_file ${PROTO_FILES})
        file(RELATIVE_PATH relative_path ${CMAKE_CURRENT_SOURCE_DIR}/../ ${_file})
        message(STATUS "protoc process: ${_file} ${relative_path}")
        execute_process(
            COMMAND ${PROTO_EXE} --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
                                --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/../
                                ${relative_path}
            RESULT_VARIABLE ret
        )

        if(ret AND NOT ret EQUAL 0)
            message(FATAL_ERROR "protoc generate proto failed ${_file}")
        endif()
    endforeach(_file)
endif(NEED_GENERATE_PROTO)

if (NEED_GENERATE_PROTO)
    file(GLOB_RECURSE PROTO_SRCS
        "${CMAKE_CURRENT_BINARY_DIR}/*.cc"
    )
else()
    file(GLOB_RECURSE PROTO_SRCS
        "${CMAKE_CURRENT_SOURCE_DIR}/generate/*.cc"
    )
endif()

add_library(foxglove-proto OBJECT ${PROTO_SRCS})
target_include_directories(foxglove-proto PUBLIC ${HOLO_ROOT}/3rdparty/include)
if (NEED_GENERATE_PROTO)
    target_include_directories(foxglove-proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
else()
    target_include_directories(foxglove-proto PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/generate)
endif()

target_compile_options(foxglove-proto PRIVATE "-w")
target_compile_options(foxglove-proto PRIVATE "-Wno-error=unused-parameter")