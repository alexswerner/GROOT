
if(realsense2_FOUND)
    add_executable(realsense_live_encoder realsense_live_encoder.cpp)
    target_link_libraries(realsense_live_encoder groot realsense2)
endif(realsense2_FOUND)

add_executable(decode_files decode_files.cpp)
target_link_libraries(decode_files groot)
