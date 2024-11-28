
if(realsense2_FOUND)
    add_executable(realsense_live_encoder realsense_live_encoder.cpp)
    target_link_libraries(realsense_live_encoder groot realsense2)

    add_executable(realsense_live_encoder_server realsense_live_encoder_server.cpp)
    target_link_libraries(realsense_live_encoder_server groot realsense2)
endif(realsense2_FOUND)

add_executable(decode_files decode_files.cpp)
target_link_libraries(decode_files groot)


add_executable(stream_decoder stream_decoder.cpp)
target_link_libraries(stream_decoder groot)