
add_library(groot 
        pdtree-encoder/JpegEncoder.cpp
        pdtree-encoder/Frame.cpp
        pdtree-decoder/Decoder.cpp
        pdtree-decoder/JpegDecoder.cpp
        )
target_link_libraries(groot ${PCL_LIBRARIES} turbojpeg)
set_target_properties(groot PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(groot PROPERTIES PUBLIC_HEADER "${CMAKE_SOURCE_DIR}/include/groot/Decoder.hpp ${CMAKE_SOURCE_DIR}/include/groot/Frame.hpp ${CMAKE_SOURCE_DIR}/include/groot/JpegDecoder.hpp ${CMAKE_SOURCE_DIR}/include/groot/JpegEncoder.hpp")

include(GNUInstallDirs)
install(TARGETS groot
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})