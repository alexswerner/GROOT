
add_library(groot 
        pdtree-encoder/JpegEncoder.cpp
        pdtree-encoder/Frame.cpp
        pdtree-decoder/Decoder.cpp
        pdtree-decoder/JpegDecoder.cpp
        )
target_link_libraries(groot ${PCL_LIBRARIES} turbojpeg)
