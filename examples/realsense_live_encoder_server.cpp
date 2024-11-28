#include "groot/Frame.hpp"
#include <bitset>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <math.h>
#include <pcl/filters/passthrough.h>
#include <sstream>


#include <arpa/inet.h>
#include <netdb.h> /* getprotobyname */
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture,
                                      rs2::texture_coordinate Texture_XY) {
    // Get Width and Height coordinates of texture
    int width = texture.get_width();   // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index = (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t *>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points &points,
                                                     rs2::video_frame const &color) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    auto Texture_Coord = points.get_texture_coordinates();
    for (std::size_t idx = 0; idx < cloud->points.size(); idx++) {
        auto &p = cloud->points.at(idx);
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        auto RGB_Color = RGB_Texture(color, Texture_Coord[idx]);
        p.r = std::get<0>(RGB_Color);
        p.g = std::get<1>(RGB_Color);
        p.b = std::get<2>(RGB_Color);

        ptr++;
    }

    pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
    Cloud_Filter.setInputCloud(cloud);               // Input generated cloud to filter
    Cloud_Filter.setFilterFieldName("z");            // Set field name to Z-coordinate
    Cloud_Filter.setFilterLimits(0.0, 1.0);          // Set accepted interval values
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Cloud_Filter.filter(*newCloud); // Filtered Cloud Outputted

    return newCloud;
}

int main(int argc, char *argv[]) {
    std::string morton_code = argv[1];

    GROOTEncoder enc(morton_code, 0.002);
    // set transformation and voxel size for input dataset

    static rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_device("218622270181");
    pipe.start(cfg);
    auto ir_sensor = pipe.get_active_profile().get_device().first<rs2::depth_sensor>();
    ir_sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET,
                         rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    ir_sensor.set_option(rs2_option::RS2_OPTION_DEPTH_UNITS, 0.001f);

    int server_sockfd = socket(AF_INET6, SOCK_STREAM, 0);
    if (server_sockfd == -1) {
        throw std::runtime_error("Could not open server socket");
    }
    int enable = 1;
    if (setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) < 0) {
        throw std::runtime_error("setsockopt(SO_REUSEADDR) failed");
    }
    struct sockaddr_in6 server_address;
    server_address.sin6_family = AF_INET6;
    server_address.sin6_addr = in6addr_any;
    server_address.sin6_port = htons(2323);
    if (bind(server_sockfd, (struct sockaddr *)&server_address, sizeof(server_address)) == -1) {
        throw std::runtime_error("bind");
    }

    if (listen(server_sockfd, 5) == -1) {
        throw std::runtime_error("listen");
    }
    struct sockaddr_in6 client_address;
    socklen_t client_len = sizeof(client_address);
    int client_sockfd = accept(server_sockfd, (struct sockaddr *)&client_address, &client_len);
    std::cout << "Got a connection" << std::endl;
    while (true) {
        char buffer[5000];
        int nbytes_read = read(client_sockfd, buffer, 132);
        if(nbytes_read==-1)break;
        if(nbytes_read==132) {
            std::cout << "Got ACK" << std::endl;
        }
        //  read input from librealsense frame
        auto frames = pipe.wait_for_frames();
        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        rs2::pointcloud pc;
        pc.map_to(color);
        auto points = pc.calculate(depth);
        auto pointcloud = points_to_pcl(points, color);
        auto start_time = std::chrono::high_resolution_clock::now();
        auto payload = enc.encode(pointcloud);
        auto stop_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms_double = stop_time - start_time;
        std::cout << "Encoding took " << ms_double.count() << "ms resulting in a payload size of "
            << payload.size() << std::endl;
        int nbytes_written = write(client_sockfd, reinterpret_cast<char const *>(payload.data()), payload.size());
        if(nbytes_written==-1)break;
    }
    std::cout << "Connection was closed" << std::endl;
    return 0;
}
