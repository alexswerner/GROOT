#include <librealsense2/rs.hpp>
#include <cstdio>
#include <iostream>
#include <bitset>
#include <fstream>
#include <sstream>
#include <chrono>
#include <math.h>
#include <pcl/filters/passthrough.h>
#include "Frame.hpp"
#include "JpegEncoder.hpp"

// color coding order read from file
int *image_coder_order_;

void compressColors(vector<uint8_t> orig_colors, vector<uint8_t> &compressed_colors, JpegEncoder *jpegEncoder_)
{
    int color_size = orig_colors.size();
    int image_width = 1024;
    int image_height = 1024;

    if (orig_colors.size() / 3 < 1024 * 512)
    {
        image_height = 512;
    }
    else if (orig_colors.size() / 3 >= 1024 * 1024)
    {
        image_width = 2048;
    }

    vector<uint8_t> reordered_color(image_width * image_height * 3, 255);
    for (int i = 0; i < image_width * image_height; i++)
    {
        // for GROOT use this
        int idx = image_coder_order_[i];
        // for SERIAL use this
        // int idx = i;
        if (idx < orig_colors.size() / 3)
        {
            reordered_color[3 * i] = orig_colors[3 * idx];
            reordered_color[3 * i + 1] = orig_colors[3 * idx + 1];
            reordered_color[3 * i + 2] = orig_colors[3 * idx + 2];
        }
    }

    //printf("COmpress color\n");
    //    compressed_colors.resize(orig_colors.size());

    /* for no compression use this
    compressed_colors.resize(reordered_color.size());
    memcpy(&compressed_colors[0], &reordered_color[0], reordered_color.size());*/

    // for GROOT
    jpegEncoder_->encode(reordered_color, compressed_colors, image_width, image_height);

    // FILE* pFile = fopen("test_output_groot.jpeg", "wb");
    // fwrite(&compressed_colors[0], sizeof(uint8_t), compressed_colors.size(), pFile);
}

void readImageCoder(string imagecoder)
{
    int imgSize = 1024;
    image_coder_order_ = new int[imgSize * imgSize];
    FILE *pFile = fopen(imagecoder.c_str(), "rb");
    std::size_t res = fread(&image_coder_order_[0], sizeof(int), imgSize * imgSize, pFile);
    if (res == 0)
        throw std::runtime_error("Failed to read ImageCoder");
    fclose(pFile);
}
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points &points, rs2::video_frame const &color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    auto Texture_Coord = points.get_texture_coordinates();
    for (std::size_t idx = 0; idx < cloud->points.size(); idx++)
    {
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
int main(int argc, char *argv[])
{
    std::string morton_code = argv[1];

    // read in color coder file
    readImageCoder(morton_code);

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

    // prepare jpeg encoder
    JpegEncoder *jpegEncoder_;
    jpegEncoder_ = new JpegEncoder();
    unsigned int avgCompSize = 0;
    Frame currentFrame;
    while (true)
    {
        // cout << "[MAIN] File name " << filelist[i] << endl;
        //  TODO: read input from librealsense frame
        auto frames = pipe.wait_for_frames();
        auto start_time = std::chrono::high_resolution_clock::now();
        auto print_time = [&]()
        {
            auto stop_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = stop_time - start_time;
            std::cout << "time: " << ms_double.count() << std::endl;
        };

        currentFrame.reset();
        std::cout << "After reset" << std::endl;
        print_time();
        

        auto depth = frames.get_depth_frame();
        auto color = frames.get_color_frame();
        rs2::pointcloud pc;
        pc.map_to(color);
        std::cout << "After acqu" << std::endl;
        print_time();
        // This call should direction insert points into the octree
        auto points = pc.calculate(depth);
        std::cout << "After calculate" << std::endl;
        print_time();

        currentFrame.setPointCloud(points_to_pcl(points, color));
        printf("After reading point cloud, size: %li\n", currentFrame.getPointCloud()->points.size());
        print_time();
        float voxelSize = 0.002;
        currentFrame.generateOctree(voxelSize);
        printf("After generating octree\n");
        print_time();
        bool is_user_adaptive = false; // with colors
        bool isShort = false;
        currentFrame.compressPDTree(is_user_adaptive, isShort);
        printf("After pdtree compression\n");
        print_time();
        if (is_user_adaptive)
        {
            currentFrame.generatePayload();
        }
        else
        {
            // printf("[MAIN] Finished geometry compression\n");
            //  TODO move color compression to Frame class
            vector<uint8_t> colors = currentFrame.getColorBytes();
            vector<uint8_t> compressed_colors;
            compressColors(colors, compressed_colors, jpegEncoder_);
            currentFrame.generatePayload(compressed_colors);
            // out_file = out_file + "_enc.bin";
        }
        printf("After color compression\n");
        print_time();
        // printf("[MAIN] write frame\n");
        unsigned int avgCompSize = 0;
        currentFrame.writeFrame("/dev/null", &avgCompSize);
        auto stop_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms_double = stop_time - start_time;
        std::cout << "Compressed frame size: " << avgCompSize << " time: " << ms_double.count() << std::endl;
        std::cout << "After writing frame" << std::endl;
        print_time();
    }

    // printf("[MAIN] Final Compressed Size: %d\n", avgCompSize / filelist.size());
}
