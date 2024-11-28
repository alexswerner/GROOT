#pragma once
#include <bitset>
#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "JpegEncoder.hpp"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/point_types.h>

using namespace std;
using PointType = pcl::PointXYZRGB;

struct Color {
    int index;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    Color() : index(0), r(0), g(0), b(0) {}
};

struct FrameHeader {
    char frame_type;
    int num_points;
    int num_breadth_bytes;
    int num_breadth_nodes;
    int num_depth_bytes;
    int num_color_bytes;
    int num_icp_nodes;
    int num_icp_points;
    Eigen::Vector3f root_center;
    float root_sidelength;
};

struct FramePayload {
    vector<uint8_t> breadth_bytes;
    vector<uint8_t> depth_bytes;
    vector<uint8_t> breadth_leaf_indices;
    vector<uint8_t> color_bytes;
};

struct Resources {
    uint8_t *fBuffer;
    uint8_t *cBuffer;
    size_t fBufferSize;
    size_t cBufferSize;
};

class Frame {
  public:
    Frame();
    ~Frame() {}

    void readPointCloud(int frame_index, std::string filename, float scale, bool isFlip, float x,
                        float y, float z);
    void writeFrame(std::string filename, unsigned int *avgCompSize);
    void generateOctree(float voxelsize);

    void compressPDTree(int is_user_adaptive, bool isShort);

    vector<uint8_t> getColorBytes();
    pcl::PointCloud<PointType>::Ptr getPointCloud();
    void setPointCloud(pcl::PointCloud<PointType>::Ptr const &pc) { cloud_ = pc; };
    // vector<PointType> getOrderedPoints();
    vector<vector<uint8_t>> getBreadthBytes();

    vector<Eigen::Vector4f> get_debug_point_list();
    vector<uint8_t> get_debug_colors();

    // generate payload with or without color
    void generatePayload();
    void generatePayload(vector<uint8_t> compressed_colors);
    std::vector<uint8_t> extractPayload();

    void reset() {
        frame_index_ = 0;
        octree_depth_ = 0;
        max_breadth_depth_ = 0;
        num_points_ = 0;
        orig_num_points_ = 0;
        breadth_bytes_.clear();
        depth_list_.clear();
        color_list_.clear();

        point_indices_.clear();
        compressed_depth_bytes_.clear();
        leaf_indices_.clear();
        child_indices_.clear();

        // With ChangeDetector octree
        // octree_.switchBuffers();

        // Normal octree
        octree_.deleteTree();
        pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
        cloud_ = temp_cloud;
        initialize();
    }

  protected:
    void initialize();
    void transformPointCloud(pcl::PointCloud<PointType>::Ptr cloud,
                             pcl::PointCloud<PointType>::Ptr out_cloud, float scale, bool isFlip,
                             float x, float y, float z);

    // functions to generate auxiliary information
    void generateLeafNodeIndices();
    void generateChildNodeIndices();

    // functions for saving compressed files
    void generateHeader(char type);
    void reorderDepthColor();
    void reorderDepthColorShort();

    // functions for efficient octree probing
    Eigen::Vector3f getChildCenter(Eigen::Vector3f parentCenter, float sidelen, int idx);
    void getNodesInOctreeContainer(pcl::octree::OctreeNode *currentNode, std::vector<int> &indices);

    // functions to compress breadth bytes
    void compressBreadthBytes();
    void compressBreadthBytesShort();
    // functions to compress depth bytes
    void compressDepthBytes(int is_user_adaptive);
    // variables

    int frame_index_ = 0;
    pcl::PointCloud<PointType>::Ptr cloud_;
    // pcl::octree::OctreePointCloud<PointType> octree_;
    pcl::octree::OctreePointCloudChangeDetector<PointType> octree_;
    int octree_depth_ = 0;
    int max_breadth_depth_ = 0;
    int num_points_ = 0;
    int orig_num_points_ = 0;

    Eigen::Vector3f root_center_;
    float root_sidelength_;

    vector<vector<uint8_t>> breadth_bytes_;
    vector<uint8_t> depth_list_;
    vector<Color> color_list_;

    vector<int> point_indices_;

    vector<uint8_t> compressed_depth_bytes_;

    FrameHeader header_;
    FramePayload payload_;

    // For debuggin purpose
    vector<uint8_t> debug_color_list_;
    vector<Eigen::Vector4f> debug_point_list_;

    vector<vector<int>> leaf_indices_;
    vector<vector<int>> child_indices_;

    Resources ress_;
};

struct ScopeTimer {
    std::chrono::time_point<std::chrono::high_resolution_clock> t;
    std::string name_;
    __attribute__((optimize("O0"))) ScopeTimer(std::string const &name)
        : name_(name), t(std::chrono::high_resolution_clock::now()) {};
    __attribute__((optimize("O0"))) ~ScopeTimer() {
        auto stop_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> ms_double = stop_time - t;
        std::cout << name_ << ": " << ms_double.count() << "ms" << std::endl;
    };
};

class GROOTEncoder {
  public:
    GROOTEncoder(std::string const &morton_code_filename, float voxel_size) {
        voxel_size_ = voxel_size;
        readImageCoder(morton_code_filename);
        jpegEncoder_ = new JpegEncoder();
    }
    typedef pcl::PointCloud<pcl::PointXYZRGB> pc_t;
    typedef pc_t::Ptr pc_ptr_t;
    std::vector<uint8_t> encode(pc_ptr_t const &p) {
        {
            ScopeTimer x("reset");
            currentFrame.reset();
        }
        {
            ScopeTimer x("setPointCloud");
            currentFrame.setPointCloud(p);
        }
        {
            ScopeTimer x("generateOctree");
            currentFrame.generateOctree(voxel_size_);
        }
        bool is_user_adaptive = false; // with colors
        bool isShort = false;
        currentFrame.compressPDTree(is_user_adaptive, isShort);
        {
            ScopeTimer x("generatePayload");
            if (is_user_adaptive) {
                currentFrame.generatePayload();
            } else {
                vector<uint8_t> colors = currentFrame.getColorBytes();
                vector<uint8_t> compressed_colors;
                compressColors(colors, compressed_colors, jpegEncoder_);
                currentFrame.generatePayload(compressed_colors);
            }
        }
        return currentFrame.extractPayload();
    }

  protected:
    float voxel_size_;
    void readImageCoder(std::string const &imagecoder) {
        int imgSize = 1024;
        image_coder_order_ = new int[imgSize * imgSize];
        FILE *pFile = fopen(imagecoder.c_str(), "rb");
        std::size_t res = fread(&image_coder_order_[0], sizeof(int), imgSize * imgSize, pFile);
        if (res == 0)
            throw std::runtime_error("Failed to read ImageCoder");
        fclose(pFile);
    }

    int *image_coder_order_;

    // prepare jpeg encoder
    JpegEncoder *jpegEncoder_;

    void compressColors(vector<uint8_t> orig_colors, vector<uint8_t> &compressed_colors,
                        JpegEncoder *jpegEncoder_) {
        int color_size = orig_colors.size();
        int image_width = 1024;
        int image_height = 1024;

        if (orig_colors.size() / 3 < 1024 * 512) {
            image_height = 512;
        } else if (orig_colors.size() / 3 >= 1024 * 1024) {
            image_width = 2048;
        }

        std::vector<uint8_t> reordered_color(image_width * image_height * 3, 255);
        for (int i = 0; i < image_width * image_height; i++) {
            // for GROOT use this
            int idx = image_coder_order_[i];
            // for SERIAL use this
            // int idx = i;
            if (idx < orig_colors.size() / 3) {
                reordered_color[3 * i] = orig_colors[3 * idx];
                reordered_color[3 * i + 1] = orig_colors[3 * idx + 1];
                reordered_color[3 * i + 2] = orig_colors[3 * idx + 2];
            }
        }

        // for GROOT
        jpegEncoder_->encode(reordered_color, compressed_colors, image_width, image_height);
    }

    Frame currentFrame;
};