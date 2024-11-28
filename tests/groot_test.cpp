#include "groot/Frame.hpp"
#include "groot/Decoder.hpp"


void groot_test(){
    std::string morton_code = getenv("MORTON_CODE");

    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pointcloud->resize(5);
    
    pointcloud->points.at(0).x = 0.;
    pointcloud->points.at(0).y = 0.;
    pointcloud->points.at(0).z = 0.;

    pointcloud->points.at(1).x = 1.;
    pointcloud->points.at(1).y = 0.;
    pointcloud->points.at(1).z = 0.;
    
    pointcloud->points.at(2).x = 0.;
    pointcloud->points.at(2).y = 1.;
    pointcloud->points.at(2).z = 0.;

    pointcloud->points.at(3).x = 0.;
    pointcloud->points.at(3).y = 0.;
    pointcloud->points.at(3).z = 1.;

    pointcloud->points.at(4).x = 1.;
    pointcloud->points.at(4).y = 1.;
    pointcloud->points.at(4).z = 1.;

    GROOTEncoder enc(morton_code, 0.002);
    auto payload = enc.encode(pointcloud);

    Decoder decoder;
    decoder.decodePayload(payload);
    auto pointcloud_out = decoder.generatePointCloud();
    for(auto const & p : pointcloud_out->points) {
        std::cout << "XYZRGB "
            << p.x << " " << p.y << " " << p.z << " "
            << p.r << " " << p.g << " " << p.b << std::endl;
    }
}


int main(int argc, char **argv){
    groot_test();
}