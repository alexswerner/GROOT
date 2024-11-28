#include <bitset>
#include <boost/filesystem.hpp>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>

#include "groot/Decoder.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    if(argc!=2) {
        throw std::runtime_error("Pass stream file as first argument");
    }
    std::ifstream infile(argv[1], std::ios_base::in | std::ios_base::binary);
    if(!infile.good()) {
        throw std::runtime_error("Could not open stream file");
    }
    infile.exceptions(std::ifstream::failbit);
    Decoder decoder;
    Manifest manifest;
    decoder.readManifest("unused", manifest);
    std::size_t counter = 0;
    while (true) {
        int totalsize;
        infile.read(reinterpret_cast<char*>(&totalsize), sizeof(int));
        if(infile.bad())
        std::cout << "Got a frame with totalsize " << totalsize << std::endl;
        std::vector<uint8_t> payload(totalsize);
        infile.read(reinterpret_cast<char*>(payload.data()), payload.size());

        decoder.decodePayload(payload);
        std::stringstream filename;
        filename << "out" << counter++ << ".ply";
        auto pc = decoder.generatePointCloud();
        // TODO: pcl::Visualizer
        if(infile.eof())break;
    }
}
