#pragma once

#include "turbojpeg.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>
using namespace std;

class JpegDecoder {
  public:
    JpegDecoder();
    ~JpegDecoder() { printf("[JPEG] decoder deleted\n"); };

    void decode(vector<uint8_t> compressed_bytes, vector<uint8_t> &decoded_bytes);

  private:
    tjhandle handle_;
    int jpegQual_;
    int nbands_;
    int flags_;
    int pixelFormat_;
    int jpegSubsamp_;
};
