/**
 * capture_console.cpp
 * Place in QT/GC3DExample/ directory, reuses the SDK include/lib paths of that project
 *
 * Function: Console program (no GUI), connects to GC3D camera and captures depth map and texture map
 *
 * Compilation (choose one):
 *   1. Manual command line:
 *      cl /EHsc /O2 capture_console.cpp ^
 *         /I..\..\..\..\04_GCI_SDK\C++SDK\include ^
 *         /I. ^
 *         /link ..\..\..\..\04_GCI_SDK\C++SDK\lib64\GC3D.lib ^
 *               ..\..\..\..\04_GCI_SDK\C++SDK\lib64\GC3DAlgorithm.lib
 *
 *   2. Or add to .pro file: add this file to GC3DExample.pro
 *
 * Output:
 *   depth.raw      - float32 real depth values (unit: mm)
 *   texture.png    - 8-bit grayscale texture (PNG format)
 *   depth_viz.png  - 8-bit depth visualization (PNG format)
 */

#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <string>
#include <sys/stat.h>

#include <gc3d.h>
#include <gc3dAlgorithm.h>

// ============================================================
// 使用 stb_image_write.h 保存PNG
// 下载: https://raw.githubusercontent.com/nothings/stb/master/stb_image_write.h
// ============================================================
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace gc3d;

// ============================================================
//  Utilities: save images
// ============================================================

// Output directory
const std::string OUTPUT_DIR = "C:\\Users\\byd\\Desktop\\PVC_folder\\SimGrasp_PVC\\c++\\saved\\";

/** Check if directory exists, if not create it */
static bool ensureDirectory(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) == 0) {
        return (info.st_mode & S_IFDIR) != 0;
    }
    // Directory doesn't exist, try to create it
    std::string cmd = "mkdir \"" + path + "\" 2>nul";
    return system(cmd.c_str()) == 0;
}

/** Build full path from filename */
static std::string getFullPath(const std::string& filename) {
    return OUTPUT_DIR + filename;
}

/** Save PNG format grayscale image (uint8) */
static bool savePNG(const std::string& path, const unsigned char* data, int w, int h) {
    // stbi_write_png(文件名, 宽, 高, 通道数, 数据, 行步长)
    // 灰度图: 1通道, 行步长 = w
    int result = stbi_write_png(path.c_str(), w, h, 1, data, w);
    if (result == 0) {
        std::cerr << "  Failed to save PNG: " << path << std::endl;
        return false;
    }
    return true;
}

/** Save float32 raw depth (can be read with NumPy np.fromfile) */
static bool saveRawFloat(const std::string& path, const float* data, int n) {
    std::ofstream f(path, std::ios::binary);
    if (!f) { std::cerr << "  Write failed: " << path << std::endl; return false; }
    f.write(reinterpret_cast<const char*>(data), n * sizeof(float));
    return true;
}

/** Depth values (float mm) -> 0-255 grayscale (invalid points -> 0) */
static void depthToGray(const float* depth, unsigned char* gray, int n) {
    float vmin = 1e30f, vmax = -1e30f;
    int validCount = 0;
    
    // Find valid depth range
    for (int i = 0; i < n; ++i) {
        float v = depth[i];
        if (v > 0.0f && !std::isinf(v) && !std::isnan(v)) {
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
            validCount++;
        }
    }
    
    if (vmax <= vmin || validCount == 0) { 
        vmin = 0.0f; 
        vmax = 1.0f; 
    }
    
    std::cout << "  Depth range: [" << vmin << ", " << vmax << "] mm (valid: " << validCount << " points)\n";
    
    float s = 255.0f / (vmax - vmin);
    for (int i = 0; i < n; ++i) {
        float v = depth[i];
        if (v <= 0.0f || std::isinf(v) || std::isnan(v)) { 
            gray[i] = 0; 
            continue; 
        }
        float c = (v - vmin) * s;
        gray[i] = (unsigned char)(c < 0 ? 0 : (c > 255 ? 255 : c));
    }
}

// ============================================================
//  Main workflow
// ============================================================

int main() {
    std::cout << "======================================\n";
    std::cout << " GC3D Camera Capture (Console)\n";
    std::cout << " Output: PNG + RAW\n";
    std::cout << "======================================\n\n";

    // Ensure output directory exists
    std::cout << "[0] Checking output directory...\n";
    if (!ensureDirectory(OUTPUT_DIR)) {
        std::cerr << "  WARNING: Could not create directory: " << OUTPUT_DIR << std::endl;
        std::cerr << "  Files will be saved to current directory instead.\n";
    } else {
        std::cout << "  Output directory: " << OUTPUT_DIR << std::endl;
    }

    // ---- 1. Scan devices ----
    std::cout << "\n[1] Scanning devices...\n";
    DeviceInformation* infos = nullptr;
    size_t devNum = 0;
    if (GC3D_SUCCESS != initialDevice(infos, devNum) || devNum == 0) {
        std::cerr << "  ERROR: No camera detected\n";
        system("pause"); return -1;
    }
    std::cout << "  Found " << devNum << " device(s):\n";
    for (size_t i = 0; i < devNum; ++i)
        std::cout << "    #" << i << "  " << infos[i].serialNum
                  << "  (" << infos[i].productType << ")\n";

    // ---- 2. Open camera ----
    std::cout << "\n[2] Opening camera...\n";
    GC3DDevice dev;
    if (GC3D_SUCCESS != dev.openDeviceByIndex(0)) {
        std::cerr << "  ERROR: Failed to open\n";
        unInitialDevice(); system("pause"); return -1;
    }
    std::cout << "  Opened #0\n";

    // ---- 3. Set parameters ----
    std::cout << "\n[3] Setting parameters...\n";
    GC3DCameraParameters p; p.exposureTime = 2000;
    dev.setCameraParameters(p);
    dev.setDenoiseParameters(3, 60.0f, 5.0f, 5.0f);
    dev.setHeightRange(100.0f, 3000.0f);
    dev.setNeedGridData(false);
    std::cout << "  Exposure=2000us  Height=[100,3000]mm\n";

    // ---- 4. Capture ----
    std::cout << "\n[4] Capturing...\n";
    if (GC3D_SUCCESS != dev.snapShot3D()) {
        std::cerr << "  ERROR: Capture failed\n";
        dev.closeDevice(); unInitialDevice(); system("pause"); return -1;
    }

    GC3DMetaData meta;
    if (GC3D_SUCCESS != dev.getGC3DMetaData(meta)) {
        std::cerr << "  ERROR: Failed to get data\n";
        dev.closeDevice(); unInitialDevice(); system("pause"); return -1;
    }

    int W = meta.imgW, H = meta.imgH, N = W * H;
    std::cout << "  Image: " << W << " x " << H
              << "  Valid points: " << meta.validPointsNum << "\n";

    // ---- 5. Write files (PNG format) ----
    std::cout << "\n[5] Writing files...\n";

    // 5.1 Save depth raw data (float32)
    if (meta.z) {
        std::string depthPath = getFullPath("depth.raw");
        if (saveRawFloat(depthPath, meta.z, N)) {
            std::cout << "  Saved: " << depthPath << " (" << N * 4 << " bytes)\n";
        }
    } else {
        std::cout << "  WARN: z is null\n";
    }

    // 5.2 Save texture as PNG
    if (meta.textureData) {
        std::string texPath = getFullPath("texture.png");
        if (savePNG(texPath, meta.textureData, W, H)) {
            std::cout << "  Saved: " << texPath << " (PNG)\n";
        }
    } else {
        std::cout << "  WARN: textureData is null\n";
    }

    // 5.3 Save depth visualization as PNG
    if (meta.z) {
        unsigned char* viz = new unsigned char[N];
        depthToGray(meta.z, viz, N);
        std::string vizPath = getFullPath("depth_viz.png");
        if (savePNG(vizPath, viz, W, H)) {
            std::cout << "  Saved: " << vizPath << " (PNG)\n";
        }
        delete[] viz;
    }

    // ---- done ----
    dev.closeDevice();
    unInitialDevice();

    std::cout << "\n======================================\n";
    std::cout << " Done! Output files saved to:\n";
    std::cout << "  " << OUTPUT_DIR << std::endl;
    std::cout << "   depth.raw      float32 depth (" << N * 4 << " bytes)\n";
    std::cout << "   texture.png    texture image (" << W << "x" << H << ")\n";
    std::cout << "   depth_viz.png  depth visualization (" << W << "x" << H << ")\n";
    std::cout << "======================================\n";

    system("pause");
    return 0;
}