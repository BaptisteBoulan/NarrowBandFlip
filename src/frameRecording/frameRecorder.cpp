// FrameRecorder.cpp
#include "FrameRecorder.h"
#include <GLFW/glfw3.h>
#include <vector>
#include <iostream>
#include <filesystem>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

FrameRecorder::FrameRecorder()
    : recording(false), frameRecorded(0) {}

FrameRecorder::~FrameRecorder() {}

void FrameRecorder::startRecording() {
    recording = true;
    frameRecorded = 0;
    createFramesDirectoryIfNeeded();
    std::cout << "Started recording frames..." << std::endl;
}

void FrameRecorder::stopRecording() {
    recording = false;
    std::cout << "Stopped recording frames." << std::endl;
}

void FrameRecorder::saveFrame(int width, int height) {
    if (!recording || frameRecorded >= MAX_FRAMES_TO_RECORD) {
        return;
    }

    std::vector<unsigned char> pixels(width * height * 3);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // Flip the image vertically
    for (int y = 0; y < height / 2; y++) {
        for (int x = 0; x < width; x++) {
            std::swap(pixels[(y * width + x) * 3], pixels[((height - 1 - y) * width + x) * 3]);
            std::swap(pixels[(y * width + x) * 3 + 1], pixels[((height - 1 - y) * width + x) * 3 + 1]);
            std::swap(pixels[(y * width + x) * 3 + 2], pixels[((height - 1 - y) * width + x) * 3 + 2]);
        }
    }

    saveFrameToDisk(width, height, pixels);
    frameRecorded++;
}

void FrameRecorder::createFramesDirectoryIfNeeded() {
    if (!std::filesystem::exists("frames")) {
        std::filesystem::create_directory("frames");
    }
}

void FrameRecorder::saveFrameToDisk(int width, int height, const std::vector<unsigned char>& pixels) {
    char filename[256];
    snprintf(filename, sizeof(filename), "frames/frame_%04d.png", frameRecorded);
    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
    std::cout << "Saved frame: " << filename << std::endl;
}

bool FrameRecorder::isRecording() const {
    return recording;
}

int FrameRecorder::getFramesRecorded() const {
    return frameRecorded;
}
