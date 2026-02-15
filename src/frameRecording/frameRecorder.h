#pragma once
#include <string>
#include <vector>

class FrameRecorder {
public:
    FrameRecorder();
    ~FrameRecorder();

    void startRecording();
    void stopRecording();
    void saveFrame(int width, int height);

    bool isRecording() const;
    int getFramesRecorded() const;

private:
    bool recording;
    int frameRecorded;
    const int MAX_FRAMES_TO_RECORD = 250;

    void createFramesDirectoryIfNeeded();
    void saveFrameToDisk(int width, int height, const std::vector<unsigned char>& pixels);
};
