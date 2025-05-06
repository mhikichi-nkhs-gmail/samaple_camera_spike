#ifndef STREAM_CAPTURE_RGB_H
#define STREAM_CAPTURE_RGB_H

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/request.h>
#include <libcamera/framebuffer_allocator.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cstring>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include "vectorStruct.h"

class StreamCaptureRGB {
public:
    enum EdgeMode {
        EDGE_LEFT,
        EDGE_RIGHT
    };

    bool initialize(int width = 3280, int height = 2464);
    cv::Mat captureFrame();                    // 非同期的にキャプチャ（タイムアウトなしで待つ）
    bool captureFrameSync(cv::Mat &output);    // 同期キャプチャ（RTOSから呼び出しやすい）
    void shutdown();
    bool camera_get_image(cv::Mat &output);
    void cropImage(cv::Mat src,int x, int y,int width,int height, cv::Mat &dst, double rate);
    void detectRotatedRectangles(const cv::Mat& binaryImage, cv::Mat& outputImage,EdgeMode edgeMode) ;
    cv::Mat cleanBinaryImage(const cv::Mat& binary);
    void drawVectorArrows(
        cv::Mat& image,
        EdgeMode edgeMode
    );
    bool isLongEdgeLeftOfCenter(const cv::RotatedRect& rect, cv::Point2f& outStart, cv::Point2f& outEnd);
    void saveEdgeVectorsToStruct(
        EdgeMode edgeMode, int w, int h);
    void overlayBinaryMask(
        const cv::Mat& baseImage,        // 元画像（グレースケールまたはカラー）
        const cv::Mat& binaryMask,       // 二値画像（0 or 255）
        cv::Mat& outputImage,            // 出力先
        double alpha,              // マスクの透明度
        const cv::Scalar& maskColor
    ) ;
    cv::Mat shared_frame;
    LineDetectionResult detectedResult;

private:
    void onRequestComplete(libcamera::Request *r);

    std::unique_ptr<libcamera::CameraManager> cm_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    libcamera::Stream *stream_{};
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;

    std::mutex mutex_;
    std::condition_variable cv_;
    bool done_ = false;

    std::mutex captureMutex_;
    int bufferIndex_ = 0;

    cv::Mat lastFrame_;
    int frameCount_ = 0;

    std::vector<cv::RotatedRect> detectedRects; // 下・中・上
    float shortSideThresholdRatio = 1.5f;

};

#endif // STREAM_CAPTURE_RGB_H
