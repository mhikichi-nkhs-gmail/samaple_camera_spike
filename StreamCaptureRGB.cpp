// StreamCaptureRGB.cpp
#include "StreamCaptureRGB.h"

#include <iostream>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/select.h>

using namespace libcamera;

bool StreamCaptureRGB::initialize(int width, int height) {
    printf("camera initialize %d,%d\n",width,height);
    cm_ = std::make_unique<CameraManager>();

    cm_->start();

    if (cm_->cameras().empty()) {
        std::cerr << "No camera found." << std::endl;
        return false;
    }

    camera_ = cm_->cameras()[0];
    if (camera_->acquire()) {
        std::cerr << "Failed to acquire camera." << std::endl;
        return false;
    }

    config_ = camera_->generateConfiguration({ StreamRole::Viewfinder });
    config_->at(0).pixelFormat = formats::RGB888;
    config_->at(0).size = { width, height };

    config_->validate();
    if (camera_->configure(config_.get()) < 0) {
        std::cerr << "Failed to configure camera." << std::endl;
        return false;
    }

    std::cout << "Configured size: "
          << config_->at(0).size.width << "x"
          << config_->at(0).size.height << std::endl;


    stream_ = config_->at(0).stream();
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    allocator_->allocate(stream_);

    camera_->requestCompleted.connect(this, &StreamCaptureRGB::onRequestComplete);
    if (camera_->start() < 0) {
        std::cerr << "Failed to start camera." << std::endl;
        return false;
    }



    printf("init fin\n");

    return true;
}

cv::Mat StreamCaptureRGB::captureFrame() {

    std::lock_guard<std::mutex> captureLock(captureMutex_);

    FrameBuffer *buffer = allocator_->buffers(stream_)[bufferIndex_++ % allocator_->buffers(stream_).size()].get();
    std::unique_ptr<Request> request = camera_->createRequest();


    ControlList controls(camera_->controls());
    std::int64_t frameDuration = 33333; // 30fps
    std::array<std::int64_t, 2> durationLimits = { frameDuration, frameDuration };
    controls.set(libcamera::controls::FrameDurationLimits,
                 libcamera::Span<const std::int64_t, 2>(durationLimits));
    request->controls() = controls;

    request->addBuffer(stream_, buffer);
    {
        std::lock_guard<std::mutex> lock2(mutex_);
        done_ = false;
    }
    camera_->queueRequest(request.release());

    std::unique_lock<std::mutex> lock2(mutex_);
    cv_.wait(lock2, [&] { return done_; });

//printf("captureFrame return \n");
    return lastFrame_.clone();
}

bool StreamCaptureRGB::captureFrameSync(cv::Mat &output) {
    cv::Mat frame = captureFrame();
    if (frame.empty()) return false;
    output = frame.clone();
    return true;
}

void StreamCaptureRGB::shutdown() {
    if (camera_) {
        camera_->stop();
        camera_->requestCompleted.disconnect(this, &StreamCaptureRGB::onRequestComplete);
        allocator_.reset();
        camera_->release();
    }
    if (cm_) cm_->stop();
}

void StreamCaptureRGB::onRequestComplete(Request *r) {
   // printf("onRequestComplete\n");
    const FrameBuffer::Plane &plane = r->buffers().begin()->second->planes()[0];
    void *mem = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    int stride = plane.length / config_->at(0).size.height;
    if (mem == MAP_FAILED) {
        std::cerr << "mmap failed\n";
        return;
    }

    const StreamConfiguration &cfg = config_->at(0);
    lastFrame_ = cv::Mat(cfg.size.height, cfg.size.width, CV_8UC3, mem,stride).clone();
    munmap(mem, plane.length);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        done_ = true;
    }
    cv_.notify_one();
  //  printf("onRequestComplete finish\n");
}

// cropとresizeを同時に行う
void StreamCaptureRGB::cropImage(cv::Mat src,int x, int y,int width,int height, cv::Mat &dst, double rate) {

    cv::Rect roi(x, y, width, height);
    cv::Mat cropped = src(roi);  // クロップ（これはコピーではないので高速）

    int sizex = width*rate;
    int sizey = height*rate;

    // リサイズ（間引き）
    cv::resize(cropped, dst, cv::Size(sizex, sizey), 0, 0, cv::INTER_NEAREST);  
}

void StreamCaptureRGB::overlayBinaryMask(
    const cv::Mat& baseImage,        // 元画像（グレースケールまたはカラー）
    const cv::Mat& binaryMask,       // 二値画像（0 or 255）
    cv::Mat& outputImage,            // 出力先
    double alpha,              // マスクの透明度
    const cv::Scalar& maskColor 
) {
    CV_Assert(binaryMask.type() == CV_8UC1);
    CV_Assert(baseImage.size() == binaryMask.size());

    // カラー化（必要なら）
    cv::Mat baseColor;
    if (baseImage.channels() == 1)
        cv::cvtColor(baseImage, baseColor, cv::COLOR_GRAY2BGR);
    else
        baseColor = baseImage.clone();

    // マスクをカラー画像に変換
    cv::Mat maskColorImage(baseImage.size(), CV_8UC3, maskColor);
    cv::Mat maskRegion;
    maskColorImage.copyTo(maskRegion, binaryMask);  // 二値画像が255の部分だけ色をつける

    // 半透明合成（ベース + alpha × マスク）
    //outputImage = baseColor.clone();
    cv::addWeighted(maskRegion, alpha, baseColor, 1.0 - alpha, 0.0, outputImage);
}


// 二値画像を元に回転矩形を検出し、出力画像に描画する関数
void StreamCaptureRGB::detectRotatedRectangles(const cv::Mat& binaryImage, cv::Mat& outputImage,    EdgeMode edgeMode) {
    CV_Assert(binaryImage.type() == CV_8UC1);
    CV_Assert(outputImage.type() == CV_8UC3);
    CV_Assert(binaryImage.size() == outputImage.size());

    int maxShortSide=100;

    //detectedRects.clear();
    detectedRects.resize(3);  // 上・中・下

    int height = binaryImage.rows;
    int width = binaryImage.cols;
    int roiHeight = height / 3;
    int centerX = width / 2;

    float referenceShortSide = -1.0f;
    cv::Scalar maskcol = cv::Scalar(0, 0, 255);
    overlayBinaryMask(outputImage,binaryImage,outputImage,0.5,maskcol);

    for (int i = 2; i >= 0; --i) {
        int yStart = i * roiHeight;
        int yEnd = (i == 2) ? height : yStart + roiHeight;
        cv::Rect roi(0, yStart, width, yEnd - yStart);
        cv::Mat roiBinary = binaryImage(roi);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roiBinary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        float bestScore = std::numeric_limits<float>::max();
        cv::RotatedRect bestRect;
        bool found = false;

        for (const auto& contour : contours) {
            if (contour.empty()) continue;
            cv::RotatedRect rect = cv::minAreaRect(contour);
            rect.center.y += yStart;

            float shortSide = std::min(rect.size.width, rect.size.height);
            if (shortSide > maxShortSide) {
                continue;  // 短辺が太すぎ → 除外
            }
            
            if (i < 2 && referenceShortSide > 0 &&
                shortSide > referenceShortSide * shortSideThresholdRatio) {
                continue;
            }

            float length = std::max(rect.size.width, rect.size.height);
            float centerDist = std::abs(rect.center.x - centerX);
            float score = (1000.0f / (length + 1e-2)) + centerDist * 0.1f;

            if (!found || score < bestScore) {
                bestRect = rect;
                bestScore = score;
                found = true;
            }
        }

        if (found) {
            detectedRects[i] = bestRect;
            if (i == 2) {
                referenceShortSide = std::min(bestRect.size.width, bestRect.size.height);
            }

            // ====== 矩形枠を描画 ======
            cv::Point2f pts[4];
            bestRect.points(pts);
            cv::Scalar boxColor = (i == 2) ? cv::Scalar(0, 0, 255) :
                                 (i == 1) ? cv::Scalar(0, 255, 0) :
                                            cv::Scalar(255, 0, 0);

            for (int j = 0; j < 4; ++j)
                cv::line(outputImage, pts[j], pts[(j + 1) % 4], boxColor, 1);

            // ====== 長辺矢印の描画 ======
            struct Edge {
                cv::Point2f p1, p2;
                float lenSq;
            };
            std::vector<Edge> edges;
            for (int j = 0; j < 4; ++j) {
                cv::Point2f p1 = pts[j];
                cv::Point2f p2 = pts[(j + 1) % 4];
                float dx = p2.x - p1.x;
                float dy = p2.y - p1.y;
                float lenSq = dx * dx + dy * dy;
                edges.push_back({p1, p2, lenSq});
            }

            std::sort(edges.begin(), edges.end(),
                      [](const Edge& a, const Edge& b) { return a.lenSq > b.lenSq; });

            Edge* selectedEdge = nullptr;
            for (int k = 0; k < 2; ++k) {
                cv::Point2f mid = (edges[k].p1 + edges[k].p2) * 0.5f;
                float dx = mid.x - bestRect.center.x;
                if ((edgeMode == EDGE_LEFT && dx <= 0) ||
                    (edgeMode == EDGE_RIGHT && dx > 0)) {
                    selectedEdge = &edges[k];
                    break;
                }
            }

            if (selectedEdge) {
                cv::Point2f start = selectedEdge->p1;
                cv::Point2f end   = selectedEdge->p2;
                if (start.y < end.y) std::swap(start, end);
                cv::arrowedLine(outputImage, start, end, boxColor, 2, cv::LINE_AA, 0, 0.2);
            }
        }
    }
}

// ノイズ除去前の処理（モルフォロジー開処理＋小面積除去）
cv::Mat StreamCaptureRGB::cleanBinaryImage(const cv::Mat& binary) {
    cv::Mat cleaned;

    // 1. モルフォロジー開処理（小さな白点除去）
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, cleaned, cv::MORPH_OPEN, kernel);

    // 2. 小さい輪郭を除去
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cleaned, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 空のマスクを作成
    cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8UC1);
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > 100.0) { // 面積が100より大きいものだけ採用（調整可能）
            cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, 255, cv::FILLED);
        }
    }

    return mask;
}

void StreamCaptureRGB::drawVectorArrows(
    cv::Mat& image,
    EdgeMode edgeMode
) {
    float referenceShortSide = -1.0f;
    float shortSideThresholdRatio = 1.5f;

    for (int i = 2; i >= 0; --i) {
        const auto& rect = detectedRects[i];
        if (rect.size.width <= 0 || rect.size.height <= 0) continue;

        float shortSide = std::min(rect.size.width, rect.size.height);

        if (i == 2) {
            referenceShortSide = shortSide;  // 最下段 → 必ず描画対象
        } else {
            printf("referenceShortSide %f,%f\n",referenceShortSide,shortSide);
            if (referenceShortSide > 0 &&
                shortSide > referenceShortSide * shortSideThresholdRatio) {
                continue;  // 無効 → 描画スキップ
            }
        }

        // ========== 長辺候補選出 ==========

        cv::Point2f pts[4];
        rect.points(pts);

        struct Edge {
            cv::Point2f p1, p2;
            float lenSq;
        };
        std::vector<Edge> edges;
        for (int j = 0; j < 4; ++j) {
            cv::Point2f p1 = pts[j];
            cv::Point2f p2 = pts[(j + 1) % 4];
            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float lenSq = dx * dx + dy * dy;
            edges.push_back({p1, p2, lenSq});
        }

        std::sort(edges.begin(), edges.end(),
                  [](const Edge& a, const Edge& b) { return a.lenSq > b.lenSq; });

        Edge* selectedEdge = nullptr;

        for (int k = 0; k < 2; ++k) {
            Edge& e = edges[k];
            cv::Point2f mid = (e.p1 + e.p2) * 0.5f;
            float dx = mid.x - rect.center.x;

            if ((edgeMode == EDGE_LEFT  && dx <= 0) ||
                (edgeMode == EDGE_RIGHT && dx >  0)) {
                selectedEdge = &e;
                break;
            }
        }

        if (selectedEdge) {
            cv::Point2f start = selectedEdge->p1;
            cv::Point2f end   = selectedEdge->p2;
            if (start.y < end.y) std::swap(start, end);  // 下→上に統一

            cv::Scalar color = (i == 2) ? cv::Scalar(0, 0, 255) :
                               (i == 1) ? cv::Scalar(0, 255, 0) :
                                          cv::Scalar(255, 0, 0);

            cv::arrowedLine(image, start, end, color, 2, cv::LINE_AA, 0, 0.2);

            // デバッグ：始点と方向確認（必要なら）
            // cv::circle(image, start, 3, color, -1);
        }
    }
}

// 長辺のベクトルが左エッジかどうか判定
bool StreamCaptureRGB::isLongEdgeLeftOfCenter(const cv::RotatedRect& rect, cv::Point2f& outStart, cv::Point2f& outEnd) {
    if (rect.size.width <= 0 || rect.size.height <= 0) return false;

    cv::Point2f pts[4];
    rect.points(pts);

    float maxLenSq = -1.0f;
    cv::Point2f start, end;

    // 最長辺を見つける
    for (int i = 0; i < 4; ++i) {
        cv::Point2f p1 = pts[i];
        cv::Point2f p2 = pts[(i + 1) % 4];
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float lenSq = dx * dx + dy * dy;

        if (lenSq > maxLenSq) {
            maxLenSq = lenSq;
            start = p1;
            end = p2;
        }
    }

    // 中点と矩形中心を比較して左側かどうか判定
    cv::Point2f mid = (start + end) * 0.5f;
    bool isLeft = (mid.x < rect.center.x);

    // 出力用にstart→endを返す（下から上向きに揃える場合はここで補正も可能）
    outStart = start;
    outEnd = end;

    return isLeft;
}

void StreamCaptureRGB::saveEdgeVectorsToStruct(
    EdgeMode edgeMode,
    int imageWidth, int imageHeight
) {

    for (int i = 0; i < 3; ++i) {
        LineVector vec = {0, 0, 0, 0};

        const auto& rect = detectedRects[i];
        if (rect.size.width <= 0 || rect.size.height <= 0) {
            detectedResult.layers[i] = vec;
            continue;
        }

        cv::Point2f pts[4];
        rect.points(pts);

        struct Edge {
            cv::Point2f p1, p2;
            float lenSq;
        };
        std::vector<Edge> edges;
        for (int j = 0; j < 4; ++j) {
            cv::Point2f p1 = pts[j];
            cv::Point2f p2 = pts[(j + 1) % 4];
            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float lenSq = dx * dx + dy * dy;
            edges.push_back({p1, p2, lenSq});
        }

        std::sort(edges.begin(), edges.end(),
                  [](const Edge& a, const Edge& b) { return a.lenSq > b.lenSq; });

        Edge* selectedEdge = nullptr;
        for (int k = 0; k < 2; ++k) {
            cv::Point2f mid = (edges[k].p1 + edges[k].p2) * 0.5f;
            float dx = mid.x - rect.center.x;

            if ((edgeMode == EDGE_LEFT && dx <= 0) ||
                (edgeMode == EDGE_RIGHT && dx > 0)) {
                selectedEdge = &edges[k];
                break;
            }
        }

        if (selectedEdge) {
            cv::Point2f p1 = selectedEdge->p1;
            cv::Point2f p2 = selectedEdge->p2;
            if (p1.y < p2.y) std::swap(p1, p2);

            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float norm = std::sqrt(dx * dx + dy * dy);

            if (norm > 1e-5f) {
                // === OpenCV座標 → 数学座標へ変換 ===
                float start_x_math = p1.x - (imageWidth / 2.0f);
                float start_y_math = imageHeight - p1.y;

                float dir_x_math = dx / norm;
                float dir_y_math = -dy / norm;  // y反転

                vec.start_x = start_x_math;
                vec.start_y = start_y_math;
                vec.dir_x = dir_x_math;
                vec.dir_y = dir_y_math;

               // printf("%d,%d  %f,%f -> %f,%f\n",imageWidth,imageHeight,p1.x,p1.y,start_x_math,start_y_math);
            }
        }

        detectedResult.layers[i] = vec;
    }   
 }

