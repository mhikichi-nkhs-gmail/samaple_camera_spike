// StreamCaptureRGB.cpp
#include "StreamCaptureRGB.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>


#include <iostream>
#include <cstring>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/select.h>

bool StreamCaptureRGB::initialize(const std::string &device, int width, int height ) {
    printf("camera initialize %d,%d\n",width,height);
        width_ = width;
        height_ = height;

        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            perror("open");
            return false;
        }

        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;

        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            perror("VIDIOC_S_FMT");
            return false;
        }
        // --- FPS 設定追加 ---
        struct v4l2_streamparm parm = {};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (ioctl(fd_, VIDIOC_G_PARM, &parm) < 0) {
            perror("VIDIOC_G_PARM");
        } else {
            parm.parm.capture.timeperframe.numerator = 1;
            parm.parm.capture.timeperframe.denominator = 30;  // ★ ここでFPS設定

            if (ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
                perror("VIDIOC_S_PARM");
            } else {
                std::cout << "Requested FPS: "
                        << parm.parm.capture.timeperframe.denominator << " / "
                        << parm.parm.capture.timeperframe.numerator << std::endl;
            }
        }

        // FPS設定後に即座に確認
        if (ioctl(fd_, VIDIOC_G_PARM, &parm) == 0) {
            std::cout << "Actual FPS: "
                    << parm.parm.capture.timeperframe.denominator << " / "
                    << parm.parm.capture.timeperframe.numerator << std::endl;
        }

        struct v4l2_requestbuffers req = {};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            perror("VIDIOC_REQBUFS");
            return false;
        }

        buffers_.resize(req.count);
        for (size_t i = 0; i < req.count; ++i) {
            struct v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("VIDIOC_QUERYBUF");
                return false;
            }

            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED) {
                perror("mmap");
                return false;
            }

            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
                perror("VIDIOC_QBUF");
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            perror("VIDIOC_STREAMON");
            return false;
        }

        return true;
}

cv::Mat StreamCaptureRGB::captureFrame(double &fpsOut) {

        static auto lastTime = std::chrono::steady_clock::now();

        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            perror("VIDIOC_DQBUF");
            return {};
        }

        uchar* data = static_cast<uchar*>(buffers_[buf.index].start);
        std::vector<uchar> jpeg_data(data, data + buf.bytesused);
        cv::Mat img= cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
        }

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        lastTime = now;
        fpsOut = elapsed > 0.0 ? 1000.0 / elapsed : 0.0;

        return img;
}

bool StreamCaptureRGB::captureFrameSync(cv::Mat &output) {
    double fpsOut;
    cv::Mat frame = captureFrame(fpsOut);
    if (frame.empty()) return false;
    output = frame.clone();
    return true;
}

void StreamCaptureRGB::shutdown() {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);

    for (auto &buf : buffers_) {
        munmap(buf.start, buf.length);
    }

    if (fd_ >= 0) close(fd_);
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
    int minShortSide=20;

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
            if (shortSide < minShortSide) {
                continue;  // 短辺が小さすぎ → 除外
            }

            if (i < 2 && referenceShortSide > 0 &&
                shortSide > referenceShortSide * shortSideThresholdRatio) {
                continue;
            }

            // 下段の場合、中心から画像幅の50%以内のみ許可
            if (i == 2) {
                float xMin = centerX - width * 0.25f;
                float xMax = centerX + width * 0.25f;
                if (rect.center.x < xMin || rect.center.x > xMax) continue;
            }
            
            float score = 0.0f;

            if (i < 2 && detectedRects[i + 1].size.width > 0 && detectedRects[i + 1].size.height > 0) {
                float dx = rect.center.x - detectedRects[i + 1].center.x;
                score = std::abs(dx);  // 連続性重視
            } else if (i == 2) {
                // 下段：画像の縦方向に対して長い矩形を優先
                float verticalLength = std::abs(rect.size.height * std::cos(rect.angle * CV_PI / 180.0f)) +
                                       std::abs(rect.size.width * std::sin(rect.angle * CV_PI / 180.0f));
                float centerDist = std::abs(rect.center.x - centerX);
                score = (1000.0f / (verticalLength + 1e-2)) + centerDist * 0.1f;
            } else {
                float length = std::max(rect.size.width, rect.size.height);
                float centerDist = std::abs(rect.center.x - centerX);
                score = (1000.0f / (length + 1e-2)) + centerDist * 0.1f;
            }

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

