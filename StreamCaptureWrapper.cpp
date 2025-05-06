// StreamCaptureWrapper.cpp
#include "StreamCaptureRGB.h"
#include "stream_capture_c.h"
#include <cstring>

#include <pthread.h>
#include <atomic>
#include <unistd.h>
#include <iostream>
#include <sys/socket.h>   // ソケットAPI（socket, bind, connect など）
#include <netinet/in.h>   // sockaddr_in 構造体と AF_INET 定数
#include <arpa/inet.h>    // inet_pton() や htons() など

#include <signal.h>
#include <kernel.h>

#include <sys/time.h>

static std::atomic<int> last_signal{0};

static StreamCaptureRGB camera;
static pthread_t capture_thread;
static std::atomic<bool> running{false};
static unsigned char *shared_buf = nullptr;
static int shared_buf_size = 0;
static int shared_width = 0, shared_height = 0;


static pthread_mutex_t shared_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t result_mutex = PTHREAD_MUTEX_INITIALIZER;

static pthread_t sender_thread;
static pthread_t result_thread;

extern "C" {
    static void signal_logger(int signum) {
        last_signal.store(signum);  // 直近のシグナル番号を記録
    }
    static void setup_signal_logging() {
        struct sigaction sa;
        sa.sa_handler = signal_logger;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
    
        for (int sig = 1; sig < NSIG; ++sig) {
            if (sig == SIGKILL || sig == SIGSTOP) continue; // これらは捕捉できない
            sigaction(sig, &sa, nullptr);
        }
    }
    
    static void mask_all_signals(sigset_t *old) {
        sigset_t all;
        sigfillset(&all);  // すべてのシグナルをセット
        sigprocmask(SIG_BLOCK, &all, old);  // 既存のマスクは old に保存
    }

    
    static void disable_interrupt(sigset_t *old)
    {
      sigset_t sigset;
      sigemptyset(&sigset);
    
      sigaddset(&sigset,SIGUSR2);
      sigaddset(&sigset,SIGALRM);
      sigaddset(&sigset,SIGPOLL);
      sigprocmask(SIG_BLOCK, &sigset, old);
      return;
    }
    
    static void enable_interrupt(sigset_t *to_set)
    {
      sigprocmask(SIG_SETMASK,to_set,NULL);
    }
    
   
    void *camera_thread_do_proc(void *args) {
        sigset_t oldsig;
        disable_interrupt(&oldsig);
            while (running) {
                // int sig = last_signal.load();
                // if (sig != 0) {
                //     std::cout << "最後に受信したシグナル: " << sig << std::endl;
                // }
                                
            cv::Mat frame;
            if (camera.captureFrameSync(frame)) {
                pthread_mutex_lock(&shared_mutex);
                std::memcpy(shared_buf, frame.data, shared_buf_size);
                shared_width = frame.cols;
                shared_height = frame.rows;
                cv::Mat image(shared_height, shared_width, CV_8UC3, shared_buf);
                if (!image.empty()) {
                    cv::imwrite("shared_frame.jpg", image);
                } else {
                    std::cerr << "Failed to construct cv::Mat from shared buffer.\n";
                }
            
                pthread_mutex_unlock(&shared_mutex);
               // enable_interrupt(&oldsig);
            } else {
                printf("sleep\n");
              //  enable_interrupt(&oldsig);
                usleep(1000);
            }
            // printf("enable_interrupt\n");
            // enable_interrupt(&oldsig);
        }
        printf("finish camera thread\n");
        return nullptr;
    }

struct timespec  sttime,mid1time,mid2time,edtime;

double diffspectime(struct timespec st,struct timespec ed) {
    unsigned int sec;
    int nsec;
    double d_sec;
    sec = ed.tv_sec - st.tv_sec;
    nsec = ed.tv_nsec - st.tv_nsec;

    d_sec = (double)sec
        + (double)nsec / (1000 * 1000 * 1000);

    return d_sec;

}

static int w,h;
bool camera_initialize(int width, int height) {

    pthread_attr_t tattr;
    int ret;
    int newprio = 99;
    sched_param param;

    /* デフォルト属性で初期化する */
    ret = pthread_attr_init (&tattr);

    /* 既存のスケジューリングパラメタを取得する */
    ret = pthread_attr_getschedparam (&tattr, &param);
    printf("now priority %d\n",param.sched_priority);

    /* 優先順位を設定する。それ以外は変更なし */
    param.sched_priority = newprio;

    /* 新しいスケジューリングパラメタを設定する */
    //ret = pthread_attr_setschedparam (&tattr, &param);

    w=width;
    h=height;
    pthread_create(&capture_thread, &tattr, [](void *) -> void * {
        sigset_t oldsig;
        disable_interrupt(&oldsig);
        if (!camera.initialize(w,h)) 
            return nullptr;
        shared_buf_size = w * h * 3;
        shared_buf = new unsigned char[shared_buf_size];
        running = true;
        printf("pthread_create\n");
        cv::Mat frame;
    
            while (running) {
                clock_gettime(CLOCK_REALTIME, &sttime);
                if (camera.captureFrameSync(frame)) {
                    // clock_gettime(CLOCK_REALTIME, &mid1time);
                    // printf("capture time:%f\n", diffspectime(sttime,mid1time));
                    pthread_mutex_lock(&shared_mutex);
                    std::memcpy(shared_buf, frame.data, shared_buf_size);
                    shared_width = frame.cols;
                    shared_height = frame.rows;
                // printf("shared_width,shared_height=%d,%d\n",shared_width,shared_height);
                   // cv::Mat image(shared_height, shared_width, CV_8UC3, shared_buf,shared_width*3);
                    // if (!image.empty()) {
                    //     cv::imwrite("shared_frame.jpg", image);
                    // } else {
                    //     std::cerr << "Failed to construct cv::Mat from shared buffer.\n";
                    // }
                
                    // clock_gettime(CLOCK_REALTIME, &mid2time);
                // printf("copy time:%f\n", diffspectime(mid1time,mid2time));
                    pthread_mutex_unlock(&shared_mutex);
                    // enable_interrupt(&oldsig);
                } 
                clock_gettime(CLOCK_REALTIME, &edtime);
                double etime = diffspectime(sttime,edtime);
                printf("all time:%f fps:%f\n", etime , 1/etime);

            // printf("enable_interrupt\n");
            // enable_interrupt(&oldsig);
        }
        printf("finish camera thread\n");
        return nullptr;
    }, nullptr);
    return true;
}

bool camera_capture(unsigned char *buf, int bufsize, int *w, int *h) {
    pthread_mutex_lock(&shared_mutex);
    if (!shared_buf || shared_width * shared_height * 3 > bufsize) {
        pthread_mutex_unlock(&shared_mutex);
        return false;
    }
    std::memcpy(buf, shared_buf, shared_width * shared_height * 3);
    *w = shared_width;
    *h = shared_height;
    pthread_mutex_unlock(&shared_mutex);
    return true;
}

void camera_shutdown() {
    running = false;
    pthread_join(capture_thread, nullptr);
    delete[] shared_buf;
    shared_buf = nullptr;
    camera.shutdown();
}

/* カメラ映像が保存されたメモリをロックしてからoutputにコピーする*/
bool camera_get_image(cv::Mat &output) {
    pthread_mutex_lock(&shared_mutex);

    if (!shared_buf || shared_width <= 0 || shared_height <= 0) {
        pthread_mutex_unlock(&shared_mutex);
        return false;
    }

    // cv::Matはメモリコピーが発生するため、この操作は安全
    output = cv::Mat(shared_height, shared_width, CV_8UC3, shared_buf).clone();
    //cv::Mat tmp(shared_height, shared_width, CV_8UC3, shared_buf);
    //cv::resize(tmp, output, cv::Size(shared_width / 2, shared_height / 2), cv::INTER_NEAREST);
    
    pthread_mutex_unlock(&shared_mutex);
    return true;
}

struct timespec  sttime2,mid1time2,mid2time2,edtime2;

/* 結果モニター用imageのクローンを排他制御で保存する　*/
bool result_put_image(cv::Mat image) {
    clock_gettime(CLOCK_REALTIME, &sttime2);

    // printf("result_put_image\n");
    int crop_width=image.cols*.75;
    int crop_height=image.rows/2;
    int crop_x=image.cols/2-crop_width/2;
    int crop_y=image.rows-crop_height;

    cv::Mat tmp,colorimg;
    camera.cropImage(image,crop_x, crop_y,crop_width,crop_height, tmp, 0.5);
    //camera.shared_frame = image.clone();
  //  cv::resize(image, camera.shared_frame, cv::Size(image.cols / 2, image.rows / 2), cv::INTER_NEAREST);

    int result_w=crop_width/2;
    int result_h=crop_height/2;
    double deep_width=0.6; // 奥側の比率
    double center_align=0.04; // センターずれ比率
    int deep_left = (result_w*(1-deep_width))/2; // 余白の半分
  //透視変換
    cv::Point2f par1[] = {{result_w*center_align,result_h},{result_w*center_align+result_w+result_w*center_align,result_h},{result_w*center_align/deep_width+result_w-deep_left,0},{result_w*center_align/deep_width+deep_left,0}};
    cv::Point2f par2[] = {{0,result_h/deep_width},{result_w,result_h/deep_width},{result_w,0},{0,0}};
    cv::Mat pspmat = cv::getPerspectiveTransform(par1, par2);
    cv::Size size = {result_w,result_h/deep_width};
    cv::Scalar borderValue= cv::Scalar(100,100,100);
    cv::warpPerspective(tmp , colorimg ,pspmat,size,cv::INTER_NEAREST,cv::BORDER_CONSTANT,borderValue);

    //グレースケール
   cv::cvtColor(colorimg, tmp, cv::COLOR_RGB2GRAY);		
    // 最大値
    double mMin, mMax;
    cv::Point minP, maxP;
    cv::minMaxLoc(tmp, &mMin, &mMax, &minP, &maxP);
    // 二値化
    cv::threshold(tmp,tmp, mMax*0.7 , 255, cv::THRESH_BINARY_INV); // 大きくすると広く拾う
    tmp = camera.cleanBinaryImage(tmp);
    //矩形検出
    camera.detectRotatedRectangles(tmp,colorimg,StreamCaptureRGB::EDGE_RIGHT);
    //結果保存
    camera.saveEdgeVectorsToStruct(StreamCaptureRGB::EDGE_RIGHT, colorimg.cols,  colorimg.rows);

    pthread_mutex_lock(&result_mutex);

    camera.shared_frame=colorimg.clone();

    pthread_mutex_unlock(&result_mutex);

    clock_gettime(CLOCK_REALTIME, &edtime2);
    double etime = diffspectime(sttime2,edtime2);
    printf("process time:%f fps:%f\n", etime , 1/etime);

    return true;
}

/* 排他制御で保存したフレームのクローンを取得する　*/
bool result_get_image(cv::Mat &output)
{
    pthread_mutex_lock(&result_mutex);
    // printf("result_get_image\n");

    output = camera.shared_frame.clone();

    pthread_mutex_unlock(&result_mutex);
    return true;
}



void *image_sender_thread(void *arg) {
    sigset_t oldsig;
    disable_interrupt(&oldsig);
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(12345); // 任意のポート
    inet_pton(AF_INET, "192.168.1.102", &server_addr.sin_addr); // ← PCのIPに変更
    while(true) {
        if (connect(sockfd, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            perror("retry connection");
            usleep(1000000);
            continue;
        }
        break;
    }
    cv::Mat frame;
    const int target_interval_us = 66667; // 約15fps相当
    //const int target_interval_us = 50000; // 約20fps相当
   // const int target_interval_us = 33333; // 約30fps相当

    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
    
        // if (!camera_get_image(frame)) {
        //     usleep(1000);  // キャプチャ失敗時は短くリトライ
        //     continue;
        // }
        // 
        result_get_image(frame);
        if (frame.empty()) {
            usleep(1000);
            continue;
        }

        std::vector<uchar> encoded;
        cv::imencode(".jpg", frame, encoded);
    
        uint32_t size = htonl(encoded.size());
        if (send(sockfd, &size, sizeof(size), 0) != 4) break;
        if (send(sockfd, encoded.data(), encoded.size(), 0) != (ssize_t)encoded.size()) break;
    
        // 処理にかかった時間を計測
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
      // printf("send frame %ld..\n",end);
    
        int wait_time = target_interval_us - elapsed_us;
        if (wait_time > 0)
            usleep(wait_time);
        else
            printf("delay..\n");
    }

    close(sockfd);
    printf("finish socket thread\n");


    return nullptr;
}

void *image_process_thread(void *arg) {
    sigset_t oldsig;
    disable_interrupt(&oldsig);
    const int target_interval_us = 33333; // 約30fps相当
    cv::Mat frame;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();
    
        if (!camera_get_image(frame)) {
            usleep(1000);  // キャプチャ失敗時は短くリトライ
            continue;
        }
        // 処理
        result_put_image(frame);

        // 処理にかかった時間を計測
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
      // printf("send frame %ld..\n",end);
    
        int wait_time = target_interval_us - elapsed_us;
        if (wait_time > 0)
            usleep(wait_time);
        else
            printf("delay..\n");
    }


    return nullptr;
}

void start_sender_thread() {
    if (pthread_create(&sender_thread, nullptr, image_sender_thread, nullptr) != 0) {
        std::cerr << "送信スレッドの起動に失敗しました\n";
    }
}

void start_result_thread() {
    if (pthread_create(&result_thread, nullptr, image_process_thread, nullptr) != 0) {
        std::cerr << "送信スレッドの起動に失敗しました\n";
    }
}

void getCameraResult(LineDetectionResult *result)
{
    *result = camera.detectedResult;
}

} // extern "C"
