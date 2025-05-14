#include "app.h"
#include "LineTracer.h"
#include <stdio.h>

#include "spike/pup/motor.h"
#include "spike/pup/colorsensor.h"

#include "stream_capture_c.h"
// #include "camera_thread.h"
#include <math.h>  // std::atan2, std::fmod


/* 関数プロトタイプ宣言 */
static int16_t steering_amount_calculation(void);
static int16_t camera_steering_amount_calculation(void);
static void motor_drive_control(int16_t);
static double vector_to_angle_deg(double dx, double dy);
static void scale_vector(double dx, double dy, double scale,
    double* out_dx, double* out_dy);
static void add_vectors(double dx1, double dy1, double dx2, double dy2,
    double* out_dx, double* out_dy);
static pup_motor_t *fg_left_motor;
static pup_motor_t *fg_right_motor;
static pup_device_t *fg_color_sensor;

void LineTracer_Configure(pbio_port_id_t left_motor_port, pbio_port_id_t right_motor_port, pbio_port_id_t color_sensor_port)
{

  /* センサー入力ポートの設定 */
  fg_color_sensor = pup_color_sensor_get_device(color_sensor_port);
  fg_left_motor   = pup_motor_get_device(left_motor_port);
  fg_right_motor   = pup_motor_get_device(right_motor_port);  

  pup_motor_setup(fg_left_motor,PUP_DIRECTION_COUNTERCLOCKWISE,true);
  pup_motor_setup(fg_right_motor,PUP_DIRECTION_CLOCKWISE,true);  

 int w=640,h=480;
//  int w=640,h=480;
 int ret = camera_initialize(w,h);
    printf("camera init %d\n",ret);
    start_sender_thread();
    start_result_thread();
}


/* ライントレースタスク(100msec周期で関数コールされる) */
void tracer_task(intptr_t unused) {
    int16_t steering_amount; /* ステアリング操舵量の計算 */

    /* ステアリング操舵量の計算 */
    //steering_amount = steering_amount_calculation();
    steering_amount = camera_steering_amount_calculation();
    
    /* 走行モータ制御 */
  // motor_drive_control(steering_amount);

    /* タスク終了 */
    ext_tsk();
}

/* ステアリング操舵量の計算 */
static int16_t steering_amount_calculation(void){

    uint16_t  target_brightness; /* 目標輝度値 */
    float32_t diff_brightness;   /* 目標輝度との差分値 */
    int16_t   steering_amount;   /* ステアリング操舵量 */
    pup_color_rgb_t rgb_val;

    /* 目標輝度値の計算 */
    target_brightness = (WHITE_BRIGHTNESS + BLACK_BRIGHTNESS) / 2;

    /* カラーセンサ値の取得 */
    rgb_val = pup_color_sensor_rgb(fg_color_sensor);

    /* 目標輝度値とカラーセンサ値の差分を計算 */
    diff_brightness = (float32_t)(target_brightness - rgb_val.g);

    /* ステアリング操舵量を計算 */
    steering_amount = (int16_t)(diff_brightness * STEERING_COEF);

    return steering_amount;
}

static int16_t camera_steering_amount_calculation(void){
    int16_t   steering_amount;   /* ステアリング操舵量 */
    double diff_pos;   /* 目標輝度との差分値 */
    LineDetectionResult result;
    getCameraResult(&result);

    double  target_pos=-20;
    double val = result.layers[2].start_x;
    double dir1_x=result.layers[2].dir_x;
    double dir1_y=result.layers[2].dir_y;
    double dir2_x=result.layers[1].dir_x;
    double dir2_y=result.layers[1].dir_y;
    double dir3_x=result.layers[0].dir_x;
    double dir3_y=result.layers[0].dir_y;
    double out1_x,out2_x;
    double out1_y,out2_y;
    scale_vector(dir2_x,dir2_y,0.4, &dir2_x,&dir2_y);
    add_vectors(dir1_x,dir1_y,dir2_x,dir2_y,  &out1_x,&out1_y);
    scale_vector(dir3_x,dir3_y,0.2, &dir3_x,&dir3_y);
    add_vectors(out1_x,out1_y,dir3_x,dir3_y,  &out2_x,&out2_y);

    diff_pos = target_pos- val; 
    if (diff_pos>150) diff_pos=150;
    if (diff_pos<-150) diff_pos=-150;

    double dir1_angle= -vector_to_angle_deg(out2_x,out2_y);
    printf("(%f,%f) (%f,%f)  %f , %f\n",result.layers[2].start_x,result.layers[2].start_y,result.layers[2].dir_x,result.layers[2].dir_y, diff_pos,dir1_angle);

//    steering_amount = (int16_t)(diff_pos * 0.086 + dir1_angle*0.5);
    steering_amount = (int16_t)(diff_pos * 0.062 + dir1_angle*0.96);

    return steering_amount;

}
/* 走行モータ制御 */
static void motor_drive_control(int16_t steering_amount){

    int left_motor_power, right_motor_power; /*左右モータ設定パワー*/

    /* 左右モータ駆動パワーの計算(走行エッジを右にする場合はRIGHT_EDGEに書き換えること) */
    left_motor_power  = (int)(BASE_SPEED + (steering_amount * LEFT_EDGE));
    right_motor_power = (int)(BASE_SPEED - (steering_amount * LEFT_EDGE));

    if (left_motor_power>100) {
        right_motor_power -= (left_motor_power-100);
        left_motor_power=100;
    }
    if (right_motor_power>100) {
        left_motor_power -= (right_motor_power-100);
        right_motor_power=100;
    }
    printf("LEFT,RIGHT = %d,%d\n",left_motor_power,right_motor_power);
    /* 左右モータ駆動パワーの設定 */
    pup_motor_set_power(fg_left_motor, left_motor_power);
    pup_motor_set_power(fg_right_motor, right_motor_power);

    return;
}


// 入力：正規化されたベクトル (dx, dy)
// 出力：Y軸基準の角度（度）、半時計回り正、[-180, +180]範囲
static double vector_to_angle_deg(double dx, double dy) {
    printf("vec2ang %f , %f\n",dx,dy);
    if (fabsf(dx) < 1e-6f && fabsf(dy) < 1e-6f) {
        return 0.0f;  // 無効ベクトル（長さほぼゼロ）の扱い
    }

    double radians = atan2f(dx, dy);  // Y軸を基準にするため atan2(dx, dy)
    double degrees = radians * 180.0f / 3.14159265f;
    return degrees;
}

static void scale_vector(double dx, double dy, double scale,
    double* out_dx, double* out_dy) {
    if (out_dx) *out_dx = dx * scale;
    if (out_dy) *out_dy = dy * scale;
}

static void add_vectors(double dx1, double dy1, double dx2, double dy2,
    double* out_dx, double* out_dy) {
    if (out_dx) *out_dx = dx1 + dx2;
    if (out_dy) *out_dy = dy1 + dy2;
}
