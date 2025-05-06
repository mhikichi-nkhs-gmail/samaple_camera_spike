#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H

// OpenCV 非依存のベクトル情報
typedef struct {
    float start_x;   // 始点X
    float start_y;   // 始点Y
    float dir_x;     // 正規化された方向ベクトルX
    float dir_y;     // 正規化された方向ベクトルY
} LineVector;

typedef struct {
    LineVector layers[3];  // [0]=上, [1]=中, [2]=下
} LineDetectionResult;

#endif