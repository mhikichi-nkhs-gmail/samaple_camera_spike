#ifndef STREAM_CAPTURE_C_H
#define STREAM_CAPTURE_C_H

#include <stdbool.h>
#include "vectorStruct.h"

#ifdef __cplusplus
extern "C" {
#endif

bool camera_initialize(int width, int height);
bool camera_capture(unsigned char *buf, int bufsize, int *w, int *h);
void camera_shutdown(void);

void start_sender_thread();
void start_result_thread();
void getCameraResult(LineDetectionResult *result);

#ifdef __cplusplus
}
#endif

#endif
