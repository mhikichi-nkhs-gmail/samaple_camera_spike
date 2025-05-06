// camera_shared.h
#ifndef CAMERA_SHARED_H
#define CAMERA_SHARED_H

#include <pthread.h>

extern pthread_mutex_t shared_mutex;
extern unsigned char *shared_buf;
extern int shared_width;
extern int shared_height;
extern int shared_buf_size;
#endif
