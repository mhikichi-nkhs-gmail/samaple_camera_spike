#include <pthread.h>

pthread_mutex_t shared_mutex = PTHREAD_MUTEX_INITIALIZER;
unsigned char *shared_buf = nullptr;
int shared_width = 0;
int shared_height = 0;
int shared_buf_size = 640 * 480 * 3;
