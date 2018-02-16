#ifndef LOG_G
#define LOG_H

#include "RTT/SEGGER_RTT.h"

#define SILENCE 0
#define ERROR 1
#define DEBUG 2
#define VERBOSE 3

#if LOG_LEVEL == SILENCE
#define LOG_ERROR(x, ...)
#define LOG_DEBUG(x, ...)
#define LOG_VERBOSE(x, ...)
#elif LOG_LEVEL == ERROR
#define LOG_ERROR(x, ...) printf("[ERROR][%s]: " x, __FUNCTION__, ##__VA_ARGS__)
#define LOG_DEBUG(x, ...)
#define LOG_VERBOSE(x, ...)
#elif LOG_LEVEL == DEBUG
#define LOG_ERROR(x, ...) printf("[ERROR][%s]: " x, __FUNCTION__, ##__VA_ARGS__)
#define LOG_DEBUG(x, ...) printf("[DEBUG][%s]: " x, __FUNCTION__, ##__VA_ARGS__)
#define LOG_VERBOSE(x, ...)
#else /* VERBOSE */
#define LOG_ERROR(x, ...) printf("[ERROR][%s]: " x, __FUNCTION__, ##__VA_ARGS__)
#define LOG_DEBUG(x, ...) printf("[DEBUG][%s]: " x, __FUNCTION__, ##__VA_ARGS__)
#define LOG_VERBOSE(x, ...) printf("[VERBOSE][%s]" x, __FUNCTION__ , ##__VA_ARGS__)
#endif /* LOG_LEVEL */
#endif /* LOG_H */
