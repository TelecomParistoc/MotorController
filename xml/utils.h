#ifndef UTILS_H
#define UTILS_H

#include <stdio.h>

typedef enum modifier_s {
    NORMAL,
    LOW,
    HIGH
} modifier_t;

extern int is_small(char c);

extern void write_addr(char *category, char *c_name, FILE *file, modifier_t modifier);

#endif /* UTILS_H */
