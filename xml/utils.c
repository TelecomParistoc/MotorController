#include "utils.h"

int is_small(char c) {
    if ((c >= 'a') && (c <= 'z')) {
        return 1;
    } else {
        return 0;
    }
}

void write_addr(char *category, char *c_name, FILE *file, modifier_t modifier) {
    int index = 0;

    while (category[index] != '\0') {
        if(is_small(category[index])) {
            fputc(category[index] + 'A' - 'a', file);
        } else if (category[index] == '.') {
            fputc('_', file);
        } else {
            fputc(category[index], file);
        }
        index++;
    }

    if (index != 0) {
        fputc('_', file);
    }
    index = 0;

    while (c_name[index] != '\0') {
        if(is_small(c_name[index])) {
            fputc(c_name[index] + 'A' - 'a', file);
        } else if (c_name[index] == '.') {
            fputc('_', file);
        } else {
            fputc(c_name[index], file);
        }
        index++;
    }

    switch (modifier) {
        case NORMAL:
            break;
        case LOW:
            fprintf(file, "_LOW");
            break;
        case HIGH:
            fprintf(file, "_HIGH");
            break;
    }
    fprintf(file, "_ADDR");
}
