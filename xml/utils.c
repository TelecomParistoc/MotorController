#include "utils.h"

int is_small(char c) {
    if ((c >= 'a') && (c <= 'z')) {
        return 1;
    } else {
        return 0;
    }
}
