#ifndef INTERFACE_GENERATOR_H
#define INTERFACE_GENERATOR_H

#include "interface_parser.h"

extern FILE* init_interface_file(char *file_name);
extern int write_interface_file(FILE *file, interface_element_t *entry);
extern void close_interface_file(FILE *file);

#endif
