#ifndef HEADER_GENERATOR_H
#define HEADER_GENERATOR_H

#include "interface_parser.h"

extern FILE* init_header_file(char *file_name);
extern int write_header_file(FILE *file, interface_element_t *entry);
extern void close_header_file(FILE *file);

#endif /* HEADER_GENERATOR_H */
