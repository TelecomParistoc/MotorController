#ifndef MARKDOWN_GENERATOR_H
#define MARKDOWN_GENERATOR_H

#include <stdio.h>
#include "interface_parser.h"

extern FILE* init_doc_file(char *file_name);
extern int write_doc_file(FILE *file, interface_element_t *entry);
extern void close_doc_file(FILE *file);

#endif
