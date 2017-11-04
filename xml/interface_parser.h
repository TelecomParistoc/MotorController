#ifndef PARSER_H
#define PARSER_H

#include <libxml/parser.h>
#include <libxml/tree.h>

#define NAME_MAX_SIZE 100
#define TYPE_MAX_SIZE 20
#define CAT_MAX_SIZE 40

typedef struct interface_element_s {
    char name[NAME_MAX_SIZE];
    char type[TYPE_MAX_SIZE];
    char c_name[NAME_MAX_SIZE];
    char category[CAT_MAX_SIZE];
    uint8_t address;
    uint8_t size;
    uint8_t read_access;
    uint8_t write_access;
    uint8_t gen_code;
    struct interface_element_s *next;
} interface_element_t;

extern xmlDoc* init_parsing(char* file_name);
extern interface_element_t* parse(xmlDoc* doc);
extern void close_parsing(xmlDoc* doc, interface_element_t* entry);

#endif
