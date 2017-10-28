#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define NAME_MAX_SIZE 100
#define ADDRESS_MAX_SIZE 5
#define TYPE_MAX_SIZE 20

#define I2C_INTERFACE_HEADER "i2c_interface.h"
#define I2C_INTERFACE_HEADER_HEADER "#ifndef I2C_INTERFACE_H\n#define I2C_INTERFACE_H\n#include \"ch.h\"\n#include \"hal.h\"\n\n"
#define I2C_INTERFACE_HEADER_FOOTER "\n#endif /* I2C_INTERFACE_H */"
#define CREATE_ITF_HEADER 1


typedef struct IntfEntry_s {
    char name[NAME_MAX_SIZE];
    char type[TYPE_MAX_SIZE];
    uint8_t address;
    uint8_t size;
    uint8_t read_access;
    uint8_t write_access;
} IntfEntry_t;


#if CREATE_ITF_HEADER
FILE *itf_header_file;
#endif

static int isNumeric(char c) {
    if ((c >= '0') && (c <= '9')) {
        return 1;
    } else {
        return 0;
    }
}

/* Extract size from well-formed type */
static int get_size(char* type) {
    int index;
    int size = 0;
    int multiplier = 1;

    if (type == NULL) {
        return 0;
    }

    index = strlen(type) - 1;
    while (isNumeric(type[index])) {
        size += (type[index] - '0') * multiplier;
        index--;
        multiplier *= 10;
    }

    return size;
}

static int get_addr(char *addr_str) {
    int index;
    int multiplier = 1;
    int addr = 0;

    if (addr_str == NULL) {
        return 255;
    }

    index = strlen(addr_str) - 1;
    while(isNumeric(addr_str[index])) {
        addr += (addr_str[index] - '0') * multiplier;
        index--;
        multiplier *= 16;
    }

    return addr;
}

static void parse_entry(xmlNode* node, IntfEntry_t* entry) {
    xmlNode *entry_element;
    entry_element = node->children;
    while (entry_element != NULL) {
        if (strcmp(entry_element->name, "name") == 0) {
            snprintf(entry->name, NAME_MAX_SIZE, "%s", entry_element->children->content);
        } else if (strcmp(entry_element->name, "type") == 0) {
            snprintf(entry->type, 20, "%s", entry_element->children->content);
            entry->size = get_size(entry->type);
        } else if (strcmp(entry_element->name, "address") == 0) {
            entry->address = get_addr(entry_element->children->content);
        } else if (strcmp(entry_element->name, "read") == 0) {
            if (strcmp(entry_element->children->content, "true") == 0)
                entry->read_access = 1;
        } else if (strcmp(entry_element->name, "write") == 0) {
            if (strcmp(entry_element->children->content, "true") == 0)
                entry->write_access = 1;
        }
        entry_element = entry_element->next;
    }
}

static int is_small(char c) {
    if ((c >= 'a') && (c <= 'z')) {
        return 1;
    } else {
        return 0;
    }
}

static void print_entry_interface_header(IntfEntry_t* entry) {
    int index = 0;

    fprintf(itf_header_file, "#define ");

    while ((entry->name[index] != ' ') && (entry->name[index] != '\0')) {
        if(is_small(entry->name[index])) {
            fputc(entry->name[index] + 'A' - 'a', itf_header_file);
        } else {
            fputc(entry->name[index], itf_header_file);
        }
        index++;
    }

    if (entry->size == 32) {
        fprintf(itf_header_file, "_LOW");
    }

    fprintf(itf_header_file, "_ADDR 0x%02x\n", entry->address);

    if (entry->size == 32) {
        fprintf(itf_header_file, "#define ");
        index = 0;

        while ((entry->name[index] != ' ') && (entry->name[index] != '\0')) {
            if(is_small(entry->name[index])) {
                fputc(entry->name[index] + 'A' - 'a', itf_header_file);
            } else {
                fputc(entry->name[index], itf_header_file);
            }
            index++;
        }
        fprintf(itf_header_file, "_HIGH_ADDR 0x%02x\n", entry->address + 2);
    }
}

static void print_entry(IntfEntry_t* entry, FILE *file) {
    if (entry == NULL) {
        return;
    }

    fprintf(file, "|%s|0x%02x|", entry->name, entry->address);
    if (entry->read_access) {
        fputc('R', file);
    }
    if (entry->write_access) {
        if (entry->read_access)
            fputc('/', file);
        fputc('W', file);
    }
    fprintf(file, "|%s|\n", entry->type);
}

static void process_entry(xmlNode *node, FILE *file) {
    IntfEntry_t entry;
    parse_entry(node, &entry);
    print_entry(&entry, file);
#if CREATE_ITF_HEADER
    print_entry_interface_header(&entry);
#endif
}

static void print_element(xmlNode *node, FILE *file) {
    xmlNode *cur_node;
    for (cur_node = node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            if (strcmp(cur_node->name, "register") == 0) {
                process_entry(cur_node, file);
            } else {
                print_element(cur_node->children, file);
            }

            /*if (cur_node->properties != NULL) {
                printf("    prop name %s value %s\n", cur_node->properties[0].name,
                        cur_node->properties[0].children[0].content);
            }*/
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        printf("Wrong number of arguments\n");
        return 1;
    }

    LIBXML_TEST_VERSION
    xmlDoc *doc;
    xmlNode *root_element;
    FILE *file;

    doc = xmlReadFile(argv[1], NULL, 0);
    file = fopen(argv[2], "w");
    if (file == NULL) {
        printf("Error when opening file %s\n", argv[2]);
        return 1;
    }

#if CREATE_ITF_HEADER
    itf_header_file = fopen(I2C_INTERFACE_HEADER, "w");
    if (itf_header_file == NULL) {
        printf("Error when creating file %s\n", I2C_INTERFACE_HEADER);
        return 1;
    }
    fprintf(itf_header_file, I2C_INTERFACE_HEADER_HEADER);
#endif

    fprintf(file, "|Name|Address|Access|Size (in bits)|\n|----|-------|------|--------------|\n");
    root_element = xmlDocGetRootElement(doc);
    print_element(root_element, file);

#if CREATE_ITF_HEADER
    fprintf(itf_header_file, I2C_INTERFACE_HEADER_FOOTER);
    fclose(itf_header_file);
#endif
    fclose(file);

    xmlFreeDoc(doc);
    xmlCleanupParser();
    return 0;
}
