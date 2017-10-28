#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#define NAME_MAX_SIZE 100
#define ADDRESS_MAX_SIZE 5
#define TYPE_MAX_SIZE 20

typedef struct IntfEntry_s {
    char name[NAME_MAX_SIZE];
    char address[ADDRESS_MAX_SIZE];
    char type[TYPE_MAX_SIZE];
    uint8_t size;
    uint8_t read_access;
    uint8_t write_access;
} IntfEntry_t;

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
            snprintf(entry->address, 5, "%s", entry_element->children->content);
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

static void print_entry(IntfEntry_t* entry, int fd) {
    if (entry == NULL) {
        return;
    }

    write(fd, "|", 1);
    write(fd, entry->name, strlen(entry->name));
    write(fd, "|", 1);
    write(fd, entry->address, strlen(entry->address));
    write(fd, "|", 1);
    if (entry->read_access) {
        write(fd, "R", 1);
    }
    if (entry->write_access) {
        if (entry->read_access)
            write(fd, "/", 1);
        write(fd, "W", 1);
    }
    write(fd, "|", 1);
    write(fd, entry->type, strlen(entry->type));
    write(fd, "|\n", 2);
}

static void process_entry(xmlNode *node, int fd) {
    IntfEntry_t entry;
    parse_entry(node, &entry);
    print_entry(&entry, fd);
}

static void print_element(xmlNode *node, int fd) {
    xmlNode *cur_node;
    for (cur_node = node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            if (strcmp(cur_node->name, "register") == 0) {
                process_entry(cur_node, fd);
            } else {
                print_element(cur_node->children, fd);
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
    int fd;

    doc = xmlReadFile(argv[1], NULL, 0);
    fd = creat(argv[2], S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
    if (fd < 0) {
        printf("Error when opening file %s\n", argv[2]);
        return 1;
    }
    write(fd, "|Name|Address|Access|Size (in bits)|\n|----|-------|------|--------------|\n", 74);
    root_element = xmlDocGetRootElement(doc);
    print_element(root_element, fd);

    xmlFreeDoc(doc);
    xmlCleanupParser();
    return 0;
}
