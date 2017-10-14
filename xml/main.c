#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

static void print_element(xmlNode *node) {
    xmlNode *cur_node;
    for (cur_node = node; cur_node; cur_node = cur_node->next) {
        if (cur_node->type == XML_ELEMENT_NODE) {
            printf("name %s, content %s\n", cur_node->name, cur_node->content);
            if (cur_node->properties != NULL) {
                printf("    prop name %s value %s\n", cur_node->properties[0].name, cur_node->properties[0].children[0].content);
            }
        } else if (cur_node->type == XML_TEXT_NODE) {
            printf("-- value %s\n", cur_node->content);
        }

        print_element(cur_node->children);
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Wrong number of arguments\n");
        return 1;
    }

    LIBXML_TEST_VERSION
    xmlDoc *doc;
    xmlNode *root_element;

    doc = xmlReadFile(argv[1], NULL, 0);

    root_element = xmlDocGetRootElement(doc);
    print_element(root_element);

    xmlFreeDoc(doc);
    xmlCleanupParser();
    return 0;
}
