#include <stdio.h>

#include "interface_parser.h"
#include "doc_generator.h"
#include "header_generator.h"
#include "interface_generator.h"
#include "config.h"

#define INTERFACE_HEADER_FILE_NAME "i2c_interface_addr.h"
#define INTERFACE_CODE_FILE_NAME "i2c_interface_gen.c"

int main(int argc, char* argv[]) {
    if (argc != 3) {
        printf("Wrong number of arguments\n");
        return 1;
    }

    xmlDoc *doc = init_parsing(argv[1]);
    interface_element_t *entry = parse(doc);

#if GEN_DOC
    FILE *doc_file = init_doc_file(argv[2]);
    write_doc_file(doc_file, entry);
    close_doc_file(doc_file);
#endif /* GEN_DOC */

#if GEN_HEADER
    FILE *header_file = init_header_file(INTERFACE_HEADER_FILE_NAME);
    write_header_file(header_file, entry);
    close_header_file(header_file);
#endif /* GEN_HEADER */

#if GEN_CODE
    FILE *code_file = init_interface_file(INTERFACE_CODE_FILE_NAME);
    write_interface_file(code_file, entry);
    close_interface_file(code_file);
#endif /* GEN_CODE */
    close_parsing(doc, entry);

    return 0;
}
