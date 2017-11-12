#include "doc_generator.h"
#include "config.h"

/******************************************************************************/
/*                               Defines                                      */
/******************************************************************************/
#define DOC_FILE_HEADER "\n\n|Name|Address|Access|Size (in bits)|\n|----|-------|------|--------------|\n"

/******************************************************************************/
/*                           Static functions                                 */
/******************************************************************************/
static void write_entry(FILE *file, interface_element_t *element) {
    /* Check input parameters */
    if (file == NULL) {
        return;
    } else if (element == NULL) {
        return;
    }

    fprintf(file, "|%s|0x%02x|", element->name, element->address);
    if (element->read_access) {
        fputc('R', file);
    }
    if (element->write_access) {
        if (element->read_access)
            fputc('/', file);
        fputc('W', file);
    }
    fprintf(file, "|%s|\n", element->type);
}

/******************************************************************************/
/*                           Public functions                                 */
/******************************************************************************/
FILE* init_doc_file(char *file_name) {
    FILE *file;
    int ret;

    if (file_name == NULL) {
#if DEBUG
        printf("Null doc file name\n");
#endif /* DEBUG */
        return NULL;
    }

    file = fopen(file_name, "w");
    if (file == NULL) {
#if DEBUG
        printf("Error when opening file %s\n", file_name);
#endif /* DEBUG */
        return NULL;
    }

    ret = fprintf(file, DOC_FILE_HEADER);
    if (ret < 0) {
#if DEBUG
        printf("Error when writing header in doc file\n");
#endif /* DEBUG */
    }
    return file;
}

int write_doc_file(FILE *file, interface_element_t *entry) {
    /* Check input parameters */
    if (file == NULL) {
        return -1;
    } else if (entry == NULL) {
        return -2;
    }

    while (entry != NULL) {
        write_entry(file, entry);
        entry = entry->next;
    }

    return 0;
}

void close_doc_file(FILE *file) {
    /* Check input parameters */
    if (file == NULL) {
        return;
    }

    fclose(file);
}
