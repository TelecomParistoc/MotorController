#include "interface_generator.h"
#include "utils.h"

#define INCLUDES "\
#include \"i2c_interface.h\"\n\
#include \"i2c_interface_gen.h\"\n\n"

#define RECEPTION_HEADER "void i2c_vt_cb(void* param)\n{\n    (void)param\n"

#define RECEPTION_SWITCH_HEADER "\n    if (rx_buffer[0] != NO_DATA) {\n        switch(rx_buffer[0])\n        {\n"

#define RECEPTION_FOOTER "\
        default:\n\
            rx_special_case();\n\
            break;\n\
        }\n\
    }\n\n\
    /* Free the rx_buffer */\n\
    rx_buffer[0] = NO_DATA;\n\
    rx_buffer[1] = NO_DATA;\n\
    rx_buffer[2] = NO_DATA;\n}\n\n"


static void write_tmp_variables(FILE *file, interface_element_t *entry) {
    while (entry != NULL) {
        if (entry->size == 32) {
            fprintf(file, "    static int32_t tmp_%s = 0;\n", entry->c_name);
        }
        entry = entry->next;
    }
}

static void write_reception_element(FILE *file, interface_element_t *entry, int high) {
    int index = 0;
    fprintf(file, "        case ");
    while ((entry->name[index] != ' ') && (entry->name[index] != '\0')) {
        if(is_small(entry->name[index])) {
            fputc(entry->name[index] + 'A' - 'a', file);
        } else {
            fputc(entry->name[index], file);
        }
        index++;
    }
    if (entry->size == 32) {
        if (high) {
            fprintf(file, "_HIGH");
        } else {
            fprintf(file, "_LOW");
        }
    }
    fprintf(file, "_ADDR:\n");

    if (entry->size < 32) {
        fprintf(file, "            %s.%s = (rx_buffer[2] << 8) | rx_buffer[1];\n", entry->category, entry->c_name);
    } else {
        if (high) {
            fprintf(file, "            tmp_%s |= ((rx_buffer[2] << 24U) | (rx_buffer[1] << 16U))\n", entry->c_name);
            fprintf(file, "            %s.%s = tmp_%s;\n", entry->category, entry->c_name, entry->c_name);
        } else {
            fprintf(file, "            tmp_%s = (rx_buffer[2] << 8) | rx_buffer[1];\n", entry->c_name);
        }
    }

    fprintf(file, "            break;\n");

    if ((entry->size == 32) && (high == 0)) {
        write_reception_element(file, entry, 1);
    }

}

static int write_reception_function(FILE *file, interface_element_t *entry) {
    /* Check input parameters */
    if (file == NULL) {
        return -1;
    }

    fprintf(file, RECEPTION_HEADER);
    write_tmp_variables(file, entry);
    fprintf(file, RECEPTION_SWITCH_HEADER);

    while (entry != NULL) {
        write_reception_element(file, entry, 0);
        entry = entry->next;
    }

    fprintf(file, RECEPTION_FOOTER);

    return 0;
}


FILE* init_interface_file(char *file_name) {
    FILE *file;
    int ret;

    /* Check input parameters */
    if (file_name == NULL) {
        return NULL;
    }

    file = fopen(file_name, "w");
    if (file == NULL) {
        return NULL;
    }

    ret = fprintf(file, INCLUDES);

    return file;
}

int write_interface_file(FILE *file, interface_element_t *entry) {
    /* Check input parameters */
    if (file == NULL) {
        return -1;
    } else if (entry == NULL) {
        return -2;
    }

    write_reception_function(file, entry);

    return 0;
}

void close_interface_file(FILE *file) {
    /* Check input parameters */
    if (file == NULL) {
        return;
    }

    fclose(file);
}
