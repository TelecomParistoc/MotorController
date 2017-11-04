#include "interface_generator.h"
#include "utils.h"

#define FILE_HEADER "#ifndef I2C_INTERFACE_GEN_C\n\
#define I2C_INTERFACE_GEN_C\n"

#define RECEPTION_HEADER "\nstatic void i2c_vt_cb(void* param)\n{\n    (void)param;\n"

#define RECEPTION_SWITCH_HEADER "\n    if (rx_buffer[0] != NO_DATA) {\n        switch(rx_buffer[0])\n        {\n"

#define RECEPTION_SWITCH_FOOTER "\
        default:\n\
            rx_special_cases(rx_buffer[0]);\n\
            break;\n\
        }\n\
    }\n"

#define RECEPTION_FOOTER "\n    /* Free the rx_buffer */\n\
    rx_buffer[0] = NO_DATA;\n\
    rx_buffer[1] = NO_DATA;\n\
    rx_buffer[2] = NO_DATA;\n}\n\n"

#define SENDING_HEADER "\nstatic void i2c_address_match(I2CDriver* i2cp) {\n\
    (void)i2cp;\n\
    uint16_t value;\n\
    bool single_byte = FALSE;\n"

#define SENDING_SWITCH_HEADER "\n    if (rx_buffer[0] != NO_DATA) {\n\
        /* Start of the read part of a read-after-write exchange */\n\
        chSysLockFromISR();\n\
        chVTResetI(&i2c_vt);\n\
        chSysUnlockFromISR();\n\n\
        /* Prepare the answer */\n\
        switch (rx_buffer[0]) {\n"

#define SENDING_SWITCH_FOOTER "        default:\n\
            tx_special_cases(rx_buffer[0], &value);\n\
            break;\n\
        }\n"

#define SENDING_FOOTER "\n        if (single_byte == FALSE) {\n\
            tx_buffer[1] = (uint8_t)((value & 0xFF00) >> 8);\n\
            tx_buffer[0] = (uint8_t)(value & 0x00FF);\n\
            i2c_response.size = 2U;\n\
        } else {\n\
            tx_buffer[0] = (uint8_t)(value & 0x00FF);\n\
            i2c_response.size = 1U;\n\
        }\n\n\
        /* Free the rx buffer */\n\
        rx_buffer[0] = NO_DATA;\n\
    } else {\n\
        /* Start of a write exchange */\n\
        chSysLockFromISR();\n\
        chVTSetI(&i2c_vt, US2ST(800), i2c_vt_cb, NULL);\n\
        chSysUnlockFromISR();\n\
    }\n\
}\n"

#define FILE_FOOTER "\n#endif /* I2C_INTERFACE_GEN_C */\n"

static void write_tmp_variables(FILE *file, interface_element_t *entry) {
    while (entry != NULL) {
        if ((entry->size == 32) && (entry->write_access == 1) && (entry->gen_code == 1)) {
            fprintf(file, "    static int32_t tmp_%s = 0;\n", entry->c_name);
        }
        entry = entry->next;
    }
}

static void write_reception_element(FILE *file, interface_element_t *entry, int high) {
    if (entry->write_access == 0) {
        return;
    }

    fprintf(file, "        case ");
    if (entry->size == 32) {
        if (high) {
            write_addr(entry->category, entry->c_name, file, HIGH);
        } else {
            write_addr(entry->category, entry->c_name, file, LOW);
        }
    } else {
        write_addr(entry->category, entry->c_name, file, NORMAL);
    }
    fprintf(file, ":\n");

    if (entry->size < 32) {
        fprintf(file, "            ");
        if (entry->category[0] != '\0') {
            fprintf(file, "%s.", entry->category);
        }
        fprintf(file, "%s = ", entry->c_name);

        if (entry->size == 8) {
            fprintf(file, "rx_buffer[1];\n");
        } else {
            fprintf(file, "(rx_buffer[2] << 8) | rx_buffer[1];\n");
        }
    } else {
        if (high) {
            fprintf(file, "            tmp_%s |= ((rx_buffer[2] << 24U) | (rx_buffer[1] << 16U));\n", entry->c_name);
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
        if (entry->gen_code) {
            write_reception_element(file, entry, 0);
        }
        entry = entry->next;
    }

    fprintf(file, RECEPTION_SWITCH_FOOTER);
    fprintf(file, RECEPTION_FOOTER);

    return 0;
}

static void write_sending_tmp_var(FILE *file, interface_element_t *entry) {
    while (entry != NULL) {
        if ((entry->size == 32) && (entry->read_access == 1) && (entry->gen_code == 1)) {
            fprintf(file, "    static int32_t saved_%s = 0;\n", entry->c_name);
        }
        entry = entry->next;
    }
}

static void write_sending_element(FILE *file, interface_element_t *entry, int high) {
    if (entry->read_access == 0) {
        return;
    }

    fprintf(file, "        case ");
    if (entry->size == 32) {
        if (high) {
            write_addr(entry->category, entry->c_name, file, HIGH);
        } else {
            write_addr(entry->category, entry->c_name, file, LOW);
        }
    } else {
        write_addr(entry->category, entry->c_name, file, NORMAL);
    }
    fprintf(file, ":\n");

    if (entry->size < 32) {
        fprintf(file, "            value = ");
        if (entry->category[0] != '\0') {
            fprintf(file, "%s.", entry->category);
        }
        fprintf(file, "%s;\n", entry->c_name);

        if (entry->size == 8) {
            fprintf(file, "            single_byte = TRUE;\n");
        }
    } else {
        if (high) {
            fprintf(file, "            value = (saved_%s && 0xFFFF0000U) >> 16U;\n", entry->c_name);
        } else {
            fprintf(file, "            saved_%s = ", entry->c_name);
            if (entry->category[0] != '\0') {
                fprintf(file, "%s.\n", entry->category);
            }
            fprintf(file, "%s;\n", entry->c_name);
            fprintf(file, "            value = saved_%s & 0x0000FFFFU;\n", entry->c_name);
        }
    }

    fprintf(file, "            break;\n");

    if ((entry->size == 32) && (high == 0)) {
        write_sending_element(file, entry, 1);
    }

}

static int write_sending_function(FILE* file, interface_element_t *entry) {
    /* Check input parameters */
    if (file == NULL) {
        return -1;
    }

    fprintf(file, SENDING_HEADER);
    write_sending_tmp_var(file, entry);
    fprintf(file, SENDING_SWITCH_HEADER);

    while (entry != NULL) {
        if (entry->gen_code) {
            write_sending_element(file, entry, 0);
        }
        entry = entry->next;
    }

    fprintf(file, SENDING_SWITCH_FOOTER);
    fprintf(file, SENDING_FOOTER);

    return 0;
}


FILE* init_interface_file(char *file_name) {
    FILE *file;
    /* Check input parameters */
    if (file_name == NULL) {
        return NULL;
    }

    file = fopen(file_name, "w");
    if (file == NULL) {
        return NULL;
    }

    return file;
}

int write_interface_file(FILE *file, interface_element_t *entry) {
    /* Check input parameters */
    if (file == NULL) {
        return -1;
    } else if (entry == NULL) {
        return -2;
    }

    fprintf(file, FILE_HEADER);
    write_reception_function(file, entry);
    write_sending_function(file, entry);
    fprintf(file, FILE_FOOTER);

    return 0;
}

void close_interface_file(FILE *file) {
    /* Check input parameters */
    if (file == NULL) {
        return;
    }

    fclose(file);
}
