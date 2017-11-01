SRC := main.c doc_generator.c header_generator.c interface_parser.c
OBJ := $(SRC:%.c=%.o)
CFLAGS += -Wall -Wextra
LFLAGS += -lxml2

all: xml

xml: $(OBJ)
	gcc -o $@ $^ $(LFLAGS)

clean:
	rm -f *.o
	rm xml
