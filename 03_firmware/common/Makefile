CC ?= cc
CFLAGS += -std=gnu11 -O2 -Wall -Wextra -Werror -Iinclude
SRC = src/atlas_dock_fsm.c src/atlas_pi.c src/atlas_proto.c
test: build/test_main
	./build/test_main
build/test_main: $(SRC) test/test_main.c include/*.h
	mkdir -p build && $(CC) $(CFLAGS) -o $@ $(SRC) test/test_main.c -lm
.PHONY: test
