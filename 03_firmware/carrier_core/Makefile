CC ?= cc
CFLAGS += -std=gnu11 -O2 -Wall -Wextra -Werror -Iinclude -I../common/include
SRC = src/atlas_carrier.c ../common/src/atlas_dock_fsm.c ../common/src/atlas_pi.c ../common/src/atlas_proto.c
test: build/test_carrier
	./build/test_carrier
build/test_carrier: $(SRC) test/test_carrier.c include/*.h ../common/include/*.h
	mkdir -p build && $(CC) $(CFLAGS) -o $@ $(SRC) test/test_carrier.c -lm
.PHONY: test
