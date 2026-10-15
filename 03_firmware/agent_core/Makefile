CC ?= cc
CFLAGS += -std=gnu11 -O2 -Wall -Wextra -Werror -Iinclude -I../common/include
SRC = src/atlas_agent.c ../common/src/atlas_pi.c ../common/src/atlas_proto.c
test: build/test_agent
	./build/test_agent
build/test_agent: $(SRC) test/test_agent.c include/*.h
	mkdir -p build && $(CC) $(CFLAGS) -o $@ $(SRC) test/test_agent.c -lm
.PHONY: test
