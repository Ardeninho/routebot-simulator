CC = cc
CFLAGS = -std=c17 -O2 -g -Wall -Wextra -Werror -pedantic
CPPFLAGS = -Iinclude
LDLIBS = -lm
CORE = src/planner.c src/controller.c src/sim.c
.PHONY: all test demo sanitize clean
all: build/routebot
build:
	mkdir -p build
build/routebot: $(CORE) src/main.c include/routebot.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) $(CORE) src/main.c -o $@ $(LDLIBS)
build/test: $(CORE) tests/test.c include/routebot.h | build
	$(CC) $(CPPFLAGS) $(CFLAGS) $(CORE) tests/test.c -o $@ $(LDLIBS)
test: build/test build/routebot
	./build/test
	python3 tests/integration.py
demo: build/routebot
	./build/routebot obstacle > build/telemetry.csv
	python3 tools/plot.py build/telemetry.csv build/demo.svg
sanitize:
	$(MAKE) clean
	$(MAKE) CFLAGS='-std=c17 -O1 -g -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer' test
clean:
	rm -rf build
