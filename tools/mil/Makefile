# Harness MIL nativo (host-only). Nessun hardware, nessun MATLAB.
#   make        -> compila build/float_mil
#   make run    -> compila ed esegue, salvando il CSV in build/mission.csv
#   make clean

CC      ?= cc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wno-unused-parameter
COREINC := ../../lib/float_core/include
CORESRC := ../../lib/float_core/src/float_core.c

BUILD   := build
BIN     := $(BUILD)/float_mil
SRCS    := mil_main.c plant.c $(CORESRC)

.PHONY: all run clean
all: $(BIN)

$(BIN): $(SRCS) plant.h $(COREINC)/float_core.h $(COREINC)/float_core_types.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(COREINC) -I. $(SRCS) -lm -o $(BIN)

run: $(BIN)
	./$(BIN) | tee $(BUILD)/mission.csv

clean:
	rm -rf $(BUILD)
