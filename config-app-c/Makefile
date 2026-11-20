# Drone config / PID tuning GUI (raylib + raygui).
#
#   make          build build/pid_tuner
#   make run      build and run
#   make clean
#
# Needs raylib (and raygui.h) installed. If pkg-config knows raylib it is used;
# otherwise the usual Linux link line is tried.

CC      ?= cc
APP_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
REPO    := $(abspath $(APP_DIR)/..)
BUILD   := $(APP_DIR)/build

SRC := $(wildcard $(APP_DIR)/src/*.c)
OBJ := $(patsubst $(APP_DIR)/src/%.c,$(BUILD)/%.o,$(SRC))

RAYLIB_CFLAGS := $(shell pkg-config --cflags raylib 2>/dev/null)
RAYLIB_LIBS   := $(shell pkg-config --libs raylib 2>/dev/null)
ifeq ($(strip $(RAYLIB_LIBS)),)
RAYLIB_LIBS := -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
endif

# protocol.h lives with the firmware so both sides share one definition.
CFLAGS  ?= -O2 -g
CFLAGS  += -std=gnu11 -D_DEFAULT_SOURCE -Wall -Wextra \
           -I$(APP_DIR)/src -I$(REPO)/libs/espnow-comm $(RAYLIB_CFLAGS) \
           -DREPO_DIR='"$(REPO)"' -DAPP_DIR='"$(APP_DIR)"' -MMD -MP
LDLIBS  += $(RAYLIB_LIBS) -lm -lpthread

TARGET := $(BUILD)/pid_tuner

.PHONY: all run clean compile_commands
all: $(TARGET)

# Regenerates compile_commands.json so clangd sees the real flags.
compile_commands:
	python3 $(APP_DIR)/gen_compile_commands.py

$(TARGET): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BUILD)/%.o: $(APP_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD):
	mkdir -p $@

run: $(TARGET)
	$(TARGET)

clean:
	rm -rf $(BUILD)

-include $(OBJ:.o=.d)
