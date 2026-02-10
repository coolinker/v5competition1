# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk

# ============================================================================
# Host-side unit tests (runs on your Mac/Linux, no VEX hardware needed)
# ============================================================================
HOST_CXX = g++
HOST_CXX_FLAGS = -std=c++17 -Wall -Wextra -g -I include -I src
HOST_TEST_SRC = test/host_tests.cpp
HOST_TEST_BIN = build/run_tests

test: $(HOST_TEST_BIN)
	@echo ""
	@./$(HOST_TEST_BIN)

$(HOST_TEST_BIN): $(HOST_TEST_SRC) $(wildcard src/control/*.cpp) $(wildcard src/localization/*.cpp) $(wildcard include/**/*.h) $(wildcard include/*.h)
	@mkdir -p build
	$(HOST_CXX) $(HOST_CXX_FLAGS) $(HOST_TEST_SRC) -o $(HOST_TEST_BIN) -lm

.PHONY: test
