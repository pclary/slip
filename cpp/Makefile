CXX     := g++
LD      := $(CXX)

INC     := -Isrc -Imjpro140/include
CFLAGS  := -std=c++14 -Wall -Wextra -Wpedantic -g -O3 -ffast-math -march=native
LDFLAGS := -g -O3 -L./mjpro140/bin -Wl,-rpath,"./mjpro140/bin"
LIBS    := -lmujoco140nogl
OUT     := slip

MODULES :=
SRC_DIR := src $(addprefix src/,$(MODULES))
OBJ_DIR := obj $(addprefix obj/,$(MODULES))
SRC     := $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
OBJ     := $(patsubst src/%.cpp,obj/%.o,$(SRC))

vpath %.cpp $(SRC_DIR)

define make-goal
$1/%.o: %.cpp
	$(CXX) $(CFLAGS) $(INC) -MMD -c $$< -o $$@
endef

all: checkdirs build

clean:
	rm -f $(OUT)
	rm -rf obj/

build: $(OBJ)
	$(LD) $^ -o $(OUT) $(LDFLAGS) $(LIBS)

checkdirs: $(OBJ_DIR)

$(OBJ_DIR):
	@mkdir -p $@

$(foreach bdir,$(OBJ_DIR),$(eval $(call make-goal,$(bdir))))

.PHONY: all checkdirs clean

-include $(OBJ:%.o=%.d)
