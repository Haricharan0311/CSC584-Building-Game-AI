# Get all cpp files in current directory and subdirectories
rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
src := $(call rwildcard,./,*.cpp)

# Object files for each source file
obj := $(patsubst %.cpp,%.o,$(src))

# Separate objects for shared files
update_obj = update.o
behavior_tree_obj = BehaviorTree.o

LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system 

# Include paths for different systems
INTELMAC_INCLUDE=-I/usr/local/include							# Intel mac
APPLESILICON_INCLUDE=-I/opt/homebrew/include					# Apple Silicon
UBUNTU_APPLESILICON_INCLUDE=-I/usr/include 						# Apple Silicon Ubuntu VM
UBUNTU_INTEL_INCLUDE=-I/usr/include 							# Intel Ubuntu VM

INTELMAC_LIB=-L/usr/local/lib									# Intel mac
APPLESILICON_LIB=-L/opt/homebrew/lib							# Apple Silicon
UBUNTU_APPLESILICON_LIB=-L/usr/lib/aarch64-linux-gnu			# Apple Silicon Ubuntu VM
UBUNTU_INTEL_LIB=-L/usr/lib/x86_64-linux-gnu					# Intel Ubuntu VM

MACOS_INCLUDE=$(APPLESILICON_INCLUDE)
MACOS_LIB=$(APPLESILICON_LIB)
UBUNTU_INCLUDE=$(UBUNTU_APPLESILICON_INCLUDE)
UBUNTU_LIB=$(UBUNTU_APPLESILICON_LIB)

MACOS_COMPILER=/usr/bin/clang++
UBUNTU_COMPILER=/usr/bin/g++

uname_s := $(shell uname -s)

# Targets for all executables
all: decision q4 train_dt q5

decision: decision.o $(update_obj)
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(UBUNTU_LIB)
endif

q4: q4.o $(update_obj) $(behavior_tree_obj)
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(UBUNTU_LIB)
endif

train_dt: train_dt.o
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(UBUNTU_LIB)
endif

q5: q5.o $(behavior_tree_obj)
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(UBUNTU_LIB)
endif

# Pattern rules for object files
%.o: %.cpp
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -c $^ -o $@ $(MACOS_INCLUDE)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -c $^ -o $@ $(UBUNTU_INCLUDE)
endif

# Run all executables in specified order
run: all
	@echo "\n=== Running decision.cpp to generate initial data ===\n"
	./decision
	@echo "\n=== Running q4.cpp to generate training data ===\n"
	./q4
	@echo "\n=== Running q5.cpp with trained decision tree ===\n"
	./q5

.PHONY: clean
clean:
	rm -f $(obj) decision q4 train_dt q5 