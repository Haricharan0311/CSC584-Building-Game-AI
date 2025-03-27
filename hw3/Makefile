# Get all cpp files in current directory and subdirectories
rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
src := $(call rwildcard,./,*.cpp)

# Object files for each source file
obj := $(patsubst %.cpp,%.o,$(src))

# Separate object for update.cpp since it's shared
update_obj = update.o

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
all: q4 dijkstra_avg a_star



q4: q4.o $(update_obj)
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(LDFLAGS) $(UBUNTU_LIB)
endif

dijkstra_avg: dijkstra_avg.o
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(UBUNTU_LIB)
endif

a_star: a_star.o
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(MACOS_LIB)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(UBUNTU_LIB)
endif

# Pattern rules for object files
%.o: %.cpp
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) -std=c++17 -c $^ -o $@ $(MACOS_INCLUDE)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) -std=c++17 -c $^ -o $@ $(UBUNTU_INCLUDE)
endif

# Run all executables in specified order with inputs
run: all
	@echo "\n=== Running dijkstra_avg with airline_routes.txt (1 17) ===\n"
	@echo "1\n17" | ./dijkstra_avg airline_routes.txt
	@echo "\n=== Running dijkstra_avg with random_graph.txt (0 13626) ===\n"
	@echo "0\n13626" | ./dijkstra_avg random_graph.txt
	@echo "\n=== Running a_star with airline_routes.txt (1 17) ===\n"
	@echo "1\n17" | ./a_star airline_routes.txt
	@echo "\n=== Running a_star with random_graph.txt (0 13626) ===\n"
	@echo "0\n13626" | ./a_star random_graph.txt
	@echo "\n=== Running q4 ===\n"
	./q4

.PHONY: clean
clean:
	rm -f $(obj)  q4 dijkstra_avg a_star