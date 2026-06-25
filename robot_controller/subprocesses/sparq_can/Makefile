CXX = g++
CXXFLAGS = -Iinc -Wall -Wextra -std=c++20 -O2 -MMD -MP
LDFLAGS =

TARGET = SPARQ_CAN

# Source and Object files
SRC_DIR = src
INC_DIR = inc
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(OBJS:.o=.d)

# Default target
all: $(TARGET)
	@chmod +x start.sh

# Link the executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

# Compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Load auto-generated header dependencies
-include $(DEPS)

# Clean up
clean:
	rm -f $(SRC_DIR)/*.o $(SRC_DIR)/*.d $(TARGET)

.PHONY: all clean
