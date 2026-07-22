CXX = g++
CXXFLAGS = -Iinc -I../sparq_can/inc -Wall -Wextra -std=c++20 -O2 -MMD -MP
LDFLAGS =

TARGET = policy_bridge

# Header-only reuse of sparq_can/inc/Sharemem.hpp + SPARQ_config.h -- no
# sparq_can .cpp files (RobstrideMotor.cpp, CAN socket code) are compiled or
# linked here. This process never touches CAN directly, only the two shm
# segments.
SRC_DIR = src
INC_DIR = inc
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:.cpp=.o)
DEPS = $(OBJS:.o=.d)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(DEPS)

clean:
	rm -f $(SRC_DIR)/*.o $(SRC_DIR)/*.d $(TARGET)

.PHONY: all clean
