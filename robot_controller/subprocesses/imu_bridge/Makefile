CXX = g++
CXXFLAGS = -Iinc -Wall -Wextra -std=c++20 -O2 -MMD -MP
LDFLAGS =

TARGET = imu_bridge

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
