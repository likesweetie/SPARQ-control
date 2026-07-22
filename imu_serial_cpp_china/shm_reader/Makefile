CXX      = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I..
LDFLAGS  = -lrt

TARGET = shm_reader

$(TARGET): main.cpp ../shm.hpp
	$(CXX) $(CXXFLAGS) -o $@ main.cpp $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: clean
