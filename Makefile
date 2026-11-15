CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2

TARGET = motor_pid_sim
SRC    = src/main.cpp src/pid.cpp

all: $(TARGET)

$(TARGET): $(SRC) src/pid.h
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
