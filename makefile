CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17

OPENCV = `pkg-config --cflags --libs opencv4`

TARGET = t
SRC = v2beta.cpp

all:
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(OPENCV)

run: all
	./$(TARGET)

clean:
	rm -f $(TARGET)