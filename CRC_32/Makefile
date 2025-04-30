CXX = g++
CXXFLAGS = -std=c++17 -Wall

TARGET = crc32_table
SRC = crc_32.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean

