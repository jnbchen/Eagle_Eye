CXX = g++
CXXFLAGS = -Wall -O3
LDFLAGS = -Wall
LDLIBS = -lopencv_core -lopencv_flann -lopencv_highgui -lopencv_imgproc

SOURCES = offline_test.cpp 
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = offline_test

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@ $(LDLIBS)

clean:
	rm -f *o $(EXECUTABLE)
