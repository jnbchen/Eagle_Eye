CXX = g++
CXXFLAGS = -Wall -O3
LDFLAGS = -Wall
LDLIBS = -lopencv_core -lopencv_flann -lopencv_highgui -lopencv_imgproc

SOURCES = im_test.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = im_test

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@ $(LDLIBS)

clean:
	rm -f *o $(EXECUTABLE)
