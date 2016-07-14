CXX = g++
CXXFLAGS = -Wall -O3
LDFLAGS = -Wall
LDLIBS = -lopencv_core -lopencv_flann -lopencv_highgui -lopencv_imgproc

SOURCES = thresholding_values.cpp 
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = thresholding_values

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@ $(LDLIBS)

clean:
	rm -f *o $(EXECUTABLE)
