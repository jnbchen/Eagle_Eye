# compiled with gcc version 5.2.1 on ubuntu

CXX = g++
CXXFLAGS = -Wall -O3
LDFLAGS = -Wall
LDLIBS = -lboost_filesystem -lboost_system

SOURCES = svgreader.cpp tinyxml2.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = svgreader

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@ $(LDLIBS)

clean:
	rm -f *o $(EXECUTABLE)
