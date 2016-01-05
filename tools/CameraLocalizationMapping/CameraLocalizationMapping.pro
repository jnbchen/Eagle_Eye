
TARGET = ../../bin/cameraLocalizationMapping

CONFIG = app warn_on release

LIBS = boost_thread boost_system

LIBPATH = /usr/local/lib

SOURCES = \
CameraLocalizationMapping.cpp \
main.cpp \
../../Elementary/Vec.cpp \
../../Elementary/Angle.cpp \
../../Elementary/ConfigReader.cpp \
../../Elementary/Gnuplot.cpp \
