TARGET = $(HOME)/.qt4/designer/libLEDPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
LEDPlugin.h \
../LED.h \

SOURCES = \
LEDPlugin.cpp \
../LED.cpp \
