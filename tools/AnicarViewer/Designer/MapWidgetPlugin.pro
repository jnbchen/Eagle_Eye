TARGET = $(HOME)/.qt4/designer/libMapWidgetPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
MapWidgetPlugin.h \
../MapWidget.h \

SOURCES = \
MapWidgetPlugin.cpp \
../MapWidget.cpp \
../../../Elementary/Vec.cpp \
../../../Elementary/Angle.cpp \
