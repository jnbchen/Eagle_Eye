
TARGET = ../../bin/offlineStarGazerCalibration

CONFIG = app warn_on release

LIBS = boost_thread boost_system

SOURCES = \
../../Localization/StarGazerCalibration.cpp \
../../Localization/StarGazerLandmarkList.cpp \
../../Elementary/Vec.cpp \
../../Elementary/Angle.cpp \
../../Elementary/ConfigReader.cpp \
../../Elementary/Gnuplot.cpp \
offlineStarGazerCalibration.cpp \
