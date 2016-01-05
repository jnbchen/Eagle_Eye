
TARGET = ../../bin/MotorInterface

CONFIG = app release warn_on

LIBS = boost_system iowkit boost_thread
LIBPATH = ../../libs/lib$(strip $(subst 32,,$(shell getconf LONG_BIT))) /usr/local/lib
INCLUDEPATH = ../../libs/include

SOURCES = \
../../Elementary/Timestamp.cpp \
../../Elementary/Angle.cpp \
../../Elementary/Vec.cpp \
../../Elementary/UDP/KogmolaborCommunication.cpp \
../../Elementary/UDP/MultiPacketUDPCommunication.cpp \
../../Elementary/UDP/NonspecificTaggedUDPCommunication.cpp \
../../Elementary/UDP/PriorityUDPCommunication.cpp \
../../Elementary/UDP/TaggedUDPCommunication.cpp \
../../Elementary/UDP/UDPSocket.cpp \
../../Vehicle/vehicle.cpp \
MotorInterface.cpp \
EPOS.cpp \
IOWarriorIIC.cpp \
