
TARGET = ../../bin/MotorInterfaceOldCar

CONFIG = app release warn_on

INCLUDEPATH = ../../libs/include

LIBS = iowkit boost_thread boost_system
LIBPATH = ../../libs/lib$(strip $(subst 32,,$(shell getconf LONG_BIT))) /usr/local/lib

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
MotorInterfaceOldCar.cpp \
../MotorInterface/IOWarriorIIC.cpp \
