### TARGET: file name of target file (if not given 'project file name without .pro' is used)
TARGET = ../../bin/AnicarViewer


### CONFIG:
###           qt - QT-application
###          x11 - X11-application
###       thread - thread support
###      warn_on - report compiler warnings (default)
###     warn_off - do not report compiler warnings
###      release - use optimized compiling
###        debug - compile with debugger-tags
###      profile - compile with profiler-tags
###          app - generate a binary (default)
###   static_lib - generate a static library
###   shared_lib - generate a shared library
CONFIG = warn_on app release qt


### DEFS: additional define settings for the compiler (without initial -D)
DEFS = 


### LIBS: non-standard libraries (without initial -l)
LIBS = boost_thread boost_system


### LIBPATH: non-standard library pathes (without initial -L)
LIBPATH = /usr/local/lib


### INCLUDEPATH: non-standard include pathes (without initial -I)
INCLUDEPATH = 


### FORMS: QT-files generated using the QT-designer (.ui-files)
QTFORMS = \
Ui_DialogModuleSelection.ui \
Ui_AnicarViewer.ui \


### QTHEADERS: list of header files that contain QT-macros
QTHEADERS = \
DialogModuleSelection.h \
MapWidget.h \
AnicarViewer.h \
LED.h \


### RESOURCES: QT-Ressource-Datei (maximal eine)
RESOURCES = AnicarViewer.qrc


### HEADERS: list of header files that should be copied to directory HEADERDEST
HEADERS = 


### HEADERDEST: directory to which HEADERS should be copied
HEADERDEST = 


### SOURCES: list of source files (.cpp, .cc, .c, .cxx, .C)
SOURCES = \
DialogModuleSelection.cpp \
main.cpp \
MapWidget.cpp \
AnicarViewer.cpp \
LED.cpp \
Joystick.cpp \
../../Elementary/Vec.cpp \
../../Elementary/Angle.cpp \
../../Elementary/Timestamp.cpp \
../../Elementary/ConfigReader.cpp \
../../Elementary/UDP/KogmolaborCommunication.cpp \
../../Elementary/UDP/MultiPacketUDPCommunication.cpp \
../../Elementary/UDP/NonspecificTaggedUDPCommunication.cpp \
../../Elementary/UDP/PriorityUDPCommunication.cpp \
../../Elementary/UDP/TaggedUDPCommunication.cpp \
../../Elementary/UDP/UDPSocket.cpp \
../../Vehicle/vehicle.cpp \
