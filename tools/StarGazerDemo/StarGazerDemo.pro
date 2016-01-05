### TARGET: file name of target file (if not given 'project file name without .pro' is used)
TARGET = ../../bin/StarGazerDemo


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
CONFIG = warn_on app release


### DEFS: additional define settings for the compiler (without initial -D)
DEFS = 


### LIBS: non-standard libraries (without initial -l)
LIBS = boost_thread


### SOURCES: list of source files (.cpp, .cc, .c, .cxx, .C)
SOURCES = \
demo.cpp \
serial_port.cpp \
StarGazer.cpp \
