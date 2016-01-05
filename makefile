
PROJECT = DerWeg.cbp
TARGET = bin/DerWeg

ISNOCAMERA = $(filter nocamera,$(MAKECMDGOALS))
ISNOSTEREO = $(filter nostereo nocamera,$(MAKECMDGOALS))
LB3264 = $(shell getconf LONG_BIT)
LB64 = $(strip $(subst 32,,$(LB3264)))

CXX = g++
LC = g++

CXXFLAGS = -O2 -g -W -Wall -std=c++0x #-march=native
INCLUDEPATH = -I libs/include -I/usr/include -I/usr/include/eigen3
LIBPATH = -L libs/lib$(LB64) -Wl,-rpath=../libs/lib$(LB64) -L/usr/ĺocal/lib 
LIBS = -l boost_system -l boost_thread -l boost_date_time -l pthread -l CalibIO # -l rt # -l boost_chrono

EXCLUDESOURCES = 

CUDACXX     = gcc-4.6 # use gcc-4.4 for CUDA 4.0, gcc-4.5 for CUDA 4.1
CUDAPATH    = /usr/local/cuda
CUDASDKPATH = /usr/local/cuda/samples #$(HOME)/NVIDIA_GPU_Computing_SDK /home/common/NVIDIA_GPU_Computing_SDK /usr/local/NVIDIA_GPU_Computing_SDK
ifeq ($(ISNOSTEREO),)
  ifeq ($(strip $(wildcard $(CUDAPATH)/bin/nvcc)),)
    ISNOSTEREO = nostereo
  endif
else
  ISNOSTEREO = nostereo
endif

ifeq ($(ISNOSTEREO),)
   INCLUDEPATH += -I$(CUDAPATH)/include $(foreach cudapath,$(CUDASDKPATH),-I$(cudapath)/C/common/inc )
   LIBPATH     += -L$(CUDAPATH)/lib$(LB64)
   LIBS        += -lToast2 -lHawaiiSIMD -lHawaii -lcudart 
   CXXFLAGS    += -DKOGMO_CUDA
   CXXFLAGS    += -Dnullptr=0 -fopenmp
else
   EXCLUDESOURCES += Application/ShowStereoCameraImage.cpp Application/MaschinenbautagDemo.cpp
endif

LIBS += -l opencv_calib3d -l opencv_contrib -l opencv_core -l opencv_features2d -l opencv_flann -l opencv_gpu -l opencv_highgui -l opencv_imgproc -l opencv_legacy -l opencv_ml -l opencv_objdetect -l opencv_video 

ifeq ($(ISNOCAMERA),)
   LIBS += -l PvAPI
else
   EXCLUDESOURCES += ImageSource/GigECameraDriver.cpp ImageSource/ImageSourceGigE.cpp ImageSource/GigEStereoCameraDriver.cpp ImageSource/StereoImageSourceGigE.cpp ImageSource/GigE.cpp
endif

SUFFIXES = .c .cc .cpp .cxx .C
SOURCES = $(filter-out $(EXCLUDESOURCES), $(foreach suf,$(SUFFIXES),$(filter %$(suf),$(strip $(subst ">,,$(subst " />,,$(subst <Unit filename=",,$(shell grep "Unit filename" $(PROJECT)))))))))
OBJS = $(filter %.o,$(foreach suf,$(SUFFIXES),$(SOURCES:$(suf)=$(suf).o)))
DEPS = $(filter %.d,$(foreach suf,$(SUFFIXES),$(SOURCES:$(suf)=$(suf).d)))



# Goals:
DerWeg : $(TARGET)
$(TARGET) : $(OBJS) $(DEPS)
	@echo "making $(TARGET) in $(LB3264)bit mode"
	@$(LC) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBPATH) $(LIBS)

link: $(OBJS)
	@echo "linking $(TARGET) in $(LB3264)bit mode"
	@$(LC) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LIBPATH) $(LIBS)
 
%.cc.d : %.cc
	@echo "making dependencies for $<"
	@$(CXX) -M -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(INCLUDEPATH) -MF $@ $<

%.cpp.d : %.cpp
	@echo "making dependencies for $<"
	@$(CXX) -M -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(INCLUDEPATH) -MF $@ $<

%.cxx.d : %.cxx
	@echo "making dependencies for $<"
	@$(CXX) -M -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(INCLUDEPATH) -MF $@ $<

%.C.d : %.C
	@echo "making dependencies for $<"
	@$(CXX) -M -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(INCLUDEPATH) -MF $@ $<

%.c.d : %.c
	@echo "making dependencies for $<"
	@$(CXX) -M -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(INCLUDEPATH) -MF $@ $<

%.cc.o : %.cc
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(INCLUDEPATH) -c -o $@ $<

%.c.o : %.c
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(INCLUDEPATH) -c -o $@ $<

%.cpp.o : %.cpp
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(INCLUDEPATH) -c -o $@ $<

%.cxx.o : %.cxx
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(INCLUDEPATH) -c -o $@ $<

%.C.o : %.C
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(INCLUDEPATH) -c -o $@ $<

%.cu.o : %.cu
	@echo "compiling $<"
	@$(CUDAPATH)/bin/nvcc -O3 --use_fast_math --compiler-bindir=$(CUDACXX) -gencode=arch=compute_20,code=\"sm_21,compute_20\" --compiler-options="$(CXXFLAGS)" $(INCLUDEPATH) -c -o $@ $<

clean :
	@echo "cleaning files"
	@rm -f $(OBJS) $(DEPS) $(TARGET)

clean.d:
	@echo "cleaning dependencies"
	@rm -f $(DEPS)

clean.o:
	@echo "cleaning object files"
	@rm -f $(OBJS)

distclean :
	@echo "cleaning files"
	@rm -f $(OBJS) $(DEPS) $(TARGET)

install : DerWeg
	make -C tools/MotorInterface
	make -C tools/AnicarViewer

nostereo : $(TARGET)
	@echo "nostereo mode"

nocamera : $(TARGET)
	@echo "nocamera mode"

show.flags:
	@echo $(CXXFLAGS) $(INCLUDEPATH) $(LIBPATH) $(LIBS)

show.sources:
	@echo $(SOURCES)

Debug : DerWeg

help:
	@echo "Makefile for DerWeg"
	@echo "make [show.flags] [show.sources] [nocamera] [nostereo] [oldcv] DerWeg | clean"
	@echo "  Builds project DerWeg."
	@echo "  Automatically links 32bit/64bit libraries."
	@echo "  If cuda is not available on the computer or option 'nostereo' is chosen"
	@echo "  the project is compiled without stereo vision support."
	@echo "  If option 'nocamera' is chosen the project is compiled without camera"
	@echo "  driver support. This includes 'nostereo'."
	@echo "  If the option 'oldcv' is chosen it links the old opencv libraries"
	@echo "  libcv.a etc. instead of the new libraries libopencv_core.so etc."
	@echo "  Option 'show.flags' displays the compiler and linker flags,"
	@echo "  'show.sources' display the list of source files."

# dependencies:

ifeq (,$(filter clean help, $(MAKECMDGOALS)))
  -include $(DEPS)
endif
