USE_RASPIKE_ART=1

mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

CXXFLAGS += -std=c++17

APPL_COBJS +=\
LineTracer.o\

#camera_thread.o\
#imgsend_thread.o\

APPL_CXXOBJS += \
StreamCaptureRGB.o\
StreamCaptureWrapper.o

#camera_logic.o\
#imgsend_logic.o\

APPL_LIBS += -lm -lstdc++ -lopencv_core -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -lcamera -lcamera-base

APPL_DIRS += $(mkfile_path)LineTracer\

INCLUDES += \
	-I$(mkfile_path)LineTracer \
	-I/usr/include/opencv4 \
	-I/usr/include/libcamera \
    -I/usr/include/c++/12 \
	-I/usr/include/c++/12/aarch64-linux-gnu
