CXX=arm-raspbian10-linux-gnueabihf-g++
DEPS_CFLAGS=-Iinclude -Iinclude/opencv -Iinclude
DEPS_LIBS=-Llib -lwpilibc -lwpiHal -lcameraserver -lntcore -lcscore -lopencv_dnn -lopencv_highgui -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -lopencv_features2d -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -lwpiutil -latomic
EXE=high_goal_vision

.PHONY: clean build

build: ${EXE}

clean:
	rm ${EXE} *.o

OBJS=cs_main.o high_goal.o

${EXE}: ${OBJS}
	${CXX} -pthread -g -o $@ $^ ${DEPS_LIBS} -Wl,--unresolved-symbols=ignore-in-shared-libs

.cpp.o:
	${CXX} -pthread -g -Og -c -o $@ -std=c++17 ${CXXFLAGS} ${DEPS_CFLAGS} $<
