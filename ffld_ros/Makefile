.PHONY: clean

BUILD_TYPE?=Release
CMAKE=cmake
CMAKE_ARGS=-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
           -DCMAKE_INSTALL_PREFIX=`rospack find ffld_ros`/ffld

all:
	mkdir -p ffld/build
	cd ffld/build && cmake .. && make
	mkdir -p lib
	mkdir -p bin
	mkdir -p include
	cp ffld/build/libFFLD.a lib/
	cp ffld/build/detect bin/
	cp ffld/build/detect_opencv bin/
	cp ffld/build/ffld bin/
	cp ffld/*.h include/

clean:
	rm -rf ffld/build bin lib
