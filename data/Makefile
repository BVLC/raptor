.PHONY: clean all

BUILD_TYPE?=Release
CMAKE=cmake
CMAKE_ARGS=-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
           -DCMAKE_INSTALL_PREFIX=`rospack find data`

all:
	bash -c 'CORRECT_PATH=`rospack find ffld_ros`/ffld; for x in `find ../data_original -type f`; do   DEST=`basename $$x`; cp $$x $$DEST; sed -i "s/\/home\/goehring\/ros_workspace\/ffld/$(CORRECT_PATH)/g" $$DEST; done'

clean:
	bash -c 'for x in `find ../data_original -type f`; do DEST=`basename $$x`; rm -f $$DEST; done'
