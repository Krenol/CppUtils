ARG IMG_VERSION=20200516

FROM krenol/cpp_raspbian:${IMG_VERSION}

ARG PROJECT_DIR=utils

ARG BUILD_TEST=ON
ENV BUILD_TEST "$BUILD_TEST"

WORKDIR /data

RUN apt-get install -y libeigen3-dev

# copy files
COPY CMakeLists.txt ./
COPY ./${PROJECT_DIR} ./${PROJECT_DIR}

#prepare build
RUN mkdir build && cd build && cmake ../${PROJECT_DIR} 

#build
RUN cmake --build ./build

COPY ./docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["start"]