FROM gazebo
SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y cmake g++ libboost-all-dev  build-essential
COPY . /app/
RUN mkdir build 
WORKDIR /app/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build .

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/app/entrypoint.sh"]
