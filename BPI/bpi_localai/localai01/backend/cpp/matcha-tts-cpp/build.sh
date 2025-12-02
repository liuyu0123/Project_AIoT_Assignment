if [ -d "build" ]; then
    rm -rf build
fi

mkdir build
cd build

cmake ..
make -j8

cp ./bin/grpc-server ../grpc-server

cd ..
tg1=../../../backend-assets/grpc/
if [ ! -d "$tg1" ]; then
    mkdir -p $tg1
fi
cp ./grpc-server $tg1/matcha-tts-cpp

tg2=/tmp/localai/backend_data/backend-assets/grpc
if [ ! -d "$tg2" ]; then
    mkdir -p $tg2
fi
cp ./grpc-server $tg2/matcha-tts-cpp

cat <<EOF > ../../../models/matcha-tts-cpp.yaml
backend: matcha-tts-cpp
name: matcha-tts-cpp
EOF