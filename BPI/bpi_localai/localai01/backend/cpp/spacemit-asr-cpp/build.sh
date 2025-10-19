if [ -d "build" ]; then
    rm -rf build
fi

mkdir build
cd build

cmake ..
make -j8

cp ./bin/spacemit-asr-cpp ../grpc-server

cd ..
tg1=../../../backend-assets/grpc/
if [ ! -d "$tg1" ]; then
    mkdir -p $tg1
fi
cp ./grpc-server $tg1/spacemit-asr-cpp

tg2=/tmp/localai/backend_data/backend-assets/grpc
if [ ! -d "$tg2" ]; then
    mkdir -p $tg2 
fi
cp ./grpc-server $tg2/spacemit-asr-cpp

cat <<EOF > ../../../models/spacemit-asr-cpp.yaml
backend: spacemit-asr-cpp
name: sensevoicesmall-cpp
EOF