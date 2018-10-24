g++ -DGPU -I./  -I /usr/local/include/opencv2  -I /usr/local/cuda/include  main1.cpp -o main `pkg-config --libs opencv` -lstdc++ -L ./ -ldarknet -lpthread
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
