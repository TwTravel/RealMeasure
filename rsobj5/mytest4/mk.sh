gcc -DGPU -g  detector.cpp -I /usr/local/cuda/include -I ./include -o dark -L ./ -ldarknet  -lpthread

