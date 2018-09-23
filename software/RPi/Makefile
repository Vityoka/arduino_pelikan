
SRC = main.cpp 
SRC += ./RoboDNN/Activations.cpp ./RoboDNN/BLAS.cpp ./RoboDNN/ConvLayers.cpp ./RoboDNN/PoolLayers.cpp ./RoboDNN/RoboDNN.cpp ./RoboDNN/UtilityLayers.cpp ./RoboDNN/Utils.cpp ./comm.cpp
INC = -I./RoboDNN -I./inc

all:
	g++ $(shell pkg-config --libs --cflags opencv) -lwiringPi -o roadsignDetector $(SRC) $(INC)

