
CC = g++
LIB_NAME = ../ADS/AdsLib-$(shell uname).a

all: getADSState.bin

getADSState.bin: $(LIB_NAME)
	$(CC) getADSState.cpp $< -std=c++11 -Wall -pedantic -lpthread -o $@

test: getADSState.bin
	./$<

clean:
	rm -f *.o *.bin
