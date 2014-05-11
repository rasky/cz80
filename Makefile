CC=gcc
CXX=g++
CFLAGS=-O3 -g
CXXFLAGS=-std=c++11 -fomit-frame-pointer -fvisibility=hidden -save-temps -O3 -g 
LDFLAGS=-g

all: z80

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)
%.o: %.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS)

z80: z80.o Z80-Marat/Z80.o test.o
	$(CXX) -o $@ $^ $(LDFLAGS)
