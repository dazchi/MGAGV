#declare variables
CCP = g++
CC = gcc
CFLAGS = -I lib/ -I/usr/include/modbus -lmodbus -lwiringPi -pthread
BUILDDIR = build
BINDIR = build/bin

clib = $(wildcard lib/*.c)
cpplib = $(wildcard lib/*.cpp)
libobj = $(cpplib:%.cpp=$(BUILDDIR)/%.o) $(clib:%.c=$(BUILDDIR)/%.o)

all: navigation jsControl test

navigation: src/navigation.cpp $(libobj)
	$(CCP) $^ -o $(BINDIR)/$@ $(CFLAGS)

jsControl: src/jsControl.cpp $(libobj)
	$(CCP) $^ -o $(BINDIR)/$@ $(CFLAGS)

test: src/test.c $(libobj)
	$(CCP) $^ -o $(BINDIR)/$@ $(CFLAGS)

$(BUILDDIR)/%.o: %.cpp
	$(CCP) -c $< -o $@ $(CFLAGS)

$(BUILDDIR)/%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean: $(libobj)
	rm $^
	rm $(BINDIR)/*


 
