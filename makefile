#declare variables
CCP = g++
CC = gcc
CFLAGS = -I lib/ -I/usr/include/modbus -lmodbus -lwiringPi -pthread
BUILDDIR = build
BINDIR = build/bin

clib = $(wildcard lib/*.c)
cpplib = $(wildcard lib/*.cpp)
libobj = $(cpplib:%.cpp=$(BUILDDIR)/%.o) $(clib:%.c=$(BUILDDIR)/%.o)

test = $(BINDIR)/test
jsControl = $(BINDIR)/jsControl


all: $(test) $(jsControl)


$(jsControl): src/jsControl.cpp $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)

$(test): src/test.c $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)


$(BUILDDIR)/%.o: %.cpp
	$(CCP) -c $< -o $@ $(CFLAGS)

$(BUILDDIR)/%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean: $(libobj)
	rm $^
	rm $(BINDIR)/*


 
