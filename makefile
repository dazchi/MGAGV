#declare variables
CCP = g++
CC = gcc
CFLAGS = -I lib/ -I/usr/include/modbus -lmodbus -lwiringPi -pthread
BUILDDIR = build
BINDIR = build/bin

clib = $(wildcard lib/*.c)
cpplib = $(wildcard lib/*.cpp)
libobj = $(cpplib:%.cpp=$(BUILDDIR)/%.o) $(clib:%.c=$(BUILDDIR)/%.o)

all: $(BINDIR)/navigation $(BINDIR)/jsControl $(BINDIR)/test $(BINDIR)/qrLocalization $(BINDIR)/tcpControl $(BINDIR)/fullNavigation

$(BINDIR)/navigation: src/navigation.cpp $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)

$(BINDIR)/jsControl: src/jsControl.cpp $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)

$(BINDIR)/test: src/test.c $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)

$(BINDIR)/qrLocalization: src/qrLocalization.cpp $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)

$(BINDIR)/tcpControl: src/tcpControl.cpp $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)

$(BINDIR)/fullNavigation: src/fullNavigation.cpp $(libobj)
	$(CCP) $^ -o $@ $(CFLAGS)

$(BUILDDIR)/%.o: %.cpp
	$(CCP) -c $< -o $@ $(CFLAGS)

$(BUILDDIR)/%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean: $(libobj)
	rm $^
	rm $(BINDIR)/*


 
