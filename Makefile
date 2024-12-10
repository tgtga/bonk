PREFIX   := /usr
CXX      := g++
CXXFLAGS := -O3 -funroll-loops
INSTALL  := install -c

# Uncomment this line for Linux and FreeBSD
LIBS=

# Uncomment this line for NetBSD and OpenBSD
#LIBS=		-lossaudio

all: bonk

bonk: bonk.cc utility.h wav.h
	$(CXX) $(CXXFLAGS) -o bonk bonk.cc $(LIBS)

install: bonk
	$(INSTALL) bonk $(PREFIX)/bin

clean:
	rm -f core bonk
