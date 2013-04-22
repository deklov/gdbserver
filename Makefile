CXX = g++
CXXFLAGS = -g -O0

test_a32: test_a32.cc gdbserver.cc

clean:
	$(RM) test_a32
