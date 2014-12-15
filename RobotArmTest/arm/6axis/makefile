CXX = g++ -Wall -g
CXXFLAGS = -Wall -O2 -DdDOUBLE -DdTRIMESH_ENABLED
TARGET = 6axis18
OBJS = $(SRCS:%.cpp=%.o)
SRCS = 6axis18.cpp

INCDIR = -I/usr/include/X11/ -I/usr/local/include/ode/ -I/usr/local/include/drawstuff/ -I/usr/include/ode
LIBDIR = -L/usr/lib -L/usr/lib/X11/ -L/usr/local/lib/
LIBS = -lode -ldrawstuff -lGL -lGLU -lXmu -lX11 -lstdc++ -lpthread -lm
LDFLAGS = $(INCDIR) $(LIBDIR) $(LIBS)
OPTS = -DdDOUBLE -DdTRIMESH_ENABLED #-framework GLUT #-framework OpenGL

.SUFFIXES: .c .cpp .o
all:$(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS)
.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCDIR) -c $<
clean:
	@rm -f *.o $(TARGET)
