
CC = g++
CFLAGS = -larmadillo -lmat -lmx
CNAME = main.cpp
EXECNAME = main
CPPFLAGS = -I//usr/local/MATLAB/MATLAB_Production_Server/R2015a/extern/include/
LDFLAGS = -L/usr/local/MATLAB/MATLAB_Production_Server/R2015a/bin/glnxa64

runn:
	$(CC) $(CPPFLAGS) $(LDFLAGS) -O3 -o $(EXECNAME) $(CNAME) $(CFLAGS)

clean:
	rm $(EXECNAME)
