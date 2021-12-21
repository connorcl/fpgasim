all:
	ghdl-llvm --vpi-compile gcc -W -Wall -O2 -c -o fpgasim.o fpgasim.c
	ghdl-llvm --vpi-link gcc -W -Wall -O2 fpgasim.o -o fpgasim.vpi -lm -lrt -lpthread

clean:
	rm *.o *.vpi