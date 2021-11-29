CFLAGS=-fPIC -O3

libfad.a:fad.o
	ar rs libfad.a fad.o
	
fad.o:fad.c fad_table.h

fad_table.h:table_gen.sce
	scilab-cli -nwni -quit -f table_gen.sce
	
