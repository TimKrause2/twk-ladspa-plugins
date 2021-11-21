CFLAGS=-fPIC -O3

libfad.a:fad.o
	ar rs libfad.a fad.o
	
fad.o:fad.c fad_table.h

fad_table.h:table_gen.sce
	/home/tim/src/scilab-5.4.1/bin/scilab-cli -nwni -f table_gen.sce
	