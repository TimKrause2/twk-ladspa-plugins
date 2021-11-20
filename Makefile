CFLAGS=-I fad -fPIC -O3

PLUGIN_SOURCES=compressor.c dcremove.c delay.c distortion.c impulsegen.c \
impulsegenvctl.c lfo_allpass.c lfo_bandpass.c lfo_bandpass5.c \
lfo_delay.c lfr_bandpass5.c lpvocoder.c phaser.c phaser2.c \
rbj_bandpass_bw.c rbj_bandpass.c rbj_highpass.c rbj_highpass12order.c \
rbj_highshelf.c rbj_lowpass.c rbj_lowpass12order.c rbj_lowshelf.c \
rbj_peakingEQ.c reverb20adjstereo.c

PLUGIN_OBJECTS=$(PLUGIN_SOURCES:.c=.o)

all:twk.so

twk.so:twk.o $(PLUGIN_OBJECTS)
	gcc -shared -o twk.so twk.o $(PLUGIN_OBJECTS) -lm -L fad -lfad
	
twk.o:twk.c

$(PLUGIN_OBJECTS):$(PLUGIN_SOURCES)

