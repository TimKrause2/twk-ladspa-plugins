CFLAGS=-I fad -fPIC -O3

PLUGIN_SOURCES=compressor.c dcremove.c delay.c distortion.c impulsegen.c \
impulsegenvctl.c lfo_allpass.c lfo_bandpass.c lfo_bandpass5.c \
lfo_delay.c lfr_bandpass5.c lpvocoder.c phaser.c phaser2.c \
rbj_bandpass_bw.c rbj_bandpass.c rbj_highpass.c rbj_highpass12order.c \
rbj_highshelf.c rbj_lowpass.c rbj_lowpass12order.c rbj_lowshelf.c \
rbj_peakingEQ.c reverb20adjstereo.c sinewave.c bw_lp.c bw_hp.c \
bw_bp.c bw_bs.c elliptical_lp.c elliptical_hp.c elliptical_bp.c \
elliptical_bs.c pitch_shifter.c

PLUGIN_OBJECTS=$(PLUGIN_SOURCES:.c=.o)
PLUGIN_ASM=$(PLUGIN_SOURCES:.c=.s)

all:libfad twk.so

twk.so:twk.o $(PLUGIN_OBJECTS) fad/libfad.a
	gcc -shared -o twk.so twk.o $(PLUGIN_OBJECTS) -lm -L fad -lfad
	
twk.o:twk.c $(PLUGIN_SOURCES)

$(PLUGIN_OBJECTS):$(PLUGIN_SOURCES)

%.s:%.c
	gcc -S $(CFLAGS) $< -o $@

$(PLUIGIN_ASM):$(PLUGIN_SOURCES)

asm:$(PLUGIN_ASM)

butt_formulas:
	maxima -b "butterworth_stages.mac"
	
ellip_formulas:
	maxima -b "elliptical_biquad_stage.mac"
	
libfad:
	make -C fad
	
