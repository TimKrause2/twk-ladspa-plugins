#include <ladspa.h>
#include <stddef.h>
#include "compressor.h"
#include "dcremove.h"
#include "delay.h"
#include "distortion.h"
#include "impulsegen.h"
#include "impulsegenvctl.h"
#include "lfo_allpass.h"
#include "lfo_bandpass.h"
#include "lfo_bandpass5.h"
#include "lfo_delay.h"
#include "lfr_bandpass5.h"
#include "lpvocoder.h"
#include "phaser.h"
#include "phaser2.h"
#include "rbj_bandpass_bw.h"
#include "rbj_bandpass.h"
#include "rbj_highpass.h"
#include "rbj_highpass12order.h"
#include "rbj_highshelf.h"
#include "rbj_lowpass.h"
#include "rbj_lowpass12order.h"
#include "rbj_lowshelf.h"
#include "rbj_peakingEQ.h"
#include "reverb20adjstereo.h"
#include "sinewave.h"

#define N_PLUGINS 25

const LADSPA_Descriptor *dTable[N_PLUGINS]=
{
	&Compressor_Descriptor,
	&DCRemove_Descriptor,
	&Delay_Descriptor,
	&Distortion_Descriptor,
	&ImpulseGen_Descriptor,
	&ImpulseGenVC_Descriptor,
	&LFOAllPass_Descriptor,
	&LFOBandpass_Descriptor,
	&LFOBandpass5_Descriptor,
	&LFODelay_Descriptor,
	&LFRBandpass5_Descriptor,
	&LPVocoder_Descriptor,
	&Phaser_Descriptor,
	&Phaser2_Descriptor,
	&RBJBandpassBW_Descriptor,
	&RBJBandpassQ_Descriptor,
	&RBJHighpassQ_Descriptor,
	&RBJHighpassQ12_Descriptor,
	&RBJHighShelf_Descriptor,
	&RBJLowpassQ_Descriptor,
	&RBJLowpassQ12_Descriptor,
	&RBJLowShelf_Descriptor,
	&RBJPeakingEQ_Descriptor,
    &Reverb20AdjStereo_Descriptor,
    &SineWave_Descriptor
};

const LADSPA_Descriptor *ladspa_descriptor(unsigned long Index)
{
	if(Index<N_PLUGINS){
		return dTable[Index];
	}else{
		return NULL;
	}
}
