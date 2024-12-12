/*

twk.so is a set of LADSPA plugins.

Copyright 2024 Tim Krause

This file is part of twk.so.

twk.so is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published
by the Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

twk.so is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with twk.so. If not, see
<https://www.gnu.org/licenses/>.

Contact: tim.krause@twkrause.ca

*/
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
#include "bw_lp.h"
#include "bw_hp.h"
#include "bw_bp.h"
#include "bw_bs.h"
#include "elliptical_lp.h"
#include "elliptical_hp.h"
#include "elliptical_bp.h"
#include "elliptical_bs.h"
#include "pitch_shifter.h"

const LADSPA_Descriptor *dTable[]=
{
    &Compressor_Descriptor,        // 5801
    &DCRemove_Descriptor,          // 5802
    &Delay_Descriptor,             // 5803
    &Distortion_Descriptor,        // 5804
    &ImpulseGen_Descriptor,        // 5805
    &ImpulseGenVC_Descriptor,      // 5806
    &LFOAllPass_Descriptor,        // 5807
    &LFOBandpass_Descriptor,       // 5808
    &LFOBandpass5_Descriptor,      // 5809
    &LFODelay_Descriptor,          // 5810
    &LFRBandpass5_Descriptor,      // 5811
    &LPVocoder_Descriptor,         // 5812
    &Phaser_Descriptor,            // 5813
    &Phaser2_Descriptor,           // 5814
    &RBJBandpassBW_Descriptor,     // 5815
    &RBJBandpassQ_Descriptor,      // 5816
    &RBJHighpassQ_Descriptor,      // 5817
    &RBJHighpassQ12_Descriptor,    // 5818
    &RBJHighShelf_Descriptor,      // 5819
    &RBJLowpassQ_Descriptor,       // 5820
    &RBJLowpassQ12_Descriptor,     // 5821
    &RBJLowShelf_Descriptor,       // 5822
    &RBJPeakingEQ_Descriptor,      // 5823
    &Reverb20AdjStereo_Descriptor, // 5824
    &SineWave_Descriptor,          // 5825
    &BW_LP_Descriptor,             // 5826
    &BW_HP_Descriptor,             // 5827
    &BW_BP_Descriptor,             // 5828
    &BW_BS_Descriptor,             // 5829
    &Ellip_LP_Descriptor,          // 5830
    &Ellip_HP_Descriptor,          // 5831
    &Ellip_BP_Descriptor,          // 5832
    &Ellip_BS_Descriptor,          // 5833
    &PShift_Descriptor,            // 5834
    NULL
};

const LADSPA_Descriptor *ladspa_descriptor(unsigned long Index)
{
    return dTable[Index];
}
