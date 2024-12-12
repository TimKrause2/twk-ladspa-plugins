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
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

enum {
	PORT_IN,
	PORT_OUT,
	PORT_WAVESHAPE,
	PORT_PRE_GAIN,
	PORT_POST_GAIN,
	PORT_NPORTS
};

static LADSPA_PortDescriptor Distortion_PortDescriptors[]=
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* Distortion_PortNames[]=
{
	"Input",
	"Output",
    "Waveshape(nth root)",
    "PreGain(dB)",
	"PostGain(dB)"
};

static LADSPA_PortRangeHint Distortion_PortRangeHints[]=
{
    {0, 0.0f, 0.0f},
    {0, 0.0f, 0.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_LOW,
        1.0f, 20.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
        LADSPA_HINT_DEFAULT_MIDDLE,
        -96.0f, 96.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
        LADSPA_HINT_DEFAULT_MIDDLE,
        -48.0f, 48.0f}
};

typedef struct 
{
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
} Distortion;

static LADSPA_Handle Distortion_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	Distortion *p_pDistortion = malloc( sizeof(Distortion) );
	if(!p_pDistortion)
		return NULL;
	p_pDistortion->m_sample_rate = p_sample_rate;
	return (LADSPA_Handle)p_pDistortion;
}

static void Distortion_connect_port(
	LADSPA_Handle p_pinstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata )
{
	Distortion *p_pDistortion = (Distortion*)p_pinstance;
	p_pDistortion->m_pport[p_port] = p_pdata;
}

static void Distortion_run(
	LADSPA_Handle p_pinstance,
	unsigned long p_sample_count )
{
	Distortion *p_pDistortion = (Distortion*)p_pinstance;
	
	float l_pre_gain = exp10f( *p_pDistortion->m_pport[PORT_PRE_GAIN] / 20.0 );
	float l_post_gain = exp10f( *p_pDistortion->m_pport[PORT_POST_GAIN] / 20.0 );
	float l_waveshape = *p_pDistortion->m_pport[PORT_WAVESHAPE];

	long sample;
	LADSPA_Data *l_psrc = p_pDistortion->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = p_pDistortion->m_pport[PORT_OUT];
	
	for(sample=p_sample_count;sample!=0;sample--){
		float l_sign;
		float l_data_in;
		if( *l_psrc < 0.0 ){
			l_data_in = -*l_psrc++;
			l_sign = -1.0;
		}else{
			l_sign = 1.0;
			l_data_in = *l_psrc++;
		}

		*l_pdst++ = powf(l_pre_gain*l_data_in,1.0f/l_waveshape)*l_post_gain*l_sign;
	}
}

static void Distortion_cleanup(
	LADSPA_Handle p_pinstance )
{
	free( p_pinstance );
}

LADSPA_Descriptor Distortion_Descriptor=
{
	5804,
	"distortion_Nth_root",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Distortion Nth root",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Distortion_PortDescriptors,
	Distortion_PortNames,
	Distortion_PortRangeHints,
	NULL,
	Distortion_instantiate,
	Distortion_connect_port,
	NULL,
	Distortion_run,
	NULL,
	NULL,
	NULL,
	Distortion_cleanup
};
