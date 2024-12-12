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
/*
 * LADSPA bandpass filter
 * 
 * fc = center frequency
 * fs = sampling frequency
 * B = bandwidth
 * 
 *                        1 - R*z^-2
 * H(z) = (1-R)-----------------------------------
 *              1 - 2*R*cos(theta)*z^-1 + R^2*z^-2
 * 
 * theta = 2*pi*fc/fs
 * R = exp(-pi*B/fs)
 * G = 1-R
 */

#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

enum {
	PORT_IN,
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_BANDWIDTH,
	PORT_GAIN,
	PORT_LFO_FREQUENCY,
	PORT_LFO_AMOUNT,
	PORT_NPORTS
};

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
	LADSPA_Data m_lfo_theta;
} Bandpass_Data;

static LADSPA_Handle Bandpass_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	Bandpass_Data *l_pBandpass = malloc( sizeof(Bandpass_Data) );
	if( l_pBandpass ){
		l_pBandpass->m_sample_rate = p_sample_rate;
		l_pBandpass->m_z1 = 0.0;
		l_pBandpass->m_z2 = 0.0;
		l_pBandpass->m_lfo_theta = 0.0;
	}
	return (LADSPA_Handle)l_pBandpass;
}

static void Bandpass_connect_port(
	LADSPA_Handle p_pInstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata )
{
	Bandpass_Data *l_pBandpass = (Bandpass_Data*)p_pInstance;
	l_pBandpass->m_pport[p_port] = p_pdata;
}

static void Bandpass_run(
	LADSPA_Handle p_pInstance,
	unsigned long p_sample_count )
{
	Bandpass_Data *l_pBandpass = (Bandpass_Data*)p_pInstance;
	
	LADSPA_Data l_R = expf(-M_PI * *l_pBandpass->m_pport[PORT_BANDWIDTH] / l_pBandpass->m_sample_rate );
	LADSPA_Data l_G = 1 - l_R;
	l_G *= exp10f( *l_pBandpass->m_pport[PORT_GAIN] / 20 );
	LADSPA_Data l_a2 = l_R*l_R;
	LADSPA_Data l_lfo_dtheta = 2 * M_PI * *l_pBandpass->m_pport[PORT_LFO_FREQUENCY] / l_pBandpass->m_sample_rate;
	
	unsigned long l_sample;
	LADSPA_Data *l_psrc = l_pBandpass->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pBandpass->m_pport[PORT_OUT];
	
	for( l_sample = 0; l_sample < p_sample_count; l_sample++ ){
		LADSPA_Data l_frequency = *l_pBandpass->m_pport[PORT_FREQUENCY] + (0.5+0.5*sinf(l_pBandpass->m_lfo_theta)) * *l_pBandpass->m_pport[PORT_LFO_AMOUNT];
		LADSPA_Data l_theta = 2 * M_PI * l_frequency / l_pBandpass->m_sample_rate;
		LADSPA_Data l_a1 = -2 * l_R * cosf(l_theta);
		LADSPA_Data m = *l_psrc - l_a1*l_pBandpass->m_z1 - l_a2*l_pBandpass->m_z2;
		*l_pdst = l_G*(m - l_R*l_pBandpass->m_z2);
		l_pBandpass->m_z2 = l_pBandpass->m_z1;
		l_pBandpass->m_z1 = m;
		l_psrc++;
		l_pdst++;
		l_pBandpass->m_lfo_theta += l_lfo_dtheta;
		if( l_pBandpass->m_lfo_theta >= 2*M_PI )
			l_pBandpass->m_lfo_theta -= 2*M_PI;
	}
}

static void Bandpass_cleanup( LADSPA_Handle p_pInstance )
{
	free( p_pInstance );
}

static LADSPA_PortDescriptor Bandpass_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char* Bandpass_PortNames[]=
{
	"Input",
	"Output",
	"Frequency",
	"Bandwidth",
	"Gain",
	"LFO frequency",
	"LFO amount"
};

static LADSPA_PortRangeHint Bandpass_PortRangeHints[]=
{
	{0, 0, 0},
	{0, 0, 0},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE,
		10, 13000
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_LOW,
		10, 1000
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_0,
		-60,24
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.001,10
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_LOW,
		0,5000
	}
};

LADSPA_Descriptor LFOBandpass_Descriptor=
{
	5808,
	"lfo_bandpass",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Bandpass with LFO",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Bandpass_PortDescriptors,
	Bandpass_PortNames,
	Bandpass_PortRangeHints,
	NULL,
	Bandpass_instantiate,
	Bandpass_connect_port,
	NULL,
	Bandpass_run,
	NULL,
	NULL,
	NULL,
	Bandpass_cleanup
};
