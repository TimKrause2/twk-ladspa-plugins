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

/*			(b0/a0) + (b1/a0)*z^-1 + (b2/a0)*z^-2
 *   H(z) = ---------------------------------------
 *              1 + (a1/a0)*z^-1 + (a2/a0)*z^-2
 * 
 * b0 = (1 - cos(omega))/2
 * b1 =  1 - cos(omega)
 * b2 = (1 - cos(omega))/2
 * a0 =  1 + alpha
 * a1 = -2 cos(omega)
 * a2 =  1 - alpha
 * 
 * omega = 2 * pi * fc / fs
 * alpha = sin(omega) * sinh( log(2)/2 * bw * omega / sin(omega) )
 *       = sin(omega) / (2*Q)
 */

enum {
	PORT_IN,
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_Q,
	PORT_GAIN,
	PORT_NPORTS
};

typedef struct {
    LADSPA_Data m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
	LADSPA_Data m_log2d2;
} Lowpass_Data;

static LADSPA_Handle Lowpass_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate ){
	Lowpass_Data *l_pLowpass = malloc( sizeof(Lowpass_Data) );
	if( l_pLowpass ){
        l_pLowpass->m_sample_rate = (float)p_sample_rate;
		l_pLowpass->m_z1 = 0.0;
		l_pLowpass->m_z2 = 0.0;
		l_pLowpass->m_log2d2 = logf(2.0)/2.0;
	}
	return (LADSPA_Handle)l_pLowpass;
}

static void Lowpass_connect_port(
	LADSPA_Handle p_pInstance,
	unsigned long p_port,
	LADSPA_Data *p_pdata)
{
	Lowpass_Data *l_pLowpass = (Lowpass_Data*)p_pInstance;
	l_pLowpass->m_pport[p_port] = p_pdata;
}

static void Lowpass_run(
	LADSPA_Handle p_pInstance,
	unsigned long p_sample_count )
{
	Lowpass_Data *l_pLowpass = (Lowpass_Data*)p_pInstance;
	LADSPA_Data *l_psrc = l_pLowpass->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pLowpass->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;
	
    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pLowpass->m_pport[PORT_FREQUENCY] /
	l_pLowpass->m_sample_rate;
	LADSPA_Data l_sin_omega = sinf( l_omega );
	LADSPA_Data l_cos_omega = cosf( l_omega );
    LADSPA_Data l_alpha = l_sin_omega /( 2.0f * *l_pLowpass->m_pport[PORT_Q]);
    LADSPA_Data l_a0 = 1.0f + l_alpha;
    register LADSPA_Data l_a1 = -2.0f * l_cos_omega / l_a0;
    register LADSPA_Data l_a2 = (1.0f - l_alpha) / l_a0;
    register LADSPA_Data l_b0 = (1.0f - l_cos_omega) / 2.0f / l_a0;
    register LADSPA_Data l_b1 = (1.0f - l_cos_omega) / l_a0;
    register LADSPA_Data l_b2 = (1.0f - l_cos_omega) / 2.0f / l_a0;
    register LADSPA_Data l_G = exp10f( *l_pLowpass->m_pport[PORT_GAIN] / 20.0f );
    for(;l_psrc!=l_psrc_end;l_psrc++, l_pdst++){
        register LADSPA_Data l_m = *l_psrc - l_a1*l_pLowpass->m_z1 - l_a2*l_pLowpass->m_z2;
		*l_pdst = l_G*(l_m*l_b0 + l_pLowpass->m_z1*l_b1 + l_pLowpass->m_z2*l_b2);
		l_pLowpass->m_z2 = l_pLowpass->m_z1;
		l_pLowpass->m_z1 = l_m;
	}
}

static void Lowpass_cleanup( LADSPA_Handle p_pInstance )
{
	free( p_pInstance );
}

static LADSPA_PortDescriptor Lowpass_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char *Lowpass_PortNames[]=
{
	"Input",
	"Output",
	"Frequency",
	"Q",
	"Gain"
};

static LADSPA_PortRangeHint Lowpass_PortRangeHints[]=
{
	{0,0,0},
	{0,0,0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
		10,13000},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE,
		0.707,100},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_0,
		-60,24}
};

LADSPA_Descriptor RBJLowpassQ_Descriptor=
{
	5820,
	"RBJ_lowpass_Q",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Lowpass RBJ(Q)",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Lowpass_PortDescriptors,
	Lowpass_PortNames,
	Lowpass_PortRangeHints,
	NULL,
	Lowpass_instantiate,
	Lowpass_connect_port,
	NULL,
	Lowpass_run,
	NULL,
	NULL,
	NULL,
	Lowpass_cleanup
};
