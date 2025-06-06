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
 * b0 = 1 + alpha*A
 * b1 = -2 cos(omega)
 * b2 = 1 - alpha*A
 * a0 = 1 + alpha/A
 * a1 = -2 cos(omega)
 * a2 = 1 - alpha/A
 * 
 * A = 10^(dBgain/40)
 * omega = 2 * pi * fc / fs
 * alpha = sin(omega) * sinh( log(2)/2 * bw * omega / sin(omega) )
 *       = sin(omega) / (2*Q)
 */
enum {
	PORT_IN,
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_BANDWIDTH,
	PORT_GAIN,
	PORT_NPORTS
};

typedef struct {
    LADSPA_Data m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
	LADSPA_Data m_log2d2;
} PeakingEQ_Data;

static LADSPA_Handle PeakingEQ_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate ){
	PeakingEQ_Data *l_pPeakingEQ = malloc( sizeof(PeakingEQ_Data) );
	if( l_pPeakingEQ ){
        l_pPeakingEQ->m_sample_rate = (float)p_sample_rate;
		l_pPeakingEQ->m_z1 = 0.0;
		l_pPeakingEQ->m_z2 = 0.0;
        l_pPeakingEQ->m_log2d2 = logf(2.0f)/2.0f;
	}
	return (LADSPA_Handle)l_pPeakingEQ;
}

static void PeakingEQ_connect_port(
	LADSPA_Handle p_pInstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata)
{
	PeakingEQ_Data *l_pPeakingEQ = (PeakingEQ_Data*)p_pInstance;
	l_pPeakingEQ->m_pport[p_port] = p_pdata;
}

static void PeakingEQ_run(
	LADSPA_Handle p_pInstance,
	unsigned long p_sample_count)
{
	PeakingEQ_Data *l_pPeakingEQ = (PeakingEQ_Data*)p_pInstance;
	LADSPA_Data *l_psrc = l_pPeakingEQ->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pPeakingEQ->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;
    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pPeakingEQ->m_pport[PORT_FREQUENCY]/
	l_pPeakingEQ->m_sample_rate;
	LADSPA_Data l_sin_omega = sinf( l_omega );
	LADSPA_Data l_alpha = l_sin_omega*sinhf( l_pPeakingEQ->m_log2d2 *
	*l_pPeakingEQ->m_pport[PORT_BANDWIDTH] * l_omega / l_sin_omega );
    LADSPA_Data l_A = exp10f( *l_pPeakingEQ->m_pport[PORT_GAIN] / 40.0f );
    LADSPA_Data l_a0 = 1.0f + l_alpha/l_A;
    register LADSPA_Data l_a1 = -2.0f*cosf(l_omega)/l_a0;
    register LADSPA_Data l_a2 = (1.0f - l_alpha/l_A)/l_a0;
    register LADSPA_Data l_b0 = (1.0f + l_alpha*l_A)/l_a0;
    register LADSPA_Data l_b1 = l_a1;
    register LADSPA_Data l_b2 = (1.0f - l_alpha*l_A)/l_a0;
    for(;l_psrc!=l_psrc_end;l_psrc++,l_pdst++){
        register LADSPA_Data l_m = *l_psrc - l_a1*l_pPeakingEQ->m_z1 - l_a2*l_pPeakingEQ->m_z2;
		*l_pdst = l_b0*l_m + l_b1*l_pPeakingEQ->m_z1 + l_b2*l_pPeakingEQ->m_z2;
		l_pPeakingEQ->m_z2 = l_pPeakingEQ->m_z1;
		l_pPeakingEQ->m_z1 = l_m;
	}
}

static void PeakingEQ_cleanup( LADSPA_Handle p_pInstance )
{
	free( p_pInstance );
}

static LADSPA_PortDescriptor PeakingEQ_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char *PeakingEQ_PortNames[]=
{
	"Input",
	"Output",
	"Frequency(Hz)",
	"Bandwidth(octaves)",
	"GAIN(dB)"
};

static LADSPA_PortRangeHint PeakingEQ_PortRangeHints[]=
{
	{0,0,0},
	{0,0,0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_SAMPLE_RATE,
		10.0/44100.0,0.45},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE,
		0.1/12.0,2.0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_0,
		-30.0,30.0}
};

LADSPA_Descriptor RBJPeakingEQ_Descriptor=
{
	5823,
	"RBJ_peakingEQ",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"PeakingEQ RBJ",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	PeakingEQ_PortDescriptors,
	PeakingEQ_PortNames,
	PeakingEQ_PortRangeHints,
	NULL,
	PeakingEQ_instantiate,
	PeakingEQ_connect_port,
	NULL,
	PeakingEQ_run,
	NULL,
	NULL,
	NULL,
	PeakingEQ_cleanup
};
