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
 * b0 = (1 + cos(omega))/2
 * b1 =-(1 + cos(omega))
 * b2 = (1 + cos(omega))/2
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

#define N_FILTERS 6

typedef struct {
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	LADSPA_Data m_z[N_FILTERS][3];
} Highpass_Data;

static LADSPA_Handle Highpass_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate ){
	Highpass_Data *l_pHighpass = malloc( sizeof(Highpass_Data) );
	if( l_pHighpass ){
		l_pHighpass->m_sample_rate = p_sample_rate;
		int f,i;
		for(f=0;f<N_FILTERS;f++){
			for(i=0;i<3;i++){
				l_pHighpass->m_z[f][i]=0.0f;
			}
		}
	}
	return (LADSPA_Handle)l_pHighpass;
}

static void Highpass_connect_port(
	LADSPA_Handle p_pInstance,
	unsigned long p_port,
	LADSPA_Data *p_pdata)
{
	Highpass_Data *l_pHighpass = (Highpass_Data*)p_pInstance;
	l_pHighpass->m_pport[p_port] = p_pdata;
}

static void Highpass_run(
	LADSPA_Handle p_pInstance,
	unsigned long p_sample_count )
{
	Highpass_Data *l_pHighpass = (Highpass_Data*)p_pInstance;
	unsigned long l_sample;
	LADSPA_Data *l_psrc = l_pHighpass->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pHighpass->m_pport[PORT_OUT];
	
	LADSPA_Data l_omega = 2.0f*(float)M_PI* *l_pHighpass->m_pport[PORT_FREQUENCY] /
	l_pHighpass->m_sample_rate;
	LADSPA_Data l_sin_omega;// = sinf( l_omega );
	LADSPA_Data l_cos_omega;// = cosf( l_omega );
	sincosf(l_omega, &l_sin_omega, &l_cos_omega);
	LADSPA_Data l_alpha = l_sin_omega /( 2.0F * powf(*l_pHighpass->m_pport[PORT_Q], 1.0F/N_FILTERS) );
	LADSPA_Data l_a0 = 1.0f + l_alpha;
	LADSPA_Data l_a1 =-2.0f * l_cos_omega / l_a0;
	LADSPA_Data l_a2 = (1.0f - l_alpha) / l_a0;
	LADSPA_Data l_temp = (1.0f + l_cos_omega)/l_a0;
	LADSPA_Data l_b0 = l_temp/2.0f;
	LADSPA_Data l_b1 =-l_temp;
	LADSPA_Data l_b2 = l_b0;
	LADSPA_Data l_G = powf(10.0F, *l_pHighpass->m_pport[PORT_GAIN] / 20.0f );
	l_sample = p_sample_count;
	while(l_sample){
		LADSPA_Data l_x = *(l_psrc++);
#define BIQUAD_EVALUATE(INDEX) \
		l_pHighpass->m_z[INDEX][0] = l_x - l_a1*l_pHighpass->m_z[INDEX][1] - l_a2*l_pHighpass->m_z[INDEX][2]; \
		l_x = l_b0*l_pHighpass->m_z[INDEX][0] + l_b1*l_pHighpass->m_z[INDEX][1] + l_b2*l_pHighpass->m_z[INDEX][2]; \
		l_pHighpass->m_z[INDEX][2] = l_pHighpass->m_z[INDEX][1]; \
		l_pHighpass->m_z[INDEX][1] = l_pHighpass->m_z[INDEX][0];

		BIQUAD_EVALUATE(0)
		BIQUAD_EVALUATE(1)
		BIQUAD_EVALUATE(2)
		BIQUAD_EVALUATE(3)
		BIQUAD_EVALUATE(4)
		BIQUAD_EVALUATE(5)

		
		*(l_pdst++) = l_x*l_G;
		
		l_sample--;
	}
}

static void Highpass_cleanup( LADSPA_Handle p_pInstance )
{
	free( p_pInstance );
}

static LADSPA_PortDescriptor Highpass_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char *Highpass_PortNames[]=
{
	"Input",
	"Output",
	"Frequency",
	"Q",
	"Gain"
};

static LADSPA_PortRangeHint Highpass_PortRangeHints[]=
{
	{0,0,0},
	{0,0,0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
		10,21000},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
		0.01,100},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_0,
		-60,24}
};

LADSPA_Descriptor RBJHighpassQ12_Descriptor=
{
	5818,
	"RBJ_highpass_Q_12order",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Highpass RBJ(Q) 12 Order",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Highpass_PortDescriptors,
	Highpass_PortNames,
	Highpass_PortRangeHints,
	NULL,
	Highpass_instantiate,
	Highpass_connect_port,
	NULL,
	Highpass_run,
	NULL,
	NULL,
	NULL,
	Highpass_cleanup
};
