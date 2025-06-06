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
 * DC remove - Removes a bias in the signal.
 *
 * h(z) = 1 - z^-1
 *        ------------
 *        1 - a1*z^-1
 *
 * a1 = cos(w) - sqrt(cos(w)^2 - 4*cos(w) + 3)
 * w = 2*pi*fc/fs
 *
 */

#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

enum {
	PORT_IN,
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_NPORTS
};

typedef struct
{
    LADSPA_Data   m_sample_rate;
	LADSPA_Data  *m_pport[PORT_NPORTS];
	LADSPA_Data   m_xz;
	LADSPA_Data   m_yz;
} DCRemove_Data;

static LADSPA_Handle DCRemove_instantiate(
		const struct _LADSPA_Descriptor *p_pDescriptor,
		unsigned long p_sample_rate)
{
	DCRemove_Data *l_pData = malloc(sizeof(DCRemove_Data));
	if(l_pData){
		l_pData->m_sample_rate = p_sample_rate;
		l_pData->m_xz = 0.0f;
		l_pData->m_yz = 0.0f;
	}
	return (LADSPA_Handle)l_pData;
}

static void DCRemove_connect_port(
		LADSPA_Handle p_instance,
		unsigned long p_port,
		LADSPA_Data *p_pData)
{
	DCRemove_Data *l_pData = (DCRemove_Data*)p_instance;
	l_pData->m_pport[p_port] = p_pData;
}

static void DCRemove_activate(LADSPA_Handle p_instance)
{
	DCRemove_Data *l_pData = (DCRemove_Data*)p_instance;
	l_pData->m_xz = 0.0f;
	l_pData->m_yz = 0.0f;
}

static void DCRemove_run(LADSPA_Handle p_instance, unsigned long p_sample_count)
{
	DCRemove_Data *l_pData = (DCRemove_Data*)p_instance;
    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pData->m_pport[PORT_FREQUENCY]/ l_pData->m_sample_rate;
	LADSPA_Data l_cos = cosf(l_omega);
    LADSPA_Data l_a1 = l_cos - sqrtf(l_cos*l_cos - 4.0f*l_cos +3.0f);
	LADSPA_Data *l_psrc = l_pData->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pData->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;
    for(;l_psrc!=l_psrc_end;l_psrc++, l_pdst++){
        LADSPA_Data y = *l_psrc - l_pData->m_xz + l_a1*l_pData->m_yz;
		l_pData->m_xz = *l_psrc;
        l_pData->m_yz = y;
        *l_pdst = y;
	}
}

static void DCRemove_deactivate(LADSPA_Handle p_instance)
{

}

static void DCRemove_cleanup(LADSPA_Handle p_instance)
{
	free(p_instance);
}

static LADSPA_PortDescriptor DCRemove_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char* DCRemove_PortNames[]=
{
	"Input",
	"Output",
	"Frequency"
};

static LADSPA_PortRangeHint DCRemove_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		1,10.0}
};

LADSPA_Descriptor DCRemove_Descriptor=
{
	5802,
	"DC_Remove",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"DC Remove",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	DCRemove_PortDescriptors,
	DCRemove_PortNames,
	DCRemove_PortRangeHints,
	NULL,
	DCRemove_instantiate,
	DCRemove_connect_port,
	DCRemove_activate,
	DCRemove_run,
	NULL,
	NULL,
	DCRemove_deactivate,
	DCRemove_cleanup
};
