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
	PORT_RX1,
	PORT_RX2,
	PORT_TX1,
	PORT_TX2,
	PORT_UNITY,
	PORT_RATIO_HI,
	PORT_RATIO_LO,
	PORT_THRESHOLD,
	PORT_DECAY,
	PORT_NPORTS
};

#define N_WINDOW 2
#define ABS_MIN 1.19209290e-7
#define PEEK2_K 0.5f

typedef struct
{
    LADSPA_Data m_env;
    LADSPA_Data m_window[N_WINDOW];
    int         m_i_window;
} Compressor_State;

typedef struct
{
    LADSPA_Data      m_sample_rate;
    LADSPA_Data*     m_pdata[PORT_NPORTS];
    Compressor_State m_cs[2];
} Compressor_Data;

void Compressor_State_Init( Compressor_State *cs ){
    cs->m_env = 0.0f;
    cs->m_i_window = 0;
    for(int s=0;s<N_WINDOW;s++){
        cs->m_window[s] = 0.0f;
    }
}

static LADSPA_Handle Compressor_instantiate(
	const struct _LADSPA_Descriptor* p_pDescriptor,
	unsigned long SampleRate)
{
	Compressor_Data* l_pData = malloc(sizeof(Compressor_Data));
	if(l_pData){
        l_pData->m_sample_rate = SampleRate;
        for(int i=0;i<2;i++){
            Compressor_State_Init( &l_pData->m_cs[i] );
        }
	}
	return (LADSPA_Handle)l_pData;
}

static void Compressor_connect_port(
	LADSPA_Handle p_instance,
	unsigned long p_port,
	LADSPA_Data*  p_pdata)
{
	Compressor_Data* m_pData = (Compressor_Data*)p_instance;
	m_pData->m_pdata[p_port] = p_pdata;
}

static void Compressor_activate(LADSPA_Handle p_instance)
{
	Compressor_Data* l_pData = (Compressor_Data*)p_instance;
    for(int i=0;i<2;i++){
        Compressor_State_Init( &l_pData->m_cs[i] );
    }
}

static void buffer_compress(
    LADSPA_Data*      p_psrc,
    LADSPA_Data*      p_pdst,
    Compressor_State* p_pcs,
    LADSPA_Data       p_unity,
    LADSPA_Data       p_ratio_hi,
    LADSPA_Data       p_ratio_lo,
    LADSPA_Data       p_threshold,
    LADSPA_Data       p_alpha_decay,
    unsigned long     p_nsamples )
{
	LADSPA_Data *l_psrc = p_psrc;
	LADSPA_Data *l_pdst = p_pdst;
    LADSPA_Data *l_psrc_end = l_psrc + p_nsamples;
    LADSPA_Data  l_root2 = sqrtf(2.0f);
    for(;l_psrc!=l_psrc_end;l_psrc++,l_pdst++){
        p_pcs->m_window[p_pcs->m_i_window] = *l_psrc;
        LADSPA_Data l_peek = fabsf(*l_psrc);
        LADSPA_Data l_peek2 = fabsf(
            p_pcs->m_window[0]*PEEK2_K +
            p_pcs->m_window[1]*PEEK2_K);
        if(l_peek2 > l_peek )
            l_peek = l_peek2;
        if(l_peek > p_pcs->m_env){
            p_pcs->m_env = l_peek;
        }else{
            LADSPA_Data l_delta = l_peek - p_pcs->m_env;
            p_pcs->m_env += l_delta*p_alpha_decay;
        }
        if(p_pcs->m_env<ABS_MIN){
			*l_pdst=0.0f;
		}else{
            LADSPA_Data l_env_db_in = log10f(p_pcs->m_env)*20.0f;
            LADSPA_Data l_env_db_out;
            if(l_env_db_in<=p_threshold){
				LADSPA_Data l_comp_env_db = p_threshold - p_unity;
				l_comp_env_db /= p_ratio_hi;
                LADSPA_Data l_exp_env_db = l_env_db_in - p_threshold;
				l_exp_env_db /= p_ratio_lo;
                l_env_db_out = p_unity + l_comp_env_db + l_exp_env_db - l_env_db_in;
			}else{
                LADSPA_Data l_comp_env_db = l_env_db_in - p_unity;
				l_comp_env_db /= p_ratio_hi;
                l_env_db_out = p_unity + l_comp_env_db - l_env_db_in;
			}
            LADSPA_Data l_gain_factor = exp10f(l_env_db_out/20.0f);
            *l_pdst = *l_psrc * l_gain_factor;
		}
        p_pcs->m_i_window++;
        p_pcs->m_i_window%=N_WINDOW;
    }
}

static void Compressor_run(LADSPA_Handle p_instance, unsigned long SampleCount)
{
	Compressor_Data* l_pData = (Compressor_Data*)p_instance;
	unsigned long n;
	LADSPA_Data l_alpha_decay = 1.0 - powf(0.05,1.0 / *l_pData->m_pdata[PORT_DECAY] / l_pData->m_sample_rate);

	buffer_compress(
		l_pData->m_pdata[PORT_RX1],
		l_pData->m_pdata[PORT_TX1],
        &l_pData->m_cs[0],
		*l_pData->m_pdata[PORT_UNITY],
		*l_pData->m_pdata[PORT_RATIO_HI],
		*l_pData->m_pdata[PORT_RATIO_LO],
		*l_pData->m_pdata[PORT_THRESHOLD],
		l_alpha_decay,
		SampleCount );
	
	buffer_compress(
		l_pData->m_pdata[PORT_RX2],
		l_pData->m_pdata[PORT_TX2],
        &l_pData->m_cs[1],
		*l_pData->m_pdata[PORT_UNITY],
		*l_pData->m_pdata[PORT_RATIO_HI],
		*l_pData->m_pdata[PORT_RATIO_LO],
		*l_pData->m_pdata[PORT_THRESHOLD],
		l_alpha_decay,
		SampleCount );
	
}

static void Compressor_deactivate(LADSPA_Handle p_instance)
{
}

static void Compressor_cleanup(LADSPA_Handle p_instance)
{
	free(p_instance);
}

static LADSPA_PortDescriptor Compressor_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,

	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,

	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char* Compressor_PortNames[]=
{
	"Input1",
	"Input2",
	"Output1",
	"Output2",
    "Unity(dBFS)",
    "RatioHi(in/out)",
    "RatioLo(in/out)",
    "Threshold(dBFS)",
    "Decay(seconds)"
};

static LADSPA_PortRangeHint Compressor_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_DEFAULT_MAXIMUM|
		LADSPA_HINT_BOUNDED_ABOVE,
		-96.0,0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.01,100.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.01,100.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_DEFAULT_MINIMUM|
		LADSPA_HINT_BOUNDED_ABOVE,
		-140.0,0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.001,5.0}
};

LADSPA_Descriptor Compressor_Descriptor=
{
	5801,
	"compressor",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Compressor",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Compressor_PortDescriptors,
	Compressor_PortNames,
	Compressor_PortRangeHints,
	NULL,
	Compressor_instantiate,
	Compressor_connect_port,
	Compressor_activate,
	Compressor_run,
	NULL,
	NULL,
	Compressor_deactivate,
	Compressor_cleanup
};
