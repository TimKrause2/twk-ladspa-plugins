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
#include <fad.h>
#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

#define T_WINDOW 0.02f

enum {
	PORT_IN,
	PORT_OUT,
    PORT_PITCH,
    PORT_NPORTS
};

typedef struct
{
    LADSPA_Data m_ratio;
    int m_Ninbuffer;
    int m_Noutbuffer;
    int m_w_index;
    int m_r_index;
    LADSPA_Data *m_pindata; // m_Nbuffer*2
    LADSPA_Data *m_poutdata;
} PShift_Unit;

PShift_Unit *PShift_Unit_new(int p_Nbuffer, int p_r_index)
{
    PShift_Unit *p_pPShift_Unit = malloc(sizeof(PShift_Unit));
    if(!p_pPShift_Unit){
        return NULL;
    }
    p_pPShift_Unit->m_ratio = 1.0f;
    p_pPShift_Unit->m_Ninbuffer = p_Nbuffer*2 + FadNwindow();
    p_pPShift_Unit->m_Noutbuffer = p_Nbuffer;
    p_pPShift_Unit->m_w_index = 0;
    p_pPShift_Unit->m_r_index = p_r_index;
    p_pPShift_Unit->m_pindata =
            malloc(sizeof(LADSPA_Data)*p_pPShift_Unit->m_Ninbuffer);
    p_pPShift_Unit->m_poutdata =
            malloc(sizeof(LADSPA_Data)*p_pPShift_Unit->m_Noutbuffer);
    return p_pPShift_Unit;
}

void PShift_Unit_destroy(PShift_Unit *p_pPShift_Unit)
{
    free(p_pPShift_Unit->m_pindata);
    free(p_pPShift_Unit->m_poutdata);
    free(p_pPShift_Unit);
}

void PShift_Unit_reset(PShift_Unit *p_pPSU)
{
    for(int i=0;i<p_pPSU->m_Ninbuffer;i++){
        p_pPSU->m_pindata[i] = 0.0f;
    }
    for(int i=0;i<p_pPSU->m_Noutbuffer;i++){
        p_pPSU->m_poutdata[i] = 0.0f;
    }
    p_pPSU->m_ratio = 1.0;
}

LADSPA_Data PShift_Unit_evaluate(PShift_Unit *p_pPSU,
                                 LADSPA_Data p_x)
{
    LADSPA_Data l_y = p_pPSU->m_poutdata[p_pPSU->m_r_index];
    p_pPSU->m_pindata[p_pPSU->m_w_index] = p_x;
    if(++p_pPSU->m_r_index == p_pPSU->m_Noutbuffer){
        p_pPSU->m_r_index = 0;
        LADSPA_Data l_delay0 =
                -p_pPSU->m_Noutbuffer*(1+p_pPSU->m_ratio*0.5f);
        LADSPA_Data l_ddelay = p_pPSU->m_ratio;
        LADSPA_Data *l_pdst = p_pPSU->m_poutdata;
        for(int i=0;i<p_pPSU->m_Noutbuffer;i++){
            int l_delay_int = floor(l_delay0);
            float l_delay_frac = l_delay0 - l_delay_int;
            int l_access_index = p_pPSU->m_w_index - FadNwindow()/2 - 1
                    + l_delay_int;
            if(l_access_index < 0)
                l_access_index+=p_pPSU->m_Ninbuffer;
            float l_alpha = (float)i/p_pPSU->m_Noutbuffer;
            float l_envelope;
            if(l_alpha<=0.5){
                l_envelope = l_alpha*2.0f;
            }else{
                l_envelope = 2.0f - l_alpha*2.0f;
            }
            float l_out = FadSample(
                        p_pPSU->m_pindata,
                        l_access_index,
                        p_pPSU->m_Ninbuffer,
                        l_delay_frac);
            *l_pdst = l_out * l_envelope;
            l_pdst++;
            l_delay0+=l_ddelay;
        }
    }
    if(++p_pPSU->m_w_index == p_pPSU->m_Ninbuffer){
        p_pPSU->m_w_index = 0;
    }
    return l_y;
}


typedef struct
{
    float m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
    PShift_Unit *m_pPSUs[2];
} PShift;

static LADSPA_Handle PShift_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
    PShift *l_pPShift = malloc( sizeof(PShift) );
    if( l_pPShift == NULL )
		return NULL;

    l_pPShift->m_sample_rate = p_sample_rate;

    int l_Nbuf = (int)(T_WINDOW*l_pPShift->m_sample_rate);
    l_pPShift->m_pPSUs[0] = PShift_Unit_new(l_Nbuf, 0);
    l_pPShift->m_pPSUs[1] = PShift_Unit_new(l_Nbuf, l_Nbuf/2);

    return (LADSPA_Handle)l_pPShift;
}

static void PShift_connect_port(
	LADSPA_Handle p_instance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata)
{
    PShift* l_pPShift = (PShift*)p_instance;
    l_pPShift->m_pport[p_port] = p_pdata;
}

static void PShift_activate( LADSPA_Handle p_instance )
{
    PShift* l_pPShift = (PShift*)p_instance;
    PShift_Unit_reset(l_pPShift->m_pPSUs[0]);
    PShift_Unit_reset(l_pPShift->m_pPSUs[1]);
}

static void PShift_run( LADSPA_Handle p_instance, unsigned long p_sample_count )
{
    PShift* l_pPShift = (PShift*)p_instance;
    LADSPA_Data *l_psrc = l_pPShift->m_pport[PORT_IN];
    LADSPA_Data *l_pdst = l_pPShift->m_pport[PORT_OUT];
    float l_ratio = powf(2.0f,*l_pPShift->m_pport[PORT_PITCH]/12.0f);
    l_pPShift->m_pPSUs[0]->m_ratio = l_ratio;
    l_pPShift->m_pPSUs[1]->m_ratio = l_ratio;
    unsigned long l_sample;
	for( l_sample=0;l_sample<p_sample_count;l_sample++){
        *l_pdst = PShift_Unit_evaluate(
                    l_pPShift->m_pPSUs[0], *l_psrc) +
                PShift_Unit_evaluate(
                    l_pPShift->m_pPSUs[1], *l_psrc);
        // update the pointers
		l_psrc++;
		l_pdst++;
	}
}

static void PShift_cleanup( LADSPA_Handle p_instance )
{
    PShift* l_pPShift = (PShift*)p_instance;
    PShift_Unit_destroy(l_pPShift->m_pPSUs[0]);
    PShift_Unit_destroy(l_pPShift->m_pPSUs[1]);
    free( l_pPShift );
}

static LADSPA_PortDescriptor PShift_PortDescriptors[]=
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
};

static const char* PShift_PortNames[]=
{
	"Input",
	"Output",
    "Pitch(semitones)"
};

static LADSPA_PortRangeHint PShift_PortRangeHints[]=
{
    {0, 0.0f, 0.0f},
    {0, 0.0f, 0.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
        LADSPA_HINT_DEFAULT_MIDDLE,
        -12.0f, 12.0f
	},
};

LADSPA_Descriptor PShift_Descriptor=
{
    5834,
    "pitch_shifter",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
    "Pitch Shifter",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
    PShift_PortDescriptors,
    PShift_PortNames,
    PShift_PortRangeHints,
	NULL,
    PShift_instantiate,
    PShift_connect_port,
    PShift_activate,
    PShift_run,
	NULL,
	NULL,
	NULL,
    PShift_cleanup
};
