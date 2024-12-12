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
 * LADSPA bandpass filter bank of 5 with random frequency modulation
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

#define RAND_FLOAT drand48()
#define FREQUENCY_MIN 10.0f
#define FREQUENCY_MAX 20.0e3f
#define PERIOD_MIN 0.01f
#define PERIOD_MAX 1.0f
#define PERIOD_MOD_MIN 0.01f
#define PERIOD_MOD_MAX 1.0f
#define AMOUNT_MIN 0.0f
#define AMOUNT_MAX 5.0E3f
#define BANDWIDTH_MIN 10.0f
#define BANDWIDTH_MAX 1000.0f
#define GAIN_MIN -60.0f
#define GAIN_MAX  24.0f

#define N_BUFFER 8192

enum {
	PORT_IN,
	PORT_OUT,

	PORT_PERIOD,
	PORT_PERIOD_MOD,
	
	PORT_FREQUENCY1,
	PORT_BANDWIDTH1,
	PORT_GAIN1,
	PORT_LFR_AMOUNT1,

	PORT_FREQUENCY2,
	PORT_BANDWIDTH2,
	PORT_GAIN2,
	PORT_LFR_AMOUNT2,

	PORT_FREQUENCY3,
	PORT_BANDWIDTH3,
	PORT_GAIN3,
	PORT_LFR_AMOUNT3,

	PORT_FREQUENCY4,
	PORT_BANDWIDTH4,
	PORT_GAIN4,
	PORT_LFR_AMOUNT4,

	PORT_FREQUENCY5,
	PORT_BANDWIDTH5,
	PORT_GAIN5,
	PORT_LFR_AMOUNT5,

	PORT_NPORTS

};

typedef struct
{
	LADSPA_Data *m_frequency;
	LADSPA_Data *m_bandwidth;
	LADSPA_Data *m_gain;
	LADSPA_Data *m_lfr_amount;
} Bandpass_Port_Data;

typedef struct
{
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
    LADSPA_Data m_a1;
    LADSPA_Data m_a2;
    LADSPA_Data m_R;
    LADSPA_Data m_G;
    LADSPA_Data m_lfr_frequency;
    LADSPA_Data m_lfr_frequency1;
    LADSPA_Data m_lfr_dfrequency;
	unsigned long m_lfr_sample;
	unsigned long m_lfr_sample_count;
    Bandpass_Port_Data *m_port_data;
} Filter_Data;

typedef struct
{
    LADSPA_Data  m_sample_rate;
    LADSPA_Data *m_pport[PORT_NPORTS];
    Filter_Data  m_filters[5];
} Bandpass_Data;

static void Filter_init(Filter_Data *filter, Bandpass_Port_Data *port_data)
{
    filter->m_z1 = 0.0f;
    filter->m_z2 = 0.0f;
    filter->m_lfr_frequency1 = 1000.0f;
    filter->m_lfr_sample = 0;
    filter->m_lfr_sample_count = 0;
    filter->m_port_data = port_data;
}

static void Filter_set(Filter_Data *filter, LADSPA_Data sample_rate)
{
    filter->m_R = expf(-M_PIf * *filter->m_port_data->m_bandwidth / sample_rate );
    filter->m_G = 1.0f - filter->m_R;
    filter->m_G *= exp10f( *filter->m_port_data->m_gain / 20.0f );
    filter->m_a2 = filter->m_R*filter->m_R;
}

static LADSPA_Data Filter_evaluate(Filter_Data *filter, Bandpass_Data *bp, LADSPA_Data x)
{

    if( filter->m_lfr_sample == filter->m_lfr_sample_count ){
        // reached the end of the last period
        filter->m_lfr_frequency = filter->m_lfr_frequency1;
        filter->m_lfr_frequency1 = *filter->m_port_data->m_frequency
                                      + RAND_FLOAT * *filter->m_port_data->m_lfr_amount;
        filter->m_lfr_sample_count =
            (*bp->m_pport[PORT_PERIOD] +
             RAND_FLOAT * *bp->m_pport[PORT_PERIOD_MOD])*
            bp->m_sample_rate;
        filter->m_lfr_sample = 0;
        filter->m_lfr_dfrequency =
            (filter->m_lfr_frequency1-filter->m_lfr_frequency)/filter->m_lfr_sample_count;
    }
    LADSPA_Data l_theta = 2.0f * M_PIf * filter->m_lfr_frequency / bp->m_sample_rate;
    filter->m_a1 = -2.0f * filter->m_R * cosf(l_theta);
    LADSPA_Data m = x - filter->m_a1*filter->m_z1 - filter->m_a2*filter->m_z2;
    LADSPA_Data y = filter->m_G*(m - filter->m_R*filter->m_z2);
    filter->m_z2 = filter->m_z1;
    filter->m_z1 = m;
    filter->m_lfr_frequency += filter->m_lfr_dfrequency;
    filter->m_lfr_sample++;
    return y;
}


static LADSPA_Handle Bandpass_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	Bandpass_Data *l_pBandpass = malloc( sizeof(Bandpass_Data) );
	if( l_pBandpass ){
		l_pBandpass->m_sample_rate = p_sample_rate;
		Filter_Data *l_pFilter = l_pBandpass->m_filters;
        Bandpass_Port_Data *l_pPortData = (Bandpass_Port_Data*)&l_pBandpass->m_pport[PORT_FREQUENCY1];
        unsigned long l_filter;
		for(l_filter=0;l_filter<5;l_filter++){
            Filter_init(l_pFilter, l_pPortData);
			l_pFilter++;
            l_pPortData++;
		}
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
	
	unsigned long l_sample;
    LADSPA_Data *l_psrc = l_pBandpass->m_pport[PORT_IN];
    LADSPA_Data *l_pdst = l_pBandpass->m_pport[PORT_OUT];
	
	unsigned long l_filter;
	Filter_Data *l_pFilter = l_pBandpass->m_filters;
	for( l_filter=0;l_filter<5;l_filter++){
        Filter_set(l_pFilter, l_pBandpass->m_sample_rate);
        l_pFilter++;
    }

    for( l_sample = 0; l_sample < p_sample_count; l_sample++ ){
        l_pFilter = l_pBandpass->m_filters;
        LADSPA_Data y = 0.0f;
        for(int i=0;i<5;i++){
            y += Filter_evaluate(l_pFilter, l_pBandpass, *l_psrc);
            l_pFilter++;
        }
        *l_pdst = y;
        l_psrc++;
        l_pdst++;
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
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	
	
};

static const char* Bandpass_PortNames[]=
{
	"Input",
	"Output",
	"Period",
	"Period Mod",
	
	"Frequency 1",
	"Bandwidth 1",
	"Gain 1",
	"LFR amount 1",
	"Frequency 2",
	"Bandwidth 2",
	"Gain 2",
	"LFR amount 2",
	"Frequency 3",
	"Bandwidth 3",
	"Gain 3",
	"LFR amount 3",
	"Frequency 4",
	"Bandwidth 4",
	"Gain 4",
	"LFR amount 4",
	"Frequency 5",
	"Bandwidth 5",
	"Gain 5",
	"LFR amount 5"
};

static LADSPA_PortRangeHint Bandpass_PortRangeHints[]=
{
	{0, 0, 0},
	{0, 0, 0},

	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|
		LADSPA_HINT_DEFAULT_MIDDLE,
        PERIOD_MIN, PERIOD_MAX
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|
		LADSPA_HINT_DEFAULT_MIDDLE,
        PERIOD_MOD_MIN, PERIOD_MOD_MAX
	},

    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_MIDDLE,
        FREQUENCY_MIN, FREQUENCY_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        BANDWIDTH_MIN, BANDWIDTH_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_0,
        GAIN_MIN, GAIN_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        AMOUNT_MIN, AMOUNT_MAX
    },

    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_MIDDLE,
        FREQUENCY_MIN, FREQUENCY_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        BANDWIDTH_MIN, BANDWIDTH_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_0,
        GAIN_MIN, GAIN_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        AMOUNT_MIN, AMOUNT_MAX
    },

    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_MIDDLE,
        FREQUENCY_MIN, FREQUENCY_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        BANDWIDTH_MIN, BANDWIDTH_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_0,
        GAIN_MIN, GAIN_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        AMOUNT_MIN, AMOUNT_MAX
    },

    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_MIDDLE,
        FREQUENCY_MIN, FREQUENCY_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        BANDWIDTH_MIN, BANDWIDTH_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_0,
        GAIN_MIN, GAIN_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        AMOUNT_MIN, AMOUNT_MAX
    },

    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_MIDDLE,
        FREQUENCY_MIN, FREQUENCY_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        BANDWIDTH_MIN, BANDWIDTH_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_0,
        GAIN_MIN, GAIN_MAX
    },
    { LADSPA_HINT_BOUNDED_BELOW|
        LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_LOW,
        AMOUNT_MIN, AMOUNT_MAX
    }

};

LADSPA_Descriptor LFRBandpass5_Descriptor=
{
	5811,
	"lfr_bp_bank",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Bandpass bank with random modulation",
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
