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

	PORT_FREQUENCY1,
	PORT_BANDWIDTH1,
	PORT_GAIN1,
	PORT_LFO_FREQUENCY1,
	PORT_LFO_AMOUNT1,

	PORT_FREQUENCY2,
	PORT_BANDWIDTH2,
	PORT_GAIN2,
	PORT_LFO_FREQUENCY2,
	PORT_LFO_AMOUNT2,

	PORT_FREQUENCY3,
	PORT_BANDWIDTH3,
	PORT_GAIN3,
	PORT_LFO_FREQUENCY3,
	PORT_LFO_AMOUNT3,

	PORT_FREQUENCY4,
	PORT_BANDWIDTH4,
	PORT_GAIN4,
	PORT_LFO_FREQUENCY4,
	PORT_LFO_AMOUNT4,

	PORT_FREQUENCY5,
	PORT_BANDWIDTH5,
	PORT_GAIN5,
	PORT_LFO_FREQUENCY5,
	PORT_LFO_AMOUNT5,

	PORT_NPORTS

};

typedef struct
{
	LADSPA_Data *m_frequency;
	LADSPA_Data *m_bandwidth;
	LADSPA_Data *m_gain;
	LADSPA_Data *m_lfo_frequency;
	LADSPA_Data *m_lfo_amount;
} Bandpass_Port_Data;

typedef struct
{
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
	LADSPA_Data m_lfo_theta;
} Filter_Data;

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	Filter_Data  m_filters[5];
} Bandpass_Data;

static LADSPA_Handle Bandpass_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	Bandpass_Data *l_pBandpass = malloc( sizeof(Bandpass_Data) );
	if( l_pBandpass ){
		l_pBandpass->m_sample_rate = p_sample_rate;
		Filter_Data *l_pFilter = l_pBandpass->m_filters;
		unsigned long l_filter;
		for(l_filter=0;l_filter<5;l_filter++){
			l_pFilter->m_z1 = 0.0;
			l_pFilter->m_z2 = 0.0;
			l_pFilter->m_lfo_theta = 0.0;
			l_pFilter++;
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
	for( l_sample = 0; l_sample < p_sample_count; l_sample++ ){
		*l_pdst++ = 0.0;
	}
	
	unsigned long l_filter;
	Filter_Data *l_pFilter = l_pBandpass->m_filters;
	Bandpass_Port_Data *l_pPortData = (Bandpass_Port_Data*)&l_pBandpass->m_pport[PORT_FREQUENCY1];
	for( l_filter=0;l_filter<5;l_filter++){
		l_psrc = l_pBandpass->m_pport[PORT_IN];
		l_pdst = l_pBandpass->m_pport[PORT_OUT];
		LADSPA_Data l_R = expf(-M_PI * *l_pPortData->m_bandwidth / l_pBandpass->m_sample_rate );
		LADSPA_Data l_G = 1 - l_R;
		l_G *= exp10f( *l_pPortData->m_gain / 20 );
		LADSPA_Data l_a2 = l_R*l_R;
		LADSPA_Data l_lfo_dtheta = 2 * M_PI * *l_pPortData->m_lfo_frequency / l_pBandpass->m_sample_rate;
	
		for( l_sample = 0; l_sample < p_sample_count; l_sample++ ){
			LADSPA_Data l_frequency = *l_pPortData->m_frequency + (0.5f+0.5f*sinf(l_pFilter->m_lfo_theta)) * *l_pPortData->m_lfo_amount;
			LADSPA_Data l_theta = 2 * M_PI * l_frequency / l_pBandpass->m_sample_rate;
			LADSPA_Data l_a1 = -2 * l_R * cosf(l_theta);
			LADSPA_Data m = *l_psrc - l_a1*l_pFilter->m_z1 - l_a2*l_pFilter->m_z2;
			*l_pdst += l_G*(m - l_R*l_pFilter->m_z2);
			l_pFilter->m_z2 = l_pFilter->m_z1;
			l_pFilter->m_z1 = m;
			l_psrc++;
			l_pdst++;
			l_pFilter->m_lfo_theta += l_lfo_dtheta;
			if( l_pFilter->m_lfo_theta >= 2*M_PI )
				l_pFilter->m_lfo_theta -= 2*M_PI;
		}
		l_pFilter++;
		l_pPortData++;
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
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
	
};

static const char* Bandpass_PortNames[]=
{
	"Input",
	"Output",
	"Frequency 1",
	"Bandwidth 1",
	"Gain 1",
	"LFO frequency 1",
	"LFO amount 1",
	"Frequency 2",
	"Bandwidth 2",
	"Gain 2",
	"LFO frequency 2",
	"LFO amount 2",
	"Frequency 3",
	"Bandwidth 3",
	"Gain 3",
	"LFO frequency 3",
	"LFO amount 3",
	"Frequency 4",
	"Bandwidth 4",
	"Gain 4",
	"LFO frequency 4",
	"LFO amount 4",
	"Frequency 5",
	"Bandwidth 5",
	"Gain 5",
	"LFO frequency 5",
	"LFO amount 5"
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
	},
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
	},
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
	},
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
	},
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
	},
};

LADSPA_Descriptor LFOBandpass5_Descriptor=
{
	5809,
	"lfo_bp_bank",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Bandpass bank w LFOs",
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
