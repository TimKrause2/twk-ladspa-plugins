#include <ladspa.h>
#include <math.h>
#include <complex.h>
#include <stdlib.h>

#define N_FILTERS 8

enum {
	PORT_IN,
	PORT_OUT,
	PORT_WET,
	PORT_FREQUENCY,
	PORT_NFILTERS,
	PORT_LFO_FREQUENCY,
	PORT_LFO_AMOUNT,
	PORT_NPORTS
};

typedef struct
{
	LADSPA_Data m_e_c;
	LADSPA_Data m_x_last;
	LADSPA_Data m_alpha;
} Filter;

static void FilterCoefficients( Filter *p_filter, LADSPA_Data p_frequency, unsigned long p_sample_rate )
{
	LADSPA_Data l_T = (LADSPA_Data)1.0f/p_sample_rate;
	LADSPA_Data l_tau = 1.0f/(2.0f*M_PI*p_frequency);
	p_filter->m_alpha = l_tau/(l_T + l_tau);
}

static void FilterInit( Filter *p_filter )
{
	p_filter->m_e_c = 0.0;
	p_filter->m_x_last = 0.0;
}

static LADSPA_Data FilterEvaluate( Filter *p_filter, LADSPA_Data p_in )
{
	p_filter->m_e_c = p_filter->m_alpha*( p_filter->m_e_c + p_in - p_filter->m_x_last );
	LADSPA_Data l_v_o = 2*p_filter->m_e_c - p_in;
	p_filter->m_x_last = p_in;
	return -l_v_o;
}

typedef struct {
	unsigned long m_sample_rate;
	LADSPA_Data  *m_pport[PORT_NPORTS];
	Filter        m_filters[N_FILTERS];
	LADSPA_Data   m_lfo_theta;
} Phaser_Data;

static LADSPA_Handle Phaser_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	Phaser_Data *l_pPhaser = malloc( sizeof(Phaser_Data) );
	if(l_pPhaser){
		l_pPhaser->m_sample_rate = p_sample_rate;
		l_pPhaser->m_lfo_theta = 0.0;
		unsigned long l_f;
		for( l_f=0;l_f<N_FILTERS;l_f++){
			FilterInit( &l_pPhaser->m_filters[l_f] );
		}
	}
	return (LADSPA_Handle)l_pPhaser;
}

static void Phaser_connect_port(
	LADSPA_Handle p_instance,
	unsigned long p_port,
	LADSPA_Data *p_pdata )
{
	Phaser_Data *l_pPhaser = (Phaser_Data*)p_instance;
	l_pPhaser->m_pport[p_port] = p_pdata;
}

static void Phaser_run( LADSPA_Handle p_instance, unsigned long p_sample_count )
{
	Phaser_Data *l_pPhaser = (Phaser_Data*)p_instance;
	unsigned long l_f;
	LADSPA_Data l_wet_gain = *l_pPhaser->m_pport[PORT_WET];
	LADSPA_Data *l_psrc = l_pPhaser->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pPhaser->m_pport[PORT_OUT];
	unsigned long l_nfilters = (unsigned long)floorf(*l_pPhaser->m_pport[PORT_NFILTERS]);
	unsigned long l_sample;
	LADSPA_Data l_freq0 = *l_pPhaser->m_pport[PORT_FREQUENCY];
	LADSPA_Data l_lfo_amount = *l_pPhaser->m_pport[PORT_LFO_AMOUNT];
	LADSPA_Data l_dtheta = 2.0f*M_PI* *l_pPhaser->m_pport[PORT_LFO_FREQUENCY] / l_pPhaser->m_sample_rate;
	for( l_sample=0;l_sample<p_sample_count;l_sample++){
		LADSPA_Data l_out = *l_psrc;
		Filter *l_pFilter = l_pPhaser->m_filters;
		LADSPA_Data l_lfo_freq = l_freq0 + (0.5f+0.5f*sinf(l_pPhaser->m_lfo_theta))*l_lfo_amount;
		for(l_f=0;l_f<l_nfilters;l_f++,l_pFilter++){
			FilterCoefficients(
				l_pFilter,
				l_lfo_freq,
				l_pPhaser->m_sample_rate);
			l_out = FilterEvaluate( l_pFilter, l_out );
		}
		*l_pdst = (l_out*l_wet_gain + *l_psrc)*0.5f;
		l_pdst++;
		l_psrc++;
		l_pPhaser->m_lfo_theta += l_dtheta;
		if(l_pPhaser->m_lfo_theta >= 2.0f*M_PI)
			l_pPhaser->m_lfo_theta-=2.0f*M_PI;
	}
}

static void Phaser_cleanup( LADSPA_Handle p_instance )
{
	free( p_instance );
}

static LADSPA_PortDescriptor Phaser_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char* Phaser_PortNames[]=
{
	"Input",
	"Output",
	"Wet",
	"Frequency",
	"N stages",
	"LFO Frequency",
	"LFO Amount"
};

static LADSPA_PortRangeHint Phaser_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MAXIMUM,
		-1.0,1.0
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|
		LADSPA_HINT_DEFAULT_LOW,
		10.0, 5000.0
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_INTEGER|
		LADSPA_HINT_DEFAULT_LOW,
		1.0,N_FILTERS
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|
		LADSPA_HINT_DEFAULT_MIDDLE,
		0.001, 10.0
	},
	{ LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_LOW,
		0.0, 5000.0
	}
};

LADSPA_Descriptor Phaser_Descriptor =
{
	5813,
	"phaser",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Phaser circuit simulator",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Phaser_PortDescriptors,
	Phaser_PortNames,
	Phaser_PortRangeHints,
	NULL,
	Phaser_instantiate,
	Phaser_connect_port,
	NULL,
	Phaser_run,
	NULL,
	NULL,
	NULL,
	Phaser_cleanup
};
