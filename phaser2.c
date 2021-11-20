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
	PORT_RADIUS,
	PORT_NFILTERS,
	PORT_LFO_FREQUENCY,
	PORT_LFO_AMOUNT,
	PORT_NPORTS
};

typedef struct
{
	LADSPA_Data two_real_z0;
	LADSPA_Data mag_z0_2;
	LADSPA_Data z[3];
} Filter;

static void FilterCoefficients( Filter *p_filter,
						 LADSPA_Data p_frequency,
						 LADSPA_Data p_radius,
						 unsigned long p_sample_rate )
{
	LADSPA_Data theta=2*M_PI*p_frequency/p_sample_rate;
	p_filter->two_real_z0 = 2.0f*p_radius*cosf(theta);
	p_filter->mag_z0_2 = p_radius*p_radius;
}

static void FilterInit( Filter *p_filter )
{
	int i;
	for(i=0;i<3;i++)
		p_filter->z[i]=0.0;
}

static LADSPA_Data FilterEvaluate( Filter *p_filter, LADSPA_Data p_in )
{
	LADSPA_Data h = -p_filter->z[1]*p_filter->two_real_z0
	+p_filter->z[2]*p_filter->mag_z0_2;
	p_filter->z[0] = p_in - h;
	LADSPA_Data y = p_filter->z[0]*p_filter->mag_z0_2
	-p_filter->z[1]*p_filter->two_real_z0
	+p_filter->z[2];
	p_filter->z[2] = p_filter->z[1];
	p_filter->z[1] = p_filter->z[0];
	return y;
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
	LADSPA_Data l_radius = *l_pPhaser->m_pport[PORT_RADIUS];
	LADSPA_Data l_lfo_amount = *l_pPhaser->m_pport[PORT_LFO_AMOUNT];
	LADSPA_Data l_dtheta = 2.0f*M_PI* *l_pPhaser->m_pport[PORT_LFO_FREQUENCY] / l_pPhaser->m_sample_rate;
	for( l_sample=0;l_sample<p_sample_count;l_sample++){
		LADSPA_Data l_out = *l_psrc;
		Filter *l_pFilter = l_pPhaser->m_filters;
		Filter l_filter;
		LADSPA_Data l_lfo_freq = l_freq0 + (0.5f+0.5f*sinf(l_pPhaser->m_lfo_theta))*l_lfo_amount;
		FilterCoefficients(
			&l_filter,
			l_lfo_freq,
			l_radius,
			l_pPhaser->m_sample_rate);
		for(l_f=0;l_f<l_nfilters;l_f++,l_pFilter++){
			l_pFilter->two_real_z0 = l_filter.two_real_z0;
			l_pFilter->mag_z0_2 = l_filter.mag_z0_2;
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
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char* Phaser_PortNames[]=
{
	"Input",
	"Output",
	"Wet Amount",
	"Pole Frequency",
	"Pole Radius",
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
		LADSPA_HINT_LOGARITHMIC|
		LADSPA_HINT_DEFAULT_HIGH,
		0.01,0.9995
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

LADSPA_Descriptor Phaser2_Descriptor =
{
	5814,
	"phaser_allpass",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Phaser allpass poles",
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
