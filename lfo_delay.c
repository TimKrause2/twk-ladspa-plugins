#include <fad.h>
#include <ladspa.h>
#include <math.h>
#include <stdlib.h>

#define DELAY_MAX 1.0f


enum {
	PORT_IN,
	PORT_OUT,
	PORT_DELAY,
	PORT_WET,
	PORT_DRY,
	PORT_FEEDBACK,
	PORT_LFO_FREQUENCY,
	PORT_LFO_AMOUNT,
	PORT_NPORTS
};

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	float      *m_pdata;
	long  m_Nbuf;
	long  m_write_index;
	float m_lfo_theta;
} LFODelay;

static LADSPA_Handle LFODelay_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	LFODelay *l_pLFODelay = malloc( sizeof(LFODelay) );
	if( l_pLFODelay == NULL )
		return NULL;

	l_pLFODelay->m_sample_rate = p_sample_rate;
	l_pLFODelay->m_Nbuf = (long)(p_sample_rate*DELAY_MAX*2) + FadNwindow() + 2;
	l_pLFODelay->m_write_index = 0;
	l_pLFODelay->m_lfo_theta=0.0;

	l_pLFODelay->m_pdata = malloc( sizeof(float) * l_pLFODelay->m_Nbuf );
	
	if( l_pLFODelay->m_pdata == NULL ) {
		free( l_pLFODelay );
		return NULL;
	}
	
	return (LADSPA_Handle)l_pLFODelay;
}

static void LFODelay_connect_port(
	LADSPA_Handle p_instance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata)
{
	LFODelay* l_pLFODelay = (LFODelay*)p_instance;
	l_pLFODelay->m_pport[p_port] = p_pdata;
}

static void LFODelay_activate( LADSPA_Handle p_instance )
{
	LFODelay* l_pLFODelay = (LFODelay*)p_instance;
	l_pLFODelay->m_lfo_theta=0.0;
	l_pLFODelay->m_write_index = 0;
	long i;
	for(i=0;i<l_pLFODelay->m_Nbuf;i++){
		l_pLFODelay->m_pdata[i]=0.0;
	}
}

static void LFODelay_run( LADSPA_Handle p_instance, unsigned long p_sample_count )
{
	LFODelay* l_pLFODelay = (LFODelay*)p_instance;
	LADSPA_Data *l_psrc = l_pLFODelay->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pLFODelay->m_pport[PORT_OUT];
	long l_sample;
	float l_dtheta = 2.0 * M_PI * *l_pLFODelay->m_pport[PORT_LFO_FREQUENCY] / l_pLFODelay->m_sample_rate;
	float l_delay0 = *l_pLFODelay->m_pport[PORT_DELAY] / 1000.0f * l_pLFODelay->m_sample_rate;
	float l_lfo_amount = *l_pLFODelay->m_pport[PORT_LFO_AMOUNT];
	float l_wet_gain = *l_pLFODelay->m_pport[PORT_WET];
	float l_dry_gain = *l_pLFODelay->m_pport[PORT_DRY];
	float l_feedback = *l_pLFODelay->m_pport[PORT_FEEDBACK];
	
	for( l_sample=0;l_sample<p_sample_count;l_sample++){
		// write the incoming data to the cyclic buffer
		l_pLFODelay->m_pdata[l_pLFODelay->m_write_index] = *l_psrc;
		// calculate the start indicis and sample fractions
		// for the dry and wet channels
		long l_dry_index = l_pLFODelay->m_write_index - (FadNwindow()/2);
		if( l_dry_index < 0 )
			l_dry_index += l_pLFODelay->m_Nbuf;
		float l_dry = l_pLFODelay->m_pdata[l_dry_index];
		float l_delay = l_delay0*(1.0f + sinf( l_pLFODelay->m_lfo_theta ) * l_lfo_amount);
		long l_delay_int = (long)ceilf(l_delay);
		float l_delay_frac = l_delay_int - l_delay;
		long l_wet_index = l_pLFODelay->m_write_index - FadNwindow() - l_delay_int;
		if( l_wet_index < 0 )
			l_wet_index += l_pLFODelay->m_Nbuf;
		float l_wet = FadSample( l_pLFODelay->m_pdata, l_wet_index, l_pLFODelay->m_Nbuf, l_delay_frac );

		*l_pdst = l_wet*l_wet_gain + l_dry*l_dry_gain;
		// perform the feedback
		l_dry += l_wet*l_feedback;
		if( l_dry > 1.0f ) l_dry = 1.0f;
		if( l_dry <-1.0f ) l_dry =-1.0f;
		l_pLFODelay->m_pdata[l_dry_index] = l_dry;
		// update the pointers and write index
		l_psrc++;
		l_pdst++;
		l_pLFODelay->m_write_index++;
		if( l_pLFODelay->m_write_index == l_pLFODelay->m_Nbuf )
			l_pLFODelay->m_write_index = 0;
		l_pLFODelay->m_lfo_theta += l_dtheta;
		if( l_pLFODelay->m_lfo_theta >= 2.0*M_PI )
			l_pLFODelay->m_lfo_theta -= 2.0*M_PI;
	}
}

static void LFODelay_cleanup( LADSPA_Handle p_instance )
{
	LFODelay* l_pLFODelay = (LFODelay*)p_instance;
	
	free( l_pLFODelay->m_pdata );
	free( l_pLFODelay );
}

static LADSPA_PortDescriptor LFODelay_PortDescriptors[]=
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* LFODelay_PortNames[]=
{
	"Input",
	"Output",
	"Delay(ms)",
	"Wet",
	"Dry",
	"Feedback",
	"LFO Frequency(Hz)",
	"LFO Amount"
};

static LADSPA_PortRangeHint LFODelay_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_LOW,
		0.0, DELAY_MAX*1000.0f
	},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_HIGH,
		-1.0, 1.0
	},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_HIGH,
		0.0, 1.0
	},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_HIGH,
		0.0, 1.0
	},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_MIDDLE |
		LADSPA_HINT_LOGARITHMIC,
		0.001,10.0
	},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_LOW,
		0.0,1.0
	}
};

LADSPA_Descriptor LFODelay_Descriptor=
{
	5810,
	"lfo_delay",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Delay with LFO",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	LFODelay_PortDescriptors,
	LFODelay_PortNames,
	LFODelay_PortRangeHints,
	NULL,
	LFODelay_instantiate,
	LFODelay_connect_port,
	LFODelay_activate,
	LFODelay_run,
	NULL,
	NULL,
	NULL,
	LFODelay_cleanup
};
