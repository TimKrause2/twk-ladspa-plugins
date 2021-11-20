#include <fad.h>
#include <ladspa.h>
#include <math.h>
#include <stdlib.h>


#define DELAY_MAX 3.0


enum {
	PORT_IN,
	PORT_OUT,
	PORT_DELAY,
	PORT_WET,
	PORT_DRY,
	PORT_FEEDBACK,
	PORT_NPORTS
};

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	float      *m_pdata;
	long  m_Nbuf;
	long  m_write_index;
} Delay;

static LADSPA_Handle Delay_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	Delay *l_pDelay = malloc( sizeof(Delay) );
	if( l_pDelay == NULL )
		return NULL;
	
	l_pDelay->m_sample_rate = p_sample_rate;
	l_pDelay->m_Nbuf = (long)(p_sample_rate*DELAY_MAX) + FadNwindow();
	l_pDelay->m_write_index = 0;
	
	l_pDelay->m_pdata = malloc( sizeof(float) * l_pDelay->m_Nbuf );
	
	if( l_pDelay->m_pdata == NULL ) {
		free( l_pDelay );
		return NULL;
	}
	
	return (LADSPA_Handle)l_pDelay;
}

static void Delay_connect_port(
	LADSPA_Handle p_instance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata)
{
	Delay* l_pDelay = (Delay*)p_instance;
	l_pDelay->m_pport[p_port] = p_pdata;
}

static void Delay_activate( LADSPA_Handle p_instance )
{
	Delay* l_pDelay = (Delay*)p_instance;
	l_pDelay->m_write_index = 0;
	long i;
	for(i=0;i<l_pDelay->m_Nbuf;i++){
		l_pDelay->m_pdata[i]=0.0;
	}
}

static void Delay_run( LADSPA_Handle p_instance, unsigned long p_sample_count )
{
	Delay* l_pDelay = (Delay*)p_instance;
	LADSPA_Data *l_psrc = l_pDelay->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pDelay->m_pport[PORT_OUT];
	long l_sample;

	for( l_sample=0;l_sample<p_sample_count;l_sample++){
		// write the incoming data to the cyclic buffer
		l_pDelay->m_pdata[l_pDelay->m_write_index] = *l_psrc;
		// calculate the start indicis and sample fractions
		// for the dry and wet channels
		long l_dry_index = l_pDelay->m_write_index - (FadNwindow()/2);
		if( l_dry_index < 0 )
			l_dry_index += l_pDelay->m_Nbuf;
		float l_dry = l_pDelay->m_pdata[l_dry_index];
		float  l_delay = *l_pDelay->m_pport[PORT_DELAY]/1000.0f * l_pDelay->m_sample_rate;
		long l_delay_int = (long)ceilf(l_delay);
		float l_delay_frac = l_delay_int - l_delay;
		long l_wet_index = l_pDelay->m_write_index - FadNwindow() - l_delay_int;
		if( l_wet_index < 0 )
			l_wet_index += l_pDelay->m_Nbuf;
		float l_wet = FadSample( l_pDelay->m_pdata, l_wet_index, l_pDelay->m_Nbuf, l_delay_frac );
		// mix the wet and dry and write results to the output buffer
		*l_pdst = l_wet * *l_pDelay->m_pport[PORT_WET] + l_dry * *l_pDelay->m_pport[PORT_DRY];
		// perform the feedback
		l_dry += l_wet * *l_pDelay->m_pport[PORT_FEEDBACK];
		if( l_dry > 1.0 ) l_dry = 1.0;
		if( l_dry <-1.0 ) l_dry =-1.0;
		l_pDelay->m_pdata[l_dry_index] = l_dry;
		// update the pointers and write index
		l_psrc++;
		l_pdst++;
		l_pDelay->m_write_index++;
		if( l_pDelay->m_write_index == l_pDelay->m_Nbuf )
			l_pDelay->m_write_index = 0;
	}
}

static void Delay_cleanup( LADSPA_Handle p_instance )
{
	Delay* l_pDelay = (Delay*)p_instance;
	
	free( l_pDelay->m_pdata );
	free( l_pDelay );
}

static LADSPA_PortDescriptor Delay_PortDescriptors[]=
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* Delay_PortNames[]=
{
	"Input",
	"Output",
	"Delay(ms)",
	"Wet",
	"Dry",
	"Feedback"
};

static LADSPA_PortRangeHint Delay_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_LOW,
		0.0, DELAY_MAX*1000.0
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
		LADSPA_HINT_DEFAULT_MIDDLE,
		-1.0, 1.0
	}
};

LADSPA_Descriptor Delay_Descriptor=
{
	5803,
	"delay",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Delay",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Delay_PortDescriptors,
	Delay_PortNames,
	Delay_PortRangeHints,
	NULL,
	Delay_instantiate,
	Delay_connect_port,
	Delay_activate,
	Delay_run,
	NULL,
	NULL,
	NULL,
	Delay_cleanup
};
