#include <fad.h>
#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

#define DELAY_MAX 10.0f


enum {
	PORT_IN,
	PORT_OUT,
	PORT_DELAY,
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
} LFOAllPass;

static LADSPA_Handle LFOAllPass_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	LFOAllPass *l_pLFOAllPass = malloc( sizeof(LFOAllPass) );
	if( l_pLFOAllPass == NULL )
		return NULL;

	l_pLFOAllPass->m_sample_rate = p_sample_rate;
	l_pLFOAllPass->m_Nbuf = (long)(p_sample_rate*DELAY_MAX*2) + FadNwindow();
	l_pLFOAllPass->m_write_index = 0;
	l_pLFOAllPass->m_lfo_theta=0.0;

	l_pLFOAllPass->m_pdata = malloc( sizeof(float) * l_pLFOAllPass->m_Nbuf );
	
	if( l_pLFOAllPass->m_pdata == NULL ) {
		free( l_pLFOAllPass );
		return NULL;
	}
	
	return (LADSPA_Handle)l_pLFOAllPass;
}

static void LFOAllPass_connect_port(
	LADSPA_Handle p_instance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata)
{
	LFOAllPass* l_pLFOAllPass = (LFOAllPass*)p_instance;
	l_pLFOAllPass->m_pport[p_port] = p_pdata;
}

static void LFOAllPass_activate( LADSPA_Handle p_instance )
{
	LFOAllPass* l_pLFOAllPass = (LFOAllPass*)p_instance;
	l_pLFOAllPass->m_lfo_theta=0.0;
	l_pLFOAllPass->m_write_index = 0;
	long i;
	for(i=0;i<l_pLFOAllPass->m_Nbuf;i++){
		l_pLFOAllPass->m_pdata[i]=0.0;
	}
}

static void LFOAllPass_run( LADSPA_Handle p_instance, unsigned long p_sample_count )
{
	LFOAllPass* l_pLFOAllPass = (LFOAllPass*)p_instance;
	LADSPA_Data *l_psrc = l_pLFOAllPass->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pLFOAllPass->m_pport[PORT_OUT];
	long l_sample;
    float l_dtheta = 2.0f * M_PIf * *l_pLFOAllPass->m_pport[PORT_LFO_FREQUENCY] / l_pLFOAllPass->m_sample_rate;
	float l_g = *l_pLFOAllPass->m_pport[PORT_FEEDBACK];
	
	for( l_sample=0;l_sample<p_sample_count;l_sample++){
		float  l_delay = *l_pLFOAllPass->m_pport[PORT_DELAY] * l_pLFOAllPass->m_sample_rate;
		l_delay *= (1.0f + sinf( l_pLFOAllPass->m_lfo_theta ) * *l_pLFOAllPass->m_pport[PORT_LFO_AMOUNT]);
		if(l_delay<FadNwindow()/2)l_delay=FadNwindow()/2;
		long l_delay_int = (long)ceilf(l_delay);
		float l_delay_frac = l_delay_int - l_delay;
        long l_wet_index = l_pLFOAllPass->m_write_index - FadNwindow()/2 - 1 - l_delay_int;
		if( l_wet_index < 0 )
			l_wet_index += l_pLFOAllPass->m_Nbuf;
		LADSPA_Data l_H = FadSample( l_pLFOAllPass->m_pdata, l_wet_index, l_pLFOAllPass->m_Nbuf, l_delay_frac );
		LADSPA_Data l_m = *l_psrc + l_H*l_g;
		if(l_m>1.0f)l_m=1.0f;
		if(l_m<-1.0f)l_m=-1.0f;
		*l_pdst = l_H - l_m*l_g;
		// write the incoming data to the cyclic buffer
		l_pLFOAllPass->m_pdata[l_pLFOAllPass->m_write_index] = l_m;
		// update the pointers and write index
		l_psrc++;
		l_pdst++;
		l_pLFOAllPass->m_write_index++;
		if( l_pLFOAllPass->m_write_index == l_pLFOAllPass->m_Nbuf )
			l_pLFOAllPass->m_write_index = 0;
		l_pLFOAllPass->m_lfo_theta += l_dtheta;
        if( l_pLFOAllPass->m_lfo_theta >= 2.0f*M_PIf )
            l_pLFOAllPass->m_lfo_theta -= 2.0f*M_PIf;
	}
}

static void LFOAllPass_cleanup( LADSPA_Handle p_instance )
{
	LFOAllPass* l_pLFOAllPass = (LFOAllPass*)p_instance;
	
	free( l_pLFOAllPass->m_pdata );
	free( l_pLFOAllPass );
}

static LADSPA_PortDescriptor LFOAllPass_PortDescriptors[]=
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* LFOAllPass_PortNames[]=
{
	"Input",
	"Output",
	"Delay",
	"Feedback",
	"LFO Frequency",
	"LFO Amount"
};

static LADSPA_PortRangeHint LFOAllPass_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_LOW,
		0.0, DELAY_MAX
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

LADSPA_Descriptor LFOAllPass_Descriptor=
{
	5807,
	"lfo_allpass",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"AllPass with LFO",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	LFOAllPass_PortDescriptors,
	LFOAllPass_PortNames,
	LFOAllPass_PortRangeHints,
	NULL,
	LFOAllPass_instantiate,
	LFOAllPass_connect_port,
	LFOAllPass_activate,
	LFOAllPass_run,
	NULL,
	NULL,
	NULL,
	LFOAllPass_cleanup
};
