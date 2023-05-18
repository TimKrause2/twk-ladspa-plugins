#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

enum {
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_AMPLITUDE,
	PORT_NPORTS
};

static LADSPA_PortDescriptor SineWave_PortDescriptors[]=
{
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* SineWave_PortNames[]=
{
	"Output",
	"Frequency(Hz)",
    "Amplitude(dBFS)"
};

static LADSPA_PortRangeHint SineWave_PortRangeHints[]=
{
	{0, 0.0f, 0.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_LOW,
        2.0f, 20000.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
        LADSPA_HINT_DEFAULT_0,
        -145.0f, 12.0f}
};


typedef struct
{
    LADSPA_Data m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
    LADSPA_Data  m_phase;
} SineWave;

static LADSPA_Handle SineWave_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
    SineWave *p_pSineWave = malloc( sizeof(SineWave) );
    if(!p_pSineWave)
		return NULL;
    p_pSineWave->m_sample_rate = (float)p_sample_rate;
    p_pSineWave->m_phase = 0.0f;
    return (LADSPA_Handle)p_pSineWave;
}

static void SineWave_connect_port(
	LADSPA_Handle p_pinstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata )
{
    SineWave *p_pSineWave = (SineWave*)p_pinstance;
    p_pSineWave->m_pport[p_port] = p_pdata;
}

static void SineWave_run(
	LADSPA_Handle p_pinstance,
	unsigned long p_sample_count )
{
    SineWave *p_pSineWave = (SineWave*)p_pinstance;

    LADSPA_Data l_frequency = *p_pSineWave->m_pport[PORT_FREQUENCY];
    register LADSPA_Data l_amp = exp10f(*p_pSineWave->m_pport[PORT_AMPLITUDE]/20.0f);
    register LADSPA_Data l_dphase = 2.0f*M_PIf*l_frequency/p_pSineWave->m_sample_rate;

    LADSPA_Data *l_pdst = p_pSineWave->m_pport[PORT_OUT];
    LADSPA_Data *l_pdst_end = l_pdst + p_sample_count;
    register LADSPA_Data l_phase = p_pSineWave->m_phase;
    for(;l_pdst!=l_pdst_end;l_pdst++){
        *l_pdst = sinf(l_phase)*l_amp;
        l_phase+=l_dphase;
        if(l_phase >= 2.0f*M_PIf){
            l_phase -= 2.0f*M_PIf;
        }
	}
    p_pSineWave->m_phase = l_phase;
}

static void SineWave_cleanup(
	LADSPA_Handle p_pinstance )
{
	free( p_pinstance );
}

LADSPA_Descriptor SineWave_Descriptor=
{
    5825,
    "SineOscillator",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
    "Sine Wave Oscillator",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
    SineWave_PortDescriptors,
    SineWave_PortNames,
    SineWave_PortRangeHints,
	NULL,
    SineWave_instantiate,
    SineWave_connect_port,
	NULL,
    SineWave_run,
	NULL,
	NULL,
	NULL,
    SineWave_cleanup
};
