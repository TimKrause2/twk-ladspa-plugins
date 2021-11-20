#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>
/*			(b0/a0) + (b1/a0)*z^-1 + (b2/a0)*z^-2
 *   H(z) = ---------------------------------------
 *              1 + (a1/a0)*z^-1 + (a2/a0)*z^-2
 * 
 * b0 = alpha
 * b1 = 0
 * b2 = -alpha
 * a0 = 1 + alpha
 * a1 = -2 cos(omega)
 * a2 = 1 - alpha
 * 
 * omega = 2 * pi * fc / fs
 * alpha = sin(omega) * sinh( log(2)/2 * bw * omega / sin(omega) )
 *       = sin(omega) / (2*Q)
 */

enum {
	PORT_IN,
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_BANDWIDTH,
	PORT_GAIN,
	PORT_NPORTS
};

typedef struct {
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
	LADSPA_Data m_log2d2;
} Bandpass_Data;

static LADSPA_Handle Bandpass_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate ){
	Bandpass_Data *l_pBandpass = malloc( sizeof(Bandpass_Data) );
	if( l_pBandpass ){
		l_pBandpass->m_sample_rate = p_sample_rate;
		l_pBandpass->m_z1 = 0.0;
		l_pBandpass->m_z2 = 0.0;
		l_pBandpass->m_log2d2 = logf(2.0)/2.0;
	}
	return (LADSPA_Handle)l_pBandpass;
}

static void Bandpass_connect_port(
	LADSPA_Handle p_pInstance,
	unsigned long p_port,
	LADSPA_Data *p_pdata)
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
	
	LADSPA_Data l_omega = 2.0*M_PI* *l_pBandpass->m_pport[PORT_FREQUENCY] /
	l_pBandpass->m_sample_rate;
	LADSPA_Data l_sin_omega = sinf( l_omega );
	LADSPA_Data l_alpha = l_sin_omega*sinhf(l_pBandpass->m_log2d2 *
	*l_pBandpass->m_pport[PORT_BANDWIDTH] * l_omega/l_sin_omega);
	LADSPA_Data l_a0 = 1.0 + l_alpha;
	LADSPA_Data l_a1 = -2.0 * cosf( l_omega ) / l_a0;
	LADSPA_Data l_a2 = (1.0 - l_alpha) / l_a0;
	LADSPA_Data l_b0 = l_alpha / l_a0;
	LADSPA_Data l_b2 = -l_alpha / l_a0;
	LADSPA_Data l_G = exp10f( *l_pBandpass->m_pport[PORT_GAIN] / 20.0 );
	for(l_sample=0;l_sample<p_sample_count;l_sample++){
		LADSPA_Data l_m = *l_psrc - l_a1*l_pBandpass->m_z1 - l_a2*l_pBandpass->m_z2;
		*l_pdst = l_G*(l_m*l_b0 + l_pBandpass->m_z2*l_b2);
		l_pBandpass->m_z2 = l_pBandpass->m_z1;
		l_pBandpass->m_z1 = l_m;
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
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char *Bandpass_PortNames[]=
{
	"Input",
	"Output",
	"Frequency(Hz)",
	"Bandwidth(octaves)",
	"Gain(dB)"
};

static LADSPA_PortRangeHint Bandpass_PortRangeHints[]=
{
	{0,0,0},
	{0,0,0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_SAMPLE_RATE,
		10.0/44100.0,0.45},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE,
		0.1/12.0,2.0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_0,
		-60,24}
};

LADSPA_Descriptor RBJBandpassBW_Descriptor=
{
	5815,
	"RBJ_bandpass_BW",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Bandpass RBJ(BW)",
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
