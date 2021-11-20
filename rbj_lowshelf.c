#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>
/*			(b0/a0) + (b1/a0)*z^-1 + (b2/a0)*z^-2
 *   H(z) = ---------------------------------------
 *              1 + (a1/a0)*z^-1 + (a2/a0)*z^-2
 * 
 *	b0 =    A*( (A+1) - (A-1)*cos(w0) + beta*sin(w0) )
 *	b1 =  2*A*( (A-1) - (A+1)*cos(w0) )
 *	b2 =    A*( (A+1) - (A-1)*cos(w0) - beta*sin(w0) )
 *	a0 =        (A+1) + (A-1)*cos(w0) + beta*sin(w0)
 *	a1 =   -2*( (A-1) + (A+1)*cos(w0) )
 *	a2 =        (A+1) + (A-1)*cos(w0) - beta*sin(w0)
 * 
 * A = 10^(dBgain/40)
 * w0 = 2 * pi * fc / fs
 * beta = sqrt((A^2+1)/S - (A-1)^2)'
 * S = 1dB/octave
 */

#define SLOPE 1.0f

enum {
	PORT_IN,
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_GAIN,
	PORT_NPORTS
};

typedef struct {
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
	LADSPA_Data m_log2d2;
} LowShelf_Data;

static LADSPA_Handle LowShelf_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate ){
	LowShelf_Data *l_pLowShelf = malloc( sizeof(LowShelf_Data) );
	if( l_pLowShelf ){
		l_pLowShelf->m_sample_rate = p_sample_rate;
		l_pLowShelf->m_z1 = 0.0;
		l_pLowShelf->m_z2 = 0.0;
		l_pLowShelf->m_log2d2 = logf(2.0)/2.0;
	}
	return (LADSPA_Handle)l_pLowShelf;
}

static void LowShelf_connect_port(
	LADSPA_Handle p_pInstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata)
{
	LowShelf_Data *l_pLowShelf = (LowShelf_Data*)p_pInstance;
	l_pLowShelf->m_pport[p_port] = p_pdata;
}

static void LowShelf_run(
	LADSPA_Handle p_pInstance,
	unsigned long p_sample_count)
{
	LowShelf_Data *l_pLowShelf = (LowShelf_Data*)p_pInstance;
	unsigned long l_sample;
	LADSPA_Data *l_psrc = l_pLowShelf->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pLowShelf->m_pport[PORT_OUT];
	LADSPA_Data l_omega = 2.0*M_PI* *l_pLowShelf->m_pport[PORT_FREQUENCY]/
	l_pLowShelf->m_sample_rate;
	LADSPA_Data l_A = exp10f( *l_pLowShelf->m_pport[PORT_GAIN] / 40.0 );
	LADSPA_Data l_beta = sqrt((l_A*l_A+1.0f)/SLOPE - (l_A-1.0f)*(l_A-1.0f));
	LADSPA_Data l_cos = cosf(l_omega);
	LADSPA_Data l_sin = sinf(l_omega);
	LADSPA_Data l_a0 =         (l_A+1) + (l_A-1)*l_cos + l_beta*l_sin;
	LADSPA_Data l_a1 =    -2*( (l_A-1) + (l_A+1)*l_cos )/l_a0;
	LADSPA_Data l_a2 =       ( (l_A+1) + (l_A-1)*l_cos - l_beta*l_sin)/l_a0;
	LADSPA_Data l_b0 =   l_A*( (l_A+1) - (l_A-1)*l_cos + l_beta*l_sin)/l_a0;
	LADSPA_Data l_b1 = 2*l_A*( (l_A-1) - (l_A+1)*l_cos )/l_a0;
	LADSPA_Data l_b2 =   l_A*( (l_A+1) - (l_A-1)*l_cos - l_beta*l_sin)/l_a0;
	for(l_sample=0;l_sample<p_sample_count;l_sample++){
		LADSPA_Data l_m = *l_psrc - l_a1*l_pLowShelf->m_z1 - l_a2*l_pLowShelf->m_z2;
		*l_pdst = l_b0*l_m + l_b1*l_pLowShelf->m_z1 + l_b2*l_pLowShelf->m_z2;
		l_pLowShelf->m_z2 = l_pLowShelf->m_z1;
		l_pLowShelf->m_z1 = l_m;
		l_psrc++;
		l_pdst++;
	}
}

static void LowShelf_cleanup( LADSPA_Handle p_pInstance )
{
	free( p_pInstance );
}

static LADSPA_PortDescriptor LowShelf_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char *LowShelf_PortNames[]=
{
	"Input",
	"Output",
	"Frequency(Hz)",
	"GAIN(dB)"
};

static LADSPA_PortRangeHint LowShelf_PortRangeHints[]=
{
	{0,0,0},
	{0,0,0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
		10.0,2000.0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_0,
		-60.0,60.0}
};

LADSPA_Descriptor RBJLowShelf_Descriptor=
{
	5822,
	"RBJ_lowshelf",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"LowShelf RBJ",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	LowShelf_PortDescriptors,
	LowShelf_PortNames,
	LowShelf_PortRangeHints,
	NULL,
	LowShelf_instantiate,
	LowShelf_connect_port,
	NULL,
	LowShelf_run,
	NULL,
	NULL,
	NULL,
	LowShelf_cleanup
};
