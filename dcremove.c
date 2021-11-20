/*
 * DC remove - Removes a bias in the signal.
 *
 * h(z) = 1 - z^-1
 *        ------------
 *        1 - a1*z^-1
 *
 * a1 = cos(w) - sqrt(cos(w)^2 - 4*cos(w) + 3)
 * w = 2*pi*fc/fs
 *
 */

#include <ladspa.h>
#include <math.h>
#include <stdlib.h>

enum {
	PORT_IN,
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_NPORTS
};

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data  *m_pport[PORT_NPORTS];
	LADSPA_Data   m_xz;
	LADSPA_Data   m_yz;
} DCRemove_Data;

static LADSPA_Handle DCRemove_instantiate(
		const struct _LADSPA_Descriptor *p_pDescriptor,
		unsigned long p_sample_rate)
{
	DCRemove_Data *l_pData = malloc(sizeof(DCRemove_Data));
	if(l_pData){
		l_pData->m_sample_rate = p_sample_rate;
		l_pData->m_xz = 0.0f;
		l_pData->m_yz = 0.0f;
	}
	return (LADSPA_Handle)l_pData;
}

static void DCRemove_connect_port(
		LADSPA_Handle p_instance,
		unsigned long p_port,
		LADSPA_Data *p_pData)
{
	DCRemove_Data *l_pData = (DCRemove_Data*)p_instance;
	l_pData->m_pport[p_port] = p_pData;
}

static void DCRemove_activate(LADSPA_Handle p_instance)
{
	DCRemove_Data *l_pData = (DCRemove_Data*)p_instance;
	l_pData->m_xz = 0.0f;
	l_pData->m_yz = 0.0f;
}

static void DCRemove_run(LADSPA_Handle p_instance, unsigned long p_sample_count)
{
	DCRemove_Data *l_pData = (DCRemove_Data*)p_instance;
	LADSPA_Data l_omega = 2*M_PI* *l_pData->m_pport[PORT_FREQUENCY]/ l_pData->m_sample_rate;
	LADSPA_Data l_cos = cosf(l_omega);
	LADSPA_Data l_a1 = l_cos - sqrtf(l_cos*l_cos - 4.0f*l_cos +3.0f);
	LADSPA_Data *l_psrc = l_pData->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = l_pData->m_pport[PORT_OUT];
	for(unsigned long n=p_sample_count;n;n--){
		*l_pdst = *l_psrc - l_pData->m_xz + l_a1*l_pData->m_yz;
		l_pData->m_xz = *l_psrc;
		l_pData->m_yz = *l_pdst;
		l_psrc++;
		l_pdst++;
	}
}

static void DCRemove_deactivate(LADSPA_Handle p_instance)
{

}

static void DCRemove_cleanup(LADSPA_Handle p_instance)
{
	free(p_instance);
}

static LADSPA_PortDescriptor DCRemove_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char* DCRemove_PortNames[]=
{
	"Input",
	"Output",
	"Frequency"
};

static LADSPA_PortRangeHint DCRemove_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		1,10.0}
};

LADSPA_Descriptor DCRemove_Descriptor=
{
	5802,
	"DC_Remove",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"DC Remove",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	DCRemove_PortDescriptors,
	DCRemove_PortNames,
	DCRemove_PortRangeHints,
	NULL,
	DCRemove_instantiate,
	DCRemove_connect_port,
	DCRemove_activate,
	DCRemove_run,
	NULL,
	NULL,
	DCRemove_deactivate,
	DCRemove_cleanup
};
