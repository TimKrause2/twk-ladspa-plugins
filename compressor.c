#include <ladspa.h>
#include <math.h>
#include <stdlib.h>

enum {
	PORT_RX1,
	PORT_RX2,
	PORT_TX1,
	PORT_TX2,
	PORT_UNITY,
	PORT_RATIO_HI,
	PORT_RATIO_LO,
	PORT_THRESHOLD,
	PORT_ATTACK,
	PORT_DECAY,
	PORT_NPORTS
};

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data* m_pdata[PORT_NPORTS];
	LADSPA_Data  m_env1;
	LADSPA_Data  m_env2;
} Compressor_Data;


static LADSPA_Handle Compressor_instantiate(
	const struct _LADSPA_Descriptor* p_pDescriptor,
	unsigned long SampleRate)
{
	Compressor_Data* l_pData = malloc(sizeof(Compressor_Data));
	if(l_pData){
		l_pData->m_sample_rate = SampleRate;
	}
	return (LADSPA_Handle)l_pData;
}

static void Compressor_connect_port(
	LADSPA_Handle p_instance,
	unsigned long p_port,
	LADSPA_Data*  p_pdata)
{
	Compressor_Data* m_pData = (Compressor_Data*)p_instance;
	m_pData->m_pdata[p_port] = p_pdata;
}

#define ABS_MIN 1.19209290e-7

static void Compressor_activate(LADSPA_Handle p_instance)
{
	Compressor_Data* l_pData = (Compressor_Data*)p_instance;
	l_pData->m_env1 = ABS_MIN;
	l_pData->m_env2 = ABS_MIN;
}

static void buffer_compress(
	LADSPA_Data* p_psrc,
	LADSPA_Data* p_pdst,
	LADSPA_Data* p_penv,
	LADSPA_Data  p_unity,
	LADSPA_Data  p_ratio_hi,
	LADSPA_Data  p_ratio_lo,
	LADSPA_Data  p_threshold,
	LADSPA_Data  p_alpha_attack,
	LADSPA_Data  p_alpha_decay,
	unsigned long p_nsamples )
{
	unsigned long n;
	LADSPA_Data *l_psrc = p_psrc;
	LADSPA_Data *l_pdst = p_pdst;
	for(n=0;n<p_nsamples;n++,l_psrc++,l_pdst++){
		LADSPA_Data l_in = *l_psrc;
		l_in = fabsf(l_in);
		LADSPA_Data l_delta = l_in - *p_penv;
		if(l_delta>0.0){
			*p_penv+=l_delta*p_alpha_attack;
		}else{
			*p_penv+=l_delta*p_alpha_decay;
		}
		if(*p_penv<ABS_MIN){
			*l_pdst=0.0;
		}else{
			LADSPA_Data l_env_db = log10f(*p_penv)*20.0;
			if(l_env_db<p_threshold){
				LADSPA_Data l_comp_env_db = p_threshold - p_unity;
				l_comp_env_db /= p_ratio_hi;
				LADSPA_Data l_exp_env_db = l_env_db - p_threshold;
				l_exp_env_db /= p_ratio_lo;
				l_env_db = p_unity + l_comp_env_db + l_exp_env_db - l_env_db;
			}else{
				LADSPA_Data l_comp_env_db = l_env_db - p_unity;
				l_comp_env_db /= p_ratio_hi;
				l_env_db = p_unity + l_comp_env_db - l_env_db;
			}
			LADSPA_Data l_gain_factor = powf(10.0,l_env_db/20.0);
			*l_pdst = *l_psrc * l_gain_factor;
		}
	}
}

static void Compressor_run(LADSPA_Handle p_instance, unsigned long SampleCount)
{
	Compressor_Data* l_pData = (Compressor_Data*)p_instance;
	unsigned long n;
// 	LADSPA_Data l_alpha_attack = 1.0 - expf(-1.0 / *l_pData->m_pdata[PORT_ATTACK] / l_pData->m_sample_rate);
// 	LADSPA_Data l_alpha_decay = 1.0 - expf(-1.0 / *l_pData->m_pdata[PORT_DECAY] / l_pData->m_sample_rate);
	LADSPA_Data l_alpha_attack = 1.0 - powf(0.05,1.0 / *l_pData->m_pdata[PORT_ATTACK] / l_pData->m_sample_rate);
	LADSPA_Data l_alpha_decay = 1.0 - powf(0.05,1.0 / *l_pData->m_pdata[PORT_DECAY] / l_pData->m_sample_rate);

	buffer_compress(
		l_pData->m_pdata[PORT_RX1],
		l_pData->m_pdata[PORT_TX1],
		&l_pData->m_env1,
		*l_pData->m_pdata[PORT_UNITY],
		*l_pData->m_pdata[PORT_RATIO_HI],
		*l_pData->m_pdata[PORT_RATIO_LO],
		*l_pData->m_pdata[PORT_THRESHOLD],
		l_alpha_attack,
		l_alpha_decay,
		SampleCount );
	
	buffer_compress(
		l_pData->m_pdata[PORT_RX2],
		l_pData->m_pdata[PORT_TX2],
		&l_pData->m_env2,
		*l_pData->m_pdata[PORT_UNITY],
		*l_pData->m_pdata[PORT_RATIO_HI],
		*l_pData->m_pdata[PORT_RATIO_LO],
		*l_pData->m_pdata[PORT_THRESHOLD],
		l_alpha_attack,
		l_alpha_decay,
		SampleCount );
	
}

static void Compressor_deactivate(LADSPA_Handle p_instance)
{
}

static void Compressor_cleanup(LADSPA_Handle p_instance)
{
	free(p_instance);
}

static LADSPA_PortDescriptor Compressor_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,

	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,

	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char* Compressor_PortNames[]=
{
	"Input1",
	"Input2",
	"Output1",
	"Output2",
	"Unity",
	"RatioHi",
	"RatioLo",
	"Threshold",
	"Attack",
	"Decay"
};

static LADSPA_PortRangeHint Compressor_PortRangeHints[]=
{
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{0, 0.0, 0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_DEFAULT_MAXIMUM|
		LADSPA_HINT_BOUNDED_ABOVE,
		-96.0,0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.01,100.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.01,100.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_DEFAULT_MINIMUM|
		LADSPA_HINT_BOUNDED_ABOVE,
		-140.0,0.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.001,5.0},
	{
		LADSPA_HINT_BOUNDED_BELOW|
		LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_DEFAULT_MIDDLE|
		LADSPA_HINT_LOGARITHMIC,
		0.001,5.0}
};

LADSPA_Descriptor Compressor_Descriptor=
{
	5801,
	"compressor",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Compressor",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Compressor_PortDescriptors,
	Compressor_PortNames,
	Compressor_PortRangeHints,
	NULL,
	Compressor_instantiate,
	Compressor_connect_port,
	Compressor_activate,
	Compressor_run,
	NULL,
	NULL,
	Compressor_deactivate,
	Compressor_cleanup
};
