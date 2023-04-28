#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

#define N_WINDOW 32
#define N_SS     1024

enum {
	PORT_OUT,
	PORT_FREQUENCY,
	PORT_AMPLITUDE,
	PORT_NPORTS
};

static LADSPA_PortDescriptor ImpulseGen_PortDescriptors[]=
{
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* ImpulseGen_PortNames[]=
{
	"Output",
	"Frequency(Hz)",
    "Amplitude(dBFS)"
};

static LADSPA_PortRangeHint ImpulseGen_PortRangeHints[]=
{
	{0, 0.0f, 0.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_LOW,
		2.0f, 2000.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_MAXIMUM,
        -140.0f, 12.0f}
};

static LADSPA_Data sinc(LADSPA_Data x)
{
	if(fabs(x)<1e-9f){
		return 1.0f;
	}else{
		return sinf(x)/x;
	}
}

static LADSPA_Data hamming(LADSPA_Data alpha)
{
    return 0.54f + 0.46f*cosf(M_PIf*alpha);
}

typedef struct
{
    LADSPA_Data  m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
    LADSPA_Data  m_impulse_data[N_SS][N_WINDOW];
    LADSPA_Data  m_accumulator[N_WINDOW];
	int m_i_acc;
	LADSPA_Data m_Tacc;
	LADSPA_Data m_Tperiod;
} ImpulseGen;

static LADSPA_Handle ImpulseGen_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	ImpulseGen *p_pImpulseGen = malloc( sizeof(ImpulseGen) );
	if(!p_pImpulseGen)
		return NULL;
	p_pImpulseGen->m_sample_rate = p_sample_rate;
	p_pImpulseGen->m_i_acc = 0;
	p_pImpulseGen->m_Tacc = 0.0f;
	p_pImpulseGen->m_Tperiod = 0.0f;
	for(int i_ss=0;i_ss<N_SS;i_ss++){
		LADSPA_Data l_alpha = (LADSPA_Data)i_ss/N_SS;
		for(int i_window=0;i_window<N_WINDOW;i_window++){
			LADSPA_Data l_x = (LADSPA_Data)(i_window-(N_WINDOW/2))-l_alpha;
            LADSPA_Data l_alpha_window = (LADSPA_Data)(i_window-(N_WINDOW/2))/(N_WINDOW/2);
            p_pImpulseGen->m_impulse_data[i_ss][i_window] = sinc(M_PIf*l_x)*hamming(l_alpha_window);
		}
	}
	for(int i_window=0;i_window<N_WINDOW;i_window++){
		p_pImpulseGen->m_accumulator[i_window] = 0.0f;
	}
	return (LADSPA_Handle)p_pImpulseGen;
}

static void ImpulseGen_impulse(ImpulseGen *p_pImpulseGen, LADSPA_Data p_alpha){
	int l_i_ss = (int)(p_alpha*N_SS);
	LADSPA_Data *l_p_impulse = &p_pImpulseGen->m_impulse_data[l_i_ss][0];
	LADSPA_Data *l_p_acc = &p_pImpulseGen->m_accumulator[p_pImpulseGen->m_i_acc];
	int l_N_loop1 = N_WINDOW - p_pImpulseGen->m_i_acc;
	int l_N_loop2 = N_WINDOW - l_N_loop1;
	for(int i=0;i<l_N_loop1;i++){
		*(l_p_acc++) += *(l_p_impulse++);
	}
	if(l_N_loop2){
		l_p_acc = p_pImpulseGen->m_accumulator;
		for(int i=0;i<l_N_loop2;i++){
			*(l_p_acc++) += *(l_p_impulse++);
		}
	}
}

static LADSPA_Data ImpulseGen_evaluate(ImpulseGen *p_pImpulseGen){
	LADSPA_Data l_deltaT = p_pImpulseGen->m_Tacc - p_pImpulseGen->m_Tperiod;
	if(l_deltaT<=-1.0f){
		p_pImpulseGen->m_Tacc += 1.0f;
	}else if(l_deltaT<=0.0f){
		ImpulseGen_impulse(p_pImpulseGen,-l_deltaT);
		p_pImpulseGen->m_Tacc = 1.0f + l_deltaT;
	}else{
		ImpulseGen_impulse(p_pImpulseGen, 0.0f);
		p_pImpulseGen->m_Tacc = 0.0f;
	}
	LADSPA_Data l_result = p_pImpulseGen->m_accumulator[p_pImpulseGen->m_i_acc];
	p_pImpulseGen->m_accumulator[p_pImpulseGen->m_i_acc] = 0.0f;
	p_pImpulseGen->m_i_acc++;
	p_pImpulseGen->m_i_acc%=N_WINDOW;
	return l_result;
}

static void ImpulseGen_connect_port(
	LADSPA_Handle p_pinstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata )
{
	ImpulseGen *p_pImpulseGen = (ImpulseGen*)p_pinstance;
	p_pImpulseGen->m_pport[p_port] = p_pdata;
}

static void ImpulseGen_run(
	LADSPA_Handle p_pinstance,
	unsigned long p_sample_count )
{
	ImpulseGen *p_pImpulseGen = (ImpulseGen*)p_pinstance;

	LADSPA_Data l_frequency = *p_pImpulseGen->m_pport[PORT_FREQUENCY];
    LADSPA_Data l_amp = exp10f(
                *p_pImpulseGen->m_pport[PORT_AMPLITUDE]/20.0f);

    p_pImpulseGen->m_Tperiod = p_pImpulseGen->m_sample_rate/l_frequency;
	
	long sample;
	LADSPA_Data *l_pdst = p_pImpulseGen->m_pport[PORT_OUT];
	
	for(sample=p_sample_count;sample!=0;sample--){
		*(l_pdst++) = ImpulseGen_evaluate(p_pImpulseGen)*l_amp;
	}
}

static void ImpulseGen_cleanup(
	LADSPA_Handle p_pinstance )
{
	free( p_pinstance );
}

LADSPA_Descriptor ImpulseGen_Descriptor=
{
	5805,
	"ImpulseGen",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"ImpulseGen",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	ImpulseGen_PortDescriptors,
	ImpulseGen_PortNames,
	ImpulseGen_PortRangeHints,
	NULL,
	ImpulseGen_instantiate,
	ImpulseGen_connect_port,
	NULL,
	ImpulseGen_run,
	NULL,
	NULL,
	NULL,
	ImpulseGen_cleanup
};
