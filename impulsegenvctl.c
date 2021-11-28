#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#define N_WINDOW 32
#define N_SS     1024

#define F_MIN    85.0f
#define F_MAX    450.0f

#define F_LOPASS 400.0f
#define Q_LOPASS 1.0f
#define ALPHA_DC 0.95f

enum {
	PORT_IN,
	PORT_OUT,
	PORT_AMPLITUDE,
	PORT_NPORTS
};

static LADSPA_PortDescriptor ImpulseGenVC_PortDescriptors[]=
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* ImpulseGenVC_PortNames[]=
{
	"Input",
	"Output",
	"Amplitude"
};

static LADSPA_PortRangeHint ImpulseGenVC_PortRangeHints[]=
{
	{0, 0.0f, 0.0f},
	{0, 0.0f, 0.0f},
	{ LADSPA_HINT_BOUNDED_BELOW |
		LADSPA_HINT_BOUNDED_ABOVE |
		LADSPA_HINT_DEFAULT_MAXIMUM,
		0.0f, 1.0f}
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
	return 0.54f + 0.46f*cosf(M_PI*alpha);
}

typedef struct
{
	LADSPA_Data m_z1;
} DCRemove;

typedef struct
{
	LADSPA_Data m_z1;
	LADSPA_Data m_z2;
	LADSPA_Data m_a1;
	LADSPA_Data m_a2;
	LADSPA_Data m_b0;
	LADSPA_Data m_b1;
	LADSPA_Data m_b2;
} LoPass;

typedef struct
{
	LADSPA_Data *m_x;
	LADSPA_Data *m_cor;
	int m_N_window;
	int m_N_cor;
	int m_i_hi;
	int m_i_x;
} PitchEstimator;

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_NPORTS];
	LADSPA_Data m_impulse_data[N_SS][N_WINDOW];
	LADSPA_Data m_accumulator[N_WINDOW];
	int m_i_acc;
	LADSPA_Data m_Tacc;
	LADSPA_Data m_Tperiod;
	DCRemove m_dc;
	LoPass   m_lp;
	PitchEstimator  m_pitch[2];
} ImpulseGen;

static LADSPA_Data DCRemove_evaluate(DCRemove *p_pdc, LADSPA_Data p_x)
{
	LADSPA_Data l_m = p_x + ALPHA_DC*p_pdc->m_z1;
	LADSPA_Data l_r = l_m - p_pdc->m_z1;
	p_pdc->m_z1 = l_m;
	return l_r;
}

static LADSPA_Data LoPass_evaluate(LoPass *p_plp, LADSPA_Data p_x)
{
	LADSPA_Data l_m = p_x - p_plp->m_a1*p_plp->m_z1 - p_plp->m_a2*p_plp->m_z2;
	LADSPA_Data l_r = p_plp->m_b0*l_m + p_plp->m_b1*p_plp->m_z1 + p_plp->m_b2*p_plp->m_z2;
	p_plp->m_z2 = p_plp->m_z1;
	p_plp->m_z1 = l_m;
	return l_r;
}

static void PitchEstimator_init(PitchEstimator *p_pPitch,
						 LADSPA_Data p_f_lo,
						 LADSPA_Data p_f_hi,
						 unsigned long p_sample_rate)
{
	p_pPitch->m_N_window = (int)((LADSPA_Data)p_sample_rate/p_f_lo);
	p_pPitch->m_i_hi = (int)((LADSPA_Data)p_sample_rate/p_f_hi);
	p_pPitch->m_N_cor = p_pPitch->m_N_window - p_pPitch->m_i_hi + 1;
	p_pPitch->m_x = malloc(sizeof(LADSPA_Data)*p_pPitch->m_N_window*2);
	p_pPitch->m_cor = malloc(sizeof(LADSPA_Data)*p_pPitch->m_N_cor);
	p_pPitch->m_i_x = 0;
	for(int i=0;i<p_pPitch->m_N_window*2;i++){
		p_pPitch->m_x[i] = 0.0f;
	}
}

static LADSPA_Data PitchEstimator_evaluate(PitchEstimator *p_pPitch, LADSPA_Data p_x)
{
	p_pPitch->m_x[p_pPitch->m_i_x++] = p_x;
	if(p_pPitch->m_i_x == p_pPitch->m_N_window*2){
		p_pPitch->m_i_x = 0;
		// auto-correlation of zero displacement
		LADSPA_Data *l_px0 = p_pPitch->m_x;
		LADSPA_Data *l_px1 = p_pPitch->m_x;
		LADSPA_Data l_cor0 = 0.0f;
		for(int i=p_pPitch->m_N_window;i;i--){
			l_cor0 += *(l_px0++) * *(l_px1++);
		}
		if(l_cor0==0.0f) return 0.0f;
		// auto-correlation scan
		int l_i = p_pPitch->m_i_hi;
		LADSPA_Data *l_pcor = p_pPitch->m_cor;
		for(int i=p_pPitch->m_N_cor;i;i--,l_i++,l_pcor++){
			l_px0 = p_pPitch->m_x;
			l_px1 = &p_pPitch->m_x[l_i];
			*l_pcor = 0.0f;
			for(int j=p_pPitch->m_N_window;j;j--){
				*l_pcor += *(l_px0++) * *(l_px1++);
			}
			*l_pcor /= l_cor0;
		}
		// peak detector
		LADSPA_Data l_peak_level=p_pPitch->m_cor[0];
		int l_peak_i=0;
		for(int i=1;i<p_pPitch->m_N_cor-1;i++){
			if(p_pPitch->m_cor[i] > l_peak_level){
				l_peak_level = p_pPitch->m_cor[i];
				l_peak_i = i;
			}
		}
		return (LADSPA_Data)(l_peak_i + p_pPitch->m_i_hi);
	}else{
		return 0.0f;
	}
	return 0.0f;
}


static void PitchEstimator_destroy(PitchEstimator *p_pPitch)
{
	free(p_pPitch->m_x);
	free(p_pPitch->m_cor);
}

static LADSPA_Handle ImpulseGenVC_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	ImpulseGen *p_pImpulseGen = malloc( sizeof(ImpulseGen) );
	if(!p_pImpulseGen)
		return NULL;
	p_pImpulseGen->m_sample_rate = p_sample_rate;
	p_pImpulseGen->m_i_acc = 0;
	p_pImpulseGen->m_Tacc = 0.0f;
	p_pImpulseGen->m_Tperiod = (LADSPA_Data)p_sample_rate/100.0f;
	for(int i_ss=0;i_ss<N_SS;i_ss++){
		LADSPA_Data l_alpha = (LADSPA_Data)i_ss/N_SS;
		for(int i_window=0;i_window<N_WINDOW;i_window++){
			LADSPA_Data l_x = (LADSPA_Data)(i_window-(N_WINDOW/2))-l_alpha;
			LADSPA_Data l_alpha = (LADSPA_Data)(i_window-(N_WINDOW/2))/(N_WINDOW/2);
			p_pImpulseGen->m_impulse_data[i_ss][i_window] = sinc(M_PI*l_x)*hamming(l_alpha);
		}
	}
	for(int i_window=0;i_window<N_WINDOW;i_window++){
		p_pImpulseGen->m_accumulator[i_window] = 0.0f;
	}

	p_pImpulseGen->m_dc.m_z1 = 0.0f;

	p_pImpulseGen->m_lp.m_z1 = 0.0f;
	p_pImpulseGen->m_lp.m_z2 = 0.0f;
	LADSPA_Data l_omega = 2.0*M_PI*F_LOPASS/p_sample_rate;
	LADSPA_Data l_sin_omega = sinf(l_omega);
	LADSPA_Data l_cos_omega = cosf(l_omega);
	LADSPA_Data l_alpha = l_sin_omega / ( 2.0f * Q_LOPASS);
	LADSPA_Data l_a0 = 1.0f + l_alpha;
	p_pImpulseGen->m_lp.m_a1 = -2.0f * l_cos_omega / l_a0;
	p_pImpulseGen->m_lp.m_a2 = (1.0f - l_alpha) / l_a0;
	p_pImpulseGen->m_lp.m_b0 = (1.0f - l_cos_omega) / 2.0f / l_a0;
	p_pImpulseGen->m_lp.m_b1 = (1.0f - l_cos_omega) / l_a0;
	p_pImpulseGen->m_lp.m_b2 = (1.0f - l_cos_omega) / 2.0f / l_a0;

	PitchEstimator_init(&p_pImpulseGen->m_pitch[0],F_MIN,F_MAX,p_sample_rate);
	PitchEstimator_init(&p_pImpulseGen->m_pitch[1],F_MIN,F_MAX,p_sample_rate);
	p_pImpulseGen->m_pitch[1].m_i_x = p_pImpulseGen->m_pitch[1].m_N_window;
	return (LADSPA_Handle)p_pImpulseGen;
}

static void ImpulseGenVC_impulse(ImpulseGen *p_pImpulseGen, LADSPA_Data p_alpha){
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

static LADSPA_Data ImpulseGenVC_evaluate(ImpulseGen *p_pImpulseGen){
	LADSPA_Data l_deltaT = p_pImpulseGen->m_Tacc - p_pImpulseGen->m_Tperiod;
	if(l_deltaT<=-1.0f){
		p_pImpulseGen->m_Tacc += 1.0f;
	}else if(l_deltaT<=0.0f){
		ImpulseGenVC_impulse(p_pImpulseGen,-l_deltaT);
		p_pImpulseGen->m_Tacc = 1.0f + l_deltaT;
	}else{
		ImpulseGenVC_impulse(p_pImpulseGen, 0.0f);
		p_pImpulseGen->m_Tacc = 0.0f;
	}
	LADSPA_Data l_result = p_pImpulseGen->m_accumulator[p_pImpulseGen->m_i_acc];
	p_pImpulseGen->m_accumulator[p_pImpulseGen->m_i_acc] = 0.0f;
	p_pImpulseGen->m_i_acc++;
	p_pImpulseGen->m_i_acc%=N_WINDOW;
	return l_result;
}

static void ImpulseGenVC_connect_port(
	LADSPA_Handle p_pinstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata )
{
	ImpulseGen *p_pImpulseGen = (ImpulseGen*)p_pinstance;
	p_pImpulseGen->m_pport[p_port] = p_pdata;
}

static void ImpulseGenVC_run(
	LADSPA_Handle p_pinstance,
	unsigned long p_sample_count )
{
	ImpulseGen *p_pImpulseGen = (ImpulseGen*)p_pinstance;

	LADSPA_Data l_amp = *p_pImpulseGen->m_pport[PORT_AMPLITUDE];

	long sample;
	LADSPA_Data *l_psrc = p_pImpulseGen->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = p_pImpulseGen->m_pport[PORT_OUT];
	
	for(sample=p_sample_count;sample!=0;sample--){
		LADSPA_Data l_x = DCRemove_evaluate(&p_pImpulseGen->m_dc, *(l_psrc++));
		l_x = LoPass_evaluate(&p_pImpulseGen->m_lp, l_x);
		LADSPA_Data l_f0 = PitchEstimator_evaluate(&p_pImpulseGen->m_pitch[0], l_x);
		LADSPA_Data l_f1 = PitchEstimator_evaluate(&p_pImpulseGen->m_pitch[1], l_x);
		if(l_f0!=0.0f || l_f1!=0.0f){
			if(l_f0!=0.0f){
				p_pImpulseGen->m_Tperiod = l_f0;
			}else{
				p_pImpulseGen->m_Tperiod = l_f1;
			}
		}
		*(l_pdst++) = ImpulseGenVC_evaluate(p_pImpulseGen)*l_amp;
	}
}

static void ImpulseGenVC_cleanup(
	LADSPA_Handle p_pinstance )
{
	ImpulseGen *p_pImpulseGen = (ImpulseGen*)p_pinstance;
	PitchEstimator_destroy(&p_pImpulseGen->m_pitch[0]);
	PitchEstimator_destroy(&p_pImpulseGen->m_pitch[1]);
	free( p_pinstance );
}

LADSPA_Descriptor ImpulseGenVC_Descriptor=
{
	5806,
	"ImpulseGenVCtl",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"ImpulseGen Voice Control",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	ImpulseGenVC_PortDescriptors,
	ImpulseGenVC_PortNames,
	ImpulseGenVC_PortRangeHints,
	NULL,
	ImpulseGenVC_instantiate,
	ImpulseGenVC_connect_port,
	NULL,
	ImpulseGenVC_run,
	NULL,
	NULL,
	NULL,
	ImpulseGenVC_cleanup
};
