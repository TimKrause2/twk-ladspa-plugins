/*
 * LADSPA Vocoder based on linear prediction
 * 
 */

#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

#define N_ORDER   48
#define T_WINDOW  20.0E-3f
#define F_LOWPASS 7.0e3f
#define Q_LOWPASS 0.707f

enum {
	PORT_IN_CTL,
	PORT_IN_RAW,
	PORT_OUT_0,
	PORT_OUT_1,
	PORT_N_PORTS
};

typedef struct
{
	double *m_px; // control signal buffer (m_N_window)
	double *m_pa; // filter coefficients   (N_ORDER)
	double *m_pz; // filter state          (N_ORDER)
	double *m_pw; // window coefficient pointer
	unsigned long m_i_window;
	unsigned long m_i_z;
	double m_gain;
	double m_envelope;
	double m_denvelope;
} Filter_Data;

typedef struct
{
	LADSPA_Data m_z[2];
	LADSPA_Data m_a1;
	LADSPA_Data m_a2;
	LADSPA_Data m_b0;
	LADSPA_Data m_b1;
	LADSPA_Data m_b2;
} LPFilter;

static void LPFilter_init(
		LPFilter *p_lpf,
		LADSPA_Data p_frequency,
		LADSPA_Data p_Q,
		unsigned long p_sample_rate)
{
	LADSPA_Data l_omega = 2.0f*M_PI*p_frequency/p_sample_rate;
	LADSPA_Data l_cos = cosf(l_omega);
	LADSPA_Data l_sin = sinf(l_omega);
	LADSPA_Data l_alpha = l_sin/(2*p_Q);
	LADSPA_Data l_a0 = 1.0f + l_alpha;
	p_lpf->m_a1 = -2.0f*l_cos/l_a0;
	p_lpf->m_a2 = (1.0f - l_alpha)/l_a0;
	p_lpf->m_b0 = (1.0f - l_cos)/2.0f/l_a0;
	p_lpf->m_b1 = (1.0f - l_cos)/l_a0;
	p_lpf->m_b2 = p_lpf->m_b0;
	p_lpf->m_z[0] = 0.0f;
	p_lpf->m_z[1] = 0.0f;
}

static LADSPA_Data LPFilter_evaluate(
		LPFilter *p_lpf,
		LADSPA_Data p_x)
{
	LADSPA_Data l_m = p_x - p_lpf->m_a1*p_lpf->m_z[0] - p_lpf->m_a2*p_lpf->m_z[1];
	LADSPA_Data l_r = p_lpf->m_b0*l_m + p_lpf->m_b1*p_lpf->m_z[0]
			+ p_lpf->m_b2*p_lpf->m_z[1];
	p_lpf->m_z[1] = p_lpf->m_z[0];
	p_lpf->m_z[0] = l_m;
	return l_r;
}

typedef struct
{
	unsigned long m_sample_rate;
	LADSPA_Data *m_pport[PORT_N_PORTS];
	unsigned long m_N_window;
	double m_gain; // gain coefficient from Levinson-Durbin recursion
	double *m_pdata; // pointer to the allocated memory
	double *m_pw; // window coefficients (m_N_window)
	double *m_pR; // autocorrelation coefficients (N_ORDER+1)
	double *m_palpha[N_ORDER]; // Levinson-Durbin coefficients	(N_ORDER*(N_ORDER+1)/2)
	LPFilter m_lpf;
	Filter_Data m_filters[2];
} LPVocoder_Data;

static LADSPA_Handle LPVocoder_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
	LPVocoder_Data* l_pData = malloc( sizeof(LPVocoder_Data) );
	if(!l_pData) return NULL;
	
	l_pData->m_sample_rate = p_sample_rate;
	l_pData->m_N_window = (unsigned long)ceil( (double)p_sample_rate*T_WINDOW );
	if( l_pData->m_N_window&1 ) l_pData->m_N_window++;
	LPFilter_init( &l_pData->m_lpf, F_LOWPASS, Q_LOWPASS, p_sample_rate);
	unsigned long l_n=0;
	l_n+= l_pData->m_N_window;   // m_pw
	l_n+= N_ORDER+1;             // m_pR
	l_n+= N_ORDER*(N_ORDER+1)/2; // m_palpha
	l_n+= (l_pData->m_N_window + N_ORDER*2)*2; // filters
	
	l_pData->m_pdata = malloc( sizeof(double) * l_n );
	if( !l_pData->m_pdata ){
		free( l_pData );
		return NULL;
	}
	
	l_pData->m_pw = l_pData->m_pdata;
	l_pData->m_pR = l_pData->m_pw + l_pData->m_N_window;
	
	double *l_palpha = l_pData->m_pR + N_ORDER+1;
	for( l_n=0; l_n<N_ORDER; l_n++){
		l_pData->m_palpha[l_n] = l_palpha;
		l_palpha+=l_n+1;
	}

	l_pData->m_filters[0].m_px = l_palpha;
	l_pData->m_filters[0].m_pa = l_pData->m_filters[0].m_px + l_pData->m_N_window;
	l_pData->m_filters[0].m_pz = l_pData->m_filters[0].m_pa + N_ORDER;
	
	l_pData->m_filters[1].m_px = l_pData->m_filters[0].m_pz + N_ORDER;
	l_pData->m_filters[1].m_pa = l_pData->m_filters[1].m_px + l_pData->m_N_window;
	l_pData->m_filters[1].m_pz = l_pData->m_filters[1].m_pa + N_ORDER;
	
	l_pData->m_filters[0].m_pw = l_pData->m_pw;
	l_pData->m_filters[0].m_i_window = 0;
	l_pData->m_filters[0].m_i_z = 0;
	l_pData->m_filters[0].m_gain = 0;
	l_pData->m_filters[0].m_envelope = 0;
	l_pData->m_filters[0].m_denvelope = 1/(l_pData->m_N_window/2-1);
	
	l_pData->m_filters[1].m_pw = &l_pData->m_pw[l_pData->m_N_window/2];
	l_pData->m_filters[1].m_i_window = l_pData->m_N_window/2;
	l_pData->m_filters[1].m_i_z = 0;
	l_pData->m_filters[1].m_gain = 0;
	l_pData->m_filters[1].m_envelope = 1.0;
	l_pData->m_filters[1].m_denvelope = -1/(l_pData->m_N_window/2-1);
	
	for( l_n=0; l_n<N_ORDER; l_n++){
		l_pData->m_filters[0].m_pa[l_n] = 0.0;
		l_pData->m_filters[0].m_pz[l_n] = 0.0;
		l_pData->m_filters[1].m_pa[l_n] = 0.0;
		l_pData->m_filters[1].m_pz[l_n] = 0.0;
	}
	
	for( l_n=0; l_n<l_pData->m_N_window; l_n++){
		l_pData->m_pw[l_n] = 0.54 - 0.46*cosf(2*M_PI*l_n/(l_pData->m_N_window-1));
	}
	
	return (LADSPA_Handle)l_pData;
}

static void LPVocoder_connect_port(
	LADSPA_Handle p_pInstance,
	unsigned long p_port,
	LADSPA_Data  *p_pdata )
{
	LPVocoder_Data *l_pData = (LPVocoder_Data*)p_pInstance;
	l_pData->m_pport[p_port] = p_pdata;
}

static void LPVocoder_correlate( LPVocoder_Data *l_pData, double *p_src )
{
	unsigned long l_i;
	for(l_i=0;l_i<=N_ORDER;l_i++){
		double *l_psrc1 = p_src;
		double *l_psrc2 = p_src + l_i;
		double  l_R = 0.0;
		unsigned long l_s;
		for(l_s=l_pData->m_N_window-l_i;l_s;l_s--){
			l_R += *l_psrc1 * *l_psrc2;
			l_psrc1++;
			l_psrc2++;
		}
		l_pData->m_pR[l_i] = l_R;
	}
}

static void LPVocoder_LevinsonDurbin( LPVocoder_Data *l_pData )
{
	double l_E = l_pData->m_pR[0];
	long l_i;
	for( l_i=0; l_i<N_ORDER; l_i++ ){
		double l_k_num = l_pData->m_pR[l_i+1];
		long l_j;
		for( l_j=0; l_j<=(l_i-1); l_j++ ){
			l_k_num -= l_pData->m_palpha[l_i-1][l_j] * l_pData->m_pR[l_i-l_j];
		}
		double l_k = l_k_num / l_E;
		l_pData->m_palpha[l_i][l_i] = l_k;
		for( l_j=0; l_j<=(l_i-1); l_j++ ){
			l_pData->m_palpha[l_i][l_j] = l_pData->m_palpha[l_i-1][l_j] - l_k*l_pData->m_palpha[l_i-1][l_i-l_j-1];
		}
		l_E *= (1 - l_k*l_k);
	}

	// program the gain of the filter
	l_pData->m_gain = sqrt( l_E );
}

static void LPVocoder_LevinsonDurbinFast( LPVocoder_Data *l_pData )
{
	double l_E = l_pData->m_pR[0];
	long l_i;
	for( l_i=0; l_i<N_ORDER; l_i++ ){
		double l_k_num = l_pData->m_pR[l_i+1];
		long l_j;
		if( (l_i-1) >= 0 ){
			double *l_palpha = l_pData->m_palpha[l_i-1];
			double *l_pR = &l_pData->m_pR[l_i];
			for( l_j=l_i-1; l_j>=0; l_j-- ){
			//for( l_j=0; l_j<=(l_i-1); l_j++ ){
				//l_k_num -= l_pData->m_palpha[l_i-1][l_j] * l_pData->m_pR[l_i-l_j];
				l_k_num -= *l_palpha * *l_pR;
				l_palpha++;
				l_pR--;
			}
		}
		double l_k = l_k_num / l_E;
		l_pData->m_palpha[l_i][l_i] = l_k;
		if( (l_i-1) >= 0 ){
			double *l_palphadst = l_pData->m_palpha[l_i];
			double *l_palphasrc = l_pData->m_palpha[l_i-1];
			double *l_palphasrck = &l_pData->m_palpha[l_i-1][l_i-1];
			//for( l_j=0; l_j<=(l_i-1); l_j++ ){
			for( l_j = l_i-1; l_j>=0; l_j--){
				//l_pData->m_palpha[l_i][l_j] = l_pData->m_palpha[l_i-1][l_j] - l_k*l_pData->m_palpha[l_i-1][l_i-l_j-1];
				*l_palphadst = *l_palphasrc - l_k * *l_palphasrck;
				l_palphadst++;
				l_palphasrc++;
				l_palphasrck--;
			}
		}
		l_E *= (1 - l_k*l_k);
	}

	// program the gain of the filter
	l_pData->m_gain = sqrt( l_E );
}

static double LPVocoder_filter_evaluate( LPVocoder_Data *p_pVocoder, Filter_Data *p_pFilter, double p_x )
{
	double l_fb = 0.0;
	long l_i_start = p_pFilter->m_i_z-1;
	if( l_i_start < 0 ) l_i_start+=N_ORDER;

	long l_N_loop1=l_i_start+1;
	long l_N_loop2=N_ORDER-l_N_loop1;

	double *l_pa = p_pFilter->m_pa;
	double *l_pz =&p_pFilter->m_pz[l_i_start];
	
	long l_i;
	for( l_i=l_N_loop1-1; ; l_i--){
		l_fb += *l_pa * *l_pz;
		l_pa++;
		if( l_i==0 ) break;
		l_pz--;
	}
	
	if( l_N_loop2 ){
		l_pz = &p_pFilter->m_pz[N_ORDER-1];
		for( l_i=l_N_loop2-1; ; l_i-- ){
			l_fb += *l_pa * *l_pz;
			if( l_i == 0 ) break;
			l_pa++;
			l_pz--;
		}
	}
	
	double l_y = p_x - l_fb;
	if( !isfinite(l_y) ) l_y=0.0;
	p_pFilter->m_pz[ p_pFilter->m_i_z ] = l_y;
	if(++p_pFilter->m_i_z==N_ORDER)
		p_pFilter->m_i_z = 0;

	l_y *= p_pFilter->m_envelope*p_pFilter->m_gain;
	
	p_pFilter->m_envelope += p_pFilter->m_denvelope;
	if( p_pFilter->m_i_window == p_pVocoder->m_N_window/2 )
		p_pFilter->m_denvelope*=-1.0;
	
	return l_y;
}

static void LPVocoder_DataIn( LPVocoder_Data *p_pVocoder, Filter_Data *p_pFilter, double p_x )
{
	p_x *= *p_pFilter->m_pw;
	p_pFilter->m_px[ p_pFilter->m_i_window ] = p_x;
	if( ++p_pFilter->m_i_window == p_pVocoder->m_N_window ){
		p_pFilter->m_i_window = 0;
		p_pFilter->m_pw = p_pVocoder->m_pw;
		p_pFilter->m_envelope = 0.0;
		p_pFilter->m_denvelope = 1.0/(p_pVocoder->m_N_window/2-1);
		LPVocoder_correlate( p_pVocoder, p_pFilter->m_px );
		double l_rms = p_pVocoder->m_pR[0]/p_pVocoder->m_N_window;
		l_rms = sqrtf( l_rms );
		if( l_rms > 1e-4 ){
			LPVocoder_LevinsonDurbinFast( p_pVocoder );
			p_pFilter->m_gain = p_pVocoder->m_gain;
			long l_i;
			double *l_palpha = p_pVocoder->m_palpha[N_ORDER-1];
			double *l_pa = p_pFilter->m_pa;
			for( l_i=N_ORDER-1;;l_i--){
				*l_pa = - *l_palpha;
				if(l_i==0){
					break;
				}else{
					l_pa++;
					l_palpha++;
				}
			}
		}else{
			p_pFilter->m_gain = 0;
		}
	}
}

static void LPVocoder_run(
	LADSPA_Handle p_pInstance,
	unsigned long p_sample_count )
{
	LPVocoder_Data *l_pVocoder = (LPVocoder_Data*)p_pInstance;
	
	LADSPA_Data *l_pSrcCTL = l_pVocoder->m_pport[PORT_IN_CTL];
	LADSPA_Data *l_pSrcRAW = l_pVocoder->m_pport[PORT_IN_RAW];
	LADSPA_Data *l_pDst0 = l_pVocoder->m_pport[PORT_OUT_0];
	LADSPA_Data *l_pDst1 = l_pVocoder->m_pport[PORT_OUT_1];
	
	long l_sample;
	for( l_sample = p_sample_count-1; ; l_sample-- ){
		double l_y = LPFilter_evaluate( &l_pVocoder->m_lpf, *l_pSrcCTL );

		LPVocoder_DataIn( l_pVocoder, &l_pVocoder->m_filters[0], l_y );
		LPVocoder_DataIn( l_pVocoder, &l_pVocoder->m_filters[1], l_y );

		l_y = LPVocoder_filter_evaluate( l_pVocoder, &l_pVocoder->m_filters[0], *l_pSrcRAW );
		l_y+= LPVocoder_filter_evaluate( l_pVocoder, &l_pVocoder->m_filters[1], *l_pSrcRAW );
		
		*l_pDst0 = *l_pDst1 = l_y;

		if( l_sample == 0 ) break;

		l_pSrcCTL++;
		l_pSrcRAW++;
		l_pDst0++;
		l_pDst1++;
	}
}

static void LPVocoder_cleanup( LADSPA_Handle p_pInstance )
{
	LPVocoder_Data *l_pData = (LPVocoder_Data*)p_pInstance;
	free( l_pData->m_pdata );

	free( p_pInstance );
}

static LADSPA_PortDescriptor LPVocoder_PortDescriptors[] =
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO
};

static const char* LPVocoder_PortNames[]=
{
	"Control Input",
	"Raw Input",
	"Output 0",
	"Output 1"
};

static LADSPA_PortRangeHint LPVocoder_PortRangeHints[]=
{
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0}
};

LADSPA_Descriptor LPVocoder_Descriptor=
{
	5812,
	"vocoder_lp",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Vocoder Linear Prediction",
	"Timothy William Krause",
	"None",
	PORT_N_PORTS,
	LPVocoder_PortDescriptors,
	LPVocoder_PortNames,
	LPVocoder_PortRangeHints,
	NULL,
	LPVocoder_instantiate,
	LPVocoder_connect_port,
	NULL,
	LPVocoder_run,
	NULL,
	NULL,
	NULL,
	LPVocoder_cleanup
};
