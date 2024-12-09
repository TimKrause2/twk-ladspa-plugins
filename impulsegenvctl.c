/*
 * Voice pitch determination steps
 *
 * 1) auto-correlation up to Norder+1 taps
 * 2) Levinson-Durbin to calculate the recursion coefficients
 * 3) filter the original signal by the inverse of the LPC model
 * 4) filter 3) by a low pass filter
 * 5) auto-correlation of 4) from max frequency interval to min
 *    frequency interval
 * 6) find the peek in the correlates. This is the interval in samples
 *    of the voice fundamental
 */

#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

#define N_ORDER  48

#define N_WINDOW 32
#define N_SS     1024

#define F_MIN    80.0f
#define F_MAX    1050.0f

#define Q_LOPASS 0.707f
#define ALPHA_DC 0.95f

enum {
	PORT_IN,
	PORT_OUT,
    PORT_IMPULSE_AMP,
    PORT_NOISE_AMP,
    PORT_NRAMP,
    PORT_PITCH,
	PORT_NPORTS
};

static LADSPA_PortDescriptor ImpulseGenVC_PortDescriptors[]=
{
	LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char* ImpulseGenVC_PortNames[]=
{
	"Input",
	"Output",
    "Impulse Amp(dB)",
    "Noise Amp(dB)",
    "Pulse Width",
    "Pitch Shift"
};

static LADSPA_PortRangeHint ImpulseGenVC_PortRangeHints[]=
{
	{0, 0.0f, 0.0f},
	{0, 0.0f, 0.0f},
    { LADSPA_HINT_BOUNDED_BELOW |
         LADSPA_HINT_BOUNDED_ABOVE |
         LADSPA_HINT_DEFAULT_MAXIMUM,
     -60.0f, 0.0f},
    { LADSPA_HINT_BOUNDED_BELOW |
         LADSPA_HINT_BOUNDED_ABOVE |
         LADSPA_HINT_DEFAULT_MAXIMUM,
     -60.0f, 0.0f},
    { LADSPA_HINT_BOUNDED_BELOW |
         LADSPA_HINT_INTEGER |
         LADSPA_HINT_BOUNDED_ABOVE |
         LADSPA_HINT_DEFAULT_MINIMUM,
     1.0f, 10.0f},
    { LADSPA_HINT_BOUNDED_BELOW |
         LADSPA_HINT_BOUNDED_ABOVE |
         LADSPA_HINT_DEFAULT_MAXIMUM,
     -12.0f, 12.0f},
};

static LADSPA_Data sinc(LADSPA_Data x)
{
	if(fabs(x)<1e-9f){
		return 1.0f;
	}else{
		return sinf(x)/x;
	}
}

static LADSPA_Data hamming(int n, int N)
{
    LADSPA_Data a0 = 25.0f/46.0f;
    LADSPA_Data a1 = 1.0f - a0;
    return a0 - a1*cosf(2.0f*M_PIf*n/(N-1));
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

typedef struct {
    double m_z[N_ORDER];
    double m_a[N_ORDER];
    int m_i_z;
} LPC_Filter;

typedef struct {
    double *m_palpha[N_ORDER];
    double  m_alpha_data[N_ORDER*(N_ORDER+1)/2];
    double  m_R[N_ORDER+1];
    double  m_gain;
} LD_Data; // Levinson-Durbin data

typedef struct
{
    LADSPA_Data *m_x;
    int m_N;
    int m_i_x;
} InputBuffer;

static void LPC_Filter_init(LPC_Filter *f)
{
    for(int i=0;i<N_ORDER;i++)
    {
        f->m_z[i] = 0.0f;
    }
    f->m_i_z = 0;
}

static LADSPA_Data LPC_Filter_evaluate(LPC_Filter *f, LADSPA_Data x)
{
    double l_fb = 0.0;
    long l_i_start = f->m_i_z-1;
    if( l_i_start < 0 ) l_i_start+=N_ORDER;

    long l_N_loop1=l_i_start+1;
    long l_N_loop2=N_ORDER-l_N_loop1;

    double *l_pa = f->m_a;
    double *l_pz =&f->m_z[l_i_start];

    long l_i;
    for( l_i=l_N_loop1-1; ; l_i--){
        l_fb += *l_pa * *l_pz;
        l_pa++;
        if( l_i==0 ) break;
        l_pz--;
    }

    if( l_N_loop2 ){
        l_pz = &f->m_z[N_ORDER-1];
        for( l_i=l_N_loop2-1; ; l_i-- ){
            l_fb += *l_pa * *l_pz;
            if( l_i == 0 ) break;
            l_pa++;
            l_pz--;
        }
    }

    double l_y = (double)x + l_fb;
    if( !isfinite(l_y) ) l_y=0.0;
    f->m_z[ f->m_i_z ] = (double)x;
    if(++f->m_i_z==N_ORDER)
        f->m_i_z = 0;

    return (LADSPA_Data)l_y;
}


void LD_Data_init(LD_Data *ld)
{
    double *data = ld->m_alpha_data;
    for(int i=0;i<N_ORDER;i++){
        ld->m_palpha[i] = data;
        data += i+1;
    }
}

void LD_Data_correlate(LD_Data *ld, InputBuffer *ib)
{
    for(int lag=0;lag<(N_ORDER+1);lag++)
    {
        double R=0;
        LADSPA_Data *x0=ib->m_x;
        LADSPA_Data *x1=&ib->m_x[lag];
        for(int i=lag;i<ib->m_N;i++){
            R += (double)*x0 * (double)*x1;
            x0++;
            x1++;
        }
        ld->m_R[lag] = R;
    }
}

void LD_Data_evaluate(LD_Data *l_pData)
{
    double l_E = l_pData->m_R[0];
    long l_i;
    for( l_i=0; l_i<N_ORDER; l_i++ ){
        double l_k_num = l_pData->m_R[l_i+1];
        long l_j;
        if( (l_i-1) >= 0 ){
            double *l_palpha = l_pData->m_palpha[l_i-1];
            double *l_pR = &l_pData->m_R[l_i];
            for( l_j=l_i-1; l_j>=0; l_j-- ){
                //for( l_j=0; l_j<=(l_i-1); l_j++ ){
                //l_k_num -= l_pData->m_palpha[l_i-1][l_j] * l_pData->m_R[l_i-l_j];
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

void LD_Data_set_filter(LD_Data *ld, LPC_Filter *lpc)
{
    for(int i=0;i<N_ORDER;i++)
    {
        lpc->m_a[i] = -ld->m_palpha[N_ORDER-1][i];
    }
}

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
    LPC_Filter m_lpc;
    LD_Data m_ld;
    int m_N_window;
    int m_N_cor;
    int m_i_hi;
    int m_noise;
    int m_SS_rate;
    LADSPA_Data *m_data; // single malloc for all data
    LADSPA_Data *m_cor; // correlation data
    LADSPA_Data *m_SS_buffer; // super sampled data
    InputBuffer m_buffers[2];
} ImpulseGen;

static LADSPA_Data DCRemove_evaluate(DCRemove *p_pdc, LADSPA_Data p_x)
{
    LADSPA_Data l_m = p_x + ALPHA_DC*p_pdc->m_z1;
	LADSPA_Data l_r = l_m - p_pdc->m_z1;
	p_pdc->m_z1 = l_m;
	return l_r;
}

static void LoPass_init(LoPass *lp)
{
    lp->m_z1 = 0.0f;
    lp->m_z2 = 0.0f;
}

static void LoPass_set(LoPass *lp, LADSPA_Data sample_rate,
                       LADSPA_Data frequency, LADSPA_Data Q)
{
    LADSPA_Data l_omega = 2.0f*M_PIf*frequency/sample_rate;
    LADSPA_Data l_sin_omega = sinf(l_omega);
    LADSPA_Data l_cos_omega = cosf(l_omega);
    LADSPA_Data l_alpha = l_sin_omega / ( 2.0f * Q);
    LADSPA_Data l_a0 = 1.0f + l_alpha;
    lp->m_a1 = -2.0f * l_cos_omega / l_a0;
    lp->m_a2 = (1.0f - l_alpha) / l_a0;
    lp->m_b0 = (1.0f - l_cos_omega) / 2.0f / l_a0;
    lp->m_b1 = (1.0f - l_cos_omega) / l_a0;
    lp->m_b2 = (1.0f - l_cos_omega) / 2.0f / l_a0;
}

static LADSPA_Data LoPass_evaluate(LoPass *p_plp, LADSPA_Data p_x)
{
	LADSPA_Data l_m = p_x - p_plp->m_a1*p_plp->m_z1 - p_plp->m_a2*p_plp->m_z2;
	LADSPA_Data l_r = p_plp->m_b0*l_m + p_plp->m_b1*p_plp->m_z1 + p_plp->m_b2*p_plp->m_z2;
	p_plp->m_z2 = p_plp->m_z1;
	p_plp->m_z1 = l_m;
	return l_r;
}

void ImpulseGen_SS(ImpulseGen *p_pImpulseGen, InputBuffer *buff)
{
    LADSPA_Data *l_src = buff->m_x;
    LADSPA_Data *l_dst = p_pImpulseGen->m_SS_buffer;
    if(p_pImpulseGen->m_SS_rate==1){
        for(int s=0;s<buff->m_N;s++){
            *(l_dst++) = *(l_src++);
        }
    }else{
        LADSPA_Data l_cbuff[N_WINDOW];
        int l_i_cbuff = 0;
        int l_i_src = 0;
        for(;l_i_cbuff<N_WINDOW/2-1;l_i_cbuff++){
            l_cbuff[l_i_cbuff] = 0.0f;
        }
        for(;l_i_cbuff<N_WINDOW;l_i_cbuff++,l_i_src++){
            l_cbuff[l_i_cbuff] = buff->m_x[l_i_src];
        }
        l_i_cbuff = 0;
        for(int s=0;s<buff->m_N;s++){
            *(l_dst++) = *(l_src++);
            for(int ss=1;ss<p_pImpulseGen->m_SS_rate;ss++){
                int N_loop1 = N_WINDOW - l_i_cbuff;
                int N_loop2 = N_WINDOW - N_loop1;
                LADSPA_Data *l_impulse = p_pImpulseGen->m_impulse_data[N_SS*ss/p_pImpulseGen->m_SS_rate];
                LADSPA_Data *l_pcbuff = &l_cbuff[l_i_cbuff];
                *l_dst = 0.0f;
                for(int i=0;i<N_loop1;i++){
                    *l_dst += *(l_impulse++) * *(l_pcbuff++);
                }
                if(N_loop2){
                    l_pcbuff = l_cbuff;
                    for(int i=0;i<N_loop2;i++){
                        *l_dst += *(l_impulse++) * *(l_pcbuff++);
                    }
                }
                l_dst++;
            }
            l_cbuff[l_i_cbuff] = (l_i_src<buff->m_N)?buff->m_x[l_i_src]:0.0f;
            l_i_cbuff++;
            if(l_i_cbuff==N_WINDOW)
                l_i_cbuff=0;
            l_i_src++;
        }
    }
}

static LADSPA_Handle ImpulseGenVC_instantiate(
	const struct _LADSPA_Descriptor *p_pDescriptor,
	unsigned long p_sample_rate )
{
    ImpulseGen *p_pImpulseGen = (ImpulseGen*)malloc( sizeof(ImpulseGen) );
	if(!p_pImpulseGen)
		return NULL;
	p_pImpulseGen->m_sample_rate = p_sample_rate;
	p_pImpulseGen->m_i_acc = 0;
	p_pImpulseGen->m_Tacc = 0.0f;
	p_pImpulseGen->m_Tperiod = (LADSPA_Data)p_sample_rate/100.0f;
    p_pImpulseGen->m_N_window = (int)ceilf((LADSPA_Data)p_sample_rate/F_MIN);
    p_pImpulseGen->m_i_hi = (int)floorf((LADSPA_Data)p_sample_rate/F_MAX);
    if(p_sample_rate <= 50000){
        p_pImpulseGen->m_SS_rate = 4;
    }else if(p_sample_rate <= 100000){
        p_pImpulseGen->m_SS_rate = 2;
    }else{
        p_pImpulseGen->m_SS_rate = 1;
    }
    p_pImpulseGen->m_N_cor = p_pImpulseGen->m_N_window - p_pImpulseGen->m_i_hi + 1;
    p_pImpulseGen->m_noise = 0;
    for(int i_ss=0;i_ss<N_SS;i_ss++){
		LADSPA_Data l_alpha = (LADSPA_Data)i_ss/N_SS;
		for(int i_window=0;i_window<N_WINDOW;i_window++){
            LADSPA_Data l_x = (LADSPA_Data)(i_window-(N_WINDOW/2-1))-l_alpha;
            p_pImpulseGen->m_impulse_data[i_ss][i_window] = sinc(M_PI*l_x)*hamming(i_window,N_WINDOW);
		}
	}
	for(int i_window=0;i_window<N_WINDOW;i_window++){
		p_pImpulseGen->m_accumulator[i_window] = 0.0f;
	}

	p_pImpulseGen->m_dc.m_z1 = 0.0f;

    LoPass_set(&p_pImpulseGen->m_lp, p_sample_rate, F_MAX, Q_LOPASS);

    int N_bytes = p_pImpulseGen->m_N_window*4;
    N_bytes += p_pImpulseGen->m_N_cor*p_pImpulseGen->m_SS_rate;
    N_bytes += p_pImpulseGen->m_N_window*2*p_pImpulseGen->m_SS_rate;
    N_bytes *= sizeof(LADSPA_Data);
    p_pImpulseGen->m_data = (LADSPA_Data*)malloc(N_bytes);
    if(!p_pImpulseGen->m_data){
        free(p_pImpulseGen);
        return NULL;
    }

    LADSPA_Data *l_data = p_pImpulseGen->m_data;
    p_pImpulseGen->m_cor = l_data;
    l_data += p_pImpulseGen->m_N_cor*p_pImpulseGen->m_SS_rate;

    p_pImpulseGen->m_SS_buffer = l_data;
    l_data += p_pImpulseGen->m_N_window*2*p_pImpulseGen->m_SS_rate;

    p_pImpulseGen->m_buffers[0].m_x = l_data;
    p_pImpulseGen->m_buffers[0].m_N = p_pImpulseGen->m_N_window*2;
    p_pImpulseGen->m_buffers[0].m_i_x = 0;
    l_data += p_pImpulseGen->m_N_window*2;
    p_pImpulseGen->m_buffers[1].m_x = l_data;
    p_pImpulseGen->m_buffers[1].m_N = p_pImpulseGen->m_N_window*2;
    p_pImpulseGen->m_buffers[1].m_i_x = p_pImpulseGen->m_N_window;

    for(int s=0;s<p_pImpulseGen->m_N_window*2;s++){
        p_pImpulseGen->m_buffers[0].m_x[s] = 0.0f;
        p_pImpulseGen->m_buffers[1].m_x[s] = 0.0f;
    }

    LD_Data_init(&p_pImpulseGen->m_ld);

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

static LADSPA_Data ImpulseGenVC_evaluate(ImpulseGen *p_pImpulseGen, int N_ramp){
	LADSPA_Data l_deltaT = p_pImpulseGen->m_Tacc - p_pImpulseGen->m_Tperiod;
    LADSPA_Data l_result;
    if(N_ramp==1){
        if(l_deltaT<=-1.0f){
            p_pImpulseGen->m_Tacc += 1.0f;
        }else if(l_deltaT<=0.0f){
            ImpulseGenVC_impulse(p_pImpulseGen,-l_deltaT);
            p_pImpulseGen->m_Tacc = 1.0f + l_deltaT;
        }else{
            ImpulseGenVC_impulse(p_pImpulseGen, 0.0f);
            p_pImpulseGen->m_Tacc = 0.0f;
        }
        l_result = p_pImpulseGen->m_accumulator[p_pImpulseGen->m_i_acc];
        p_pImpulseGen->m_accumulator[p_pImpulseGen->m_i_acc] = 0.0f;
        p_pImpulseGen->m_i_acc++;
        p_pImpulseGen->m_i_acc%=N_WINDOW;
    }else{
        if(p_pImpulseGen->m_Tacc>N_ramp*2){
            l_result = 0.0f;
        }else if(p_pImpulseGen->m_Tacc>N_ramp){
            l_result = 2.0f - p_pImpulseGen->m_Tacc/N_ramp;
        }else{
            l_result = p_pImpulseGen->m_Tacc/N_ramp;
        }
        if(l_deltaT<=-1.0f){
            p_pImpulseGen->m_Tacc += 1.0f;
        }else if(l_deltaT<=0.0f){
            p_pImpulseGen->m_Tacc = 1.0f + l_deltaT;
        }else{
            p_pImpulseGen->m_Tacc = 0.0f;
        }
    }
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

    LADSPA_Data l_impulse_amp = exp10f(*p_pImpulseGen->m_pport[PORT_IMPULSE_AMP]/20.0f);
    LADSPA_Data l_noise_amp = exp10f(*p_pImpulseGen->m_pport[PORT_NOISE_AMP]/20.0f);
    int l_N_ramp = (int)*p_pImpulseGen->m_pport[PORT_NRAMP];
    LADSPA_Data l_pitch = *p_pImpulseGen->m_pport[PORT_PITCH];

	long sample;
	LADSPA_Data *l_psrc = p_pImpulseGen->m_pport[PORT_IN];
	LADSPA_Data *l_pdst = p_pImpulseGen->m_pport[PORT_OUT];
	
	for(sample=p_sample_count;sample!=0;sample--){
		LADSPA_Data l_x = DCRemove_evaluate(&p_pImpulseGen->m_dc, *(l_psrc++));
        for(int b=0;b<2;b++){
            InputBuffer *buff = &p_pImpulseGen->m_buffers[b];
            buff->m_x[buff->m_i_x++] = l_x;
            if(buff->m_i_x == buff->m_N){
                buff->m_i_x = 0;
                // buffer ready
                LD_Data_correlate(&p_pImpulseGen->m_ld, buff);
                if(p_pImpulseGen->m_ld.m_R[0] < 1e-9)
                    continue;
                LD_Data_evaluate(&p_pImpulseGen->m_ld);
                LD_Data_set_filter(&p_pImpulseGen->m_ld, &p_pImpulseGen->m_lpc);
                LPC_Filter_init(&p_pImpulseGen->m_lpc);
                LoPass_init(&p_pImpulseGen->m_lp);
                for(int s=0;s<buff->m_N;s++){
                    LADSPA_Data temp = LPC_Filter_evaluate(&p_pImpulseGen->m_lpc, buff->m_x[s]);
                    buff->m_x[s] = LoPass_evaluate(&p_pImpulseGen->m_lp, temp);
                }
                ImpulseGen_SS(p_pImpulseGen, buff);
                LADSPA_Data *l_cor = p_pImpulseGen->m_cor;
                int lag=p_pImpulseGen->m_i_hi*p_pImpulseGen->m_SS_rate;
                int lag_last=p_pImpulseGen->m_N_window*p_pImpulseGen->m_SS_rate + (p_pImpulseGen->m_SS_rate - 1);
                int N_s = p_pImpulseGen->m_N_window*2*p_pImpulseGen->m_SS_rate;
                for(;lag<=lag_last;lag++){
                    *l_cor = 0.0f;
                    LADSPA_Data *x0 = p_pImpulseGen->m_SS_buffer;
                    LADSPA_Data *x1 = &p_pImpulseGen->m_SS_buffer[lag];
                    for(int s=lag;s<N_s;s++){
                        *l_cor += *x0 * *x1;
                        x0++;
                        x1++;
                    }
                    l_cor++;
                }
                LADSPA_Data l_cor0=0.0f;
                LADSPA_Data *x = p_pImpulseGen->m_SS_buffer;
                for(int s=0;s<N_s;s++){
                    l_cor0 += *x * *x;
                    x++;
                }
                // find the average
                LADSPA_Data l_avg=0.0;
                l_cor = p_pImpulseGen->m_cor;
                for(int i=0;i<p_pImpulseGen->m_N_cor*p_pImpulseGen->m_SS_rate;i++){
                    l_avg += *l_cor;
                }
                l_avg /= p_pImpulseGen->m_N_cor*p_pImpulseGen->m_SS_rate;

                // find the peek
                l_cor = p_pImpulseGen->m_cor;
                LADSPA_Data peek = *l_cor;
                int i_peek = 0;
                l_cor++;
                for(int i=1;i<p_pImpulseGen->m_N_cor*p_pImpulseGen->m_SS_rate;i++){
                    if(*l_cor > peek){
                        peek = *l_cor;
                        i_peek = i;
                    }
                    l_cor++;
                }
                if((peek - l_avg)/l_cor0 > 0.3f){
                    p_pImpulseGen->m_noise = 0;
                    p_pImpulseGen->m_Tperiod = (LADSPA_Data)(p_pImpulseGen->m_i_hi+(LADSPA_Data)i_peek/p_pImpulseGen->m_SS_rate)*powf(2.0f,-l_pitch/12.0f);
                }else{
                    p_pImpulseGen->m_noise = 1;
                }
            }
        }
        if(!p_pImpulseGen->m_noise)
            *(l_pdst++) = ImpulseGenVC_evaluate(p_pImpulseGen, l_N_ramp)*l_impulse_amp;
        else
            *(l_pdst++) = ((float)drand48()*2.0f-1.0f)*l_noise_amp;
	}
}

static void ImpulseGenVC_cleanup(
	LADSPA_Handle p_pinstance )
{
	ImpulseGen *p_pImpulseGen = (ImpulseGen*)p_pinstance;
    free(p_pImpulseGen->m_data);
    free( p_pImpulseGen );
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
