#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

/*
 *                              s^2
 *  Hbp(s) = -------------------------------------------------
 *            Q^2*s^4 + Q*c*s^3 + (2*Q^2+1)*s^2 + Q*c*s + Q^2
 *
 * Transfer function for a single conjugate pole pair filter
 * after band-pass transformation
 *
 *		       1/a0*(b0 + b1*z^-1 + b2*z^-2 + b3*z^-3 + b4*z^-4)
 *   Hbq(z) = ---------------------------------------------------
 *            1 + (a1/a0)*z^-1 + (a2/a0)*z^-2 + (a3/a0)*z^-3
 *              + (a4/a0)*z^-4
 *
 * b0 = K^2
 * b1 = 0
 * b2 = -2*K^2
 * b3 = 0
 * b4 = K^2
 * a0 = (K^3+K)*Q*c + (K^4+2*K^2+1)*Q^2 + K^2
 * a1 = 2*(K-K^3)*Q*c + 4*(1-K^4)*Q^2
 * a2 = (6*K^4-4*K^2+6)*Q^2 - 2*K^2
 * a3 = 2*(K^3-K)*Q*c + 4*(1-K^4)*Q^2
 * a4 = -(K^3+K)*Q*c + (K^4+2*K^2+1)*Q^2 + K^2
 *
 * omega = 2 * pi * fc / fs
 * K = 1/tan(omega/2)
 *
 *
 * Transfer function for a single real pole after band-pass
 * transformation
 *
 *              1/a0*(b0 + b1*z^-1 + b2*z^-2)
 *  Hsp(z) = ----------------------------------
 *             1 + (a1/a0)*z^-1 + (a2/a0)*z^2
 *
 * b0 = K
 * b1 = 0
 * b2 = -K
 * a0 = (K^2+1)*Q + K
 * a1 = (2-2*K^2)*Q
 * a2 = (K^2+1)*Q - K
 *
 */

enum {
    PORT_IN,
    PORT_OUT,
    PORT_N,
    PORT_FREQUENCY,
    PORT_Q,
    PORT_NPORTS
};

typedef struct {
    double m_z[3];
    double m_a1;
    double m_a2;
    double m_g;
} SP_Filter;

static void SP_Filter_init(SP_Filter *sp)
{
    sp->m_z[0] = 0.0;
    sp->m_z[1] = 0.0;
    sp->m_z[2] = 0.0;
}

static void SP_Filter_set(SP_Filter *sp, LADSPA_Data K, LADSPA_Data Q)
{
    double K2 = K*K;
    double l_a0 = (K2+1.0)*Q;
    double l_a2 = l_a0;
    l_a0 += K;
    l_a2 -= K;
    double l_a1 = (2.0-2.0*K2)*Q;
    sp->m_a1 = l_a1/l_a0;
    sp->m_a2 = l_a2/l_a0;
    sp->m_g = K/l_a0;
}

static double SP_Filter_eval(SP_Filter *sp, double x)
{
    sp->m_z[0] = x - sp->m_a1*sp->m_z[1] - sp->m_a2*sp->m_z[2];
    double l_y = sp->m_z[0] - sp->m_z[2];
    l_y *= sp->m_g;
    sp->m_z[2] = sp->m_z[1];
    sp->m_z[1] = sp->m_z[0];
    return l_y;
}

typedef struct {
    double m_z[5];
    double m_a1;
    double m_a2;
    double m_a3;
    double m_a4;
    double m_g;
} BQ_Filter;

static void BQ_Filter_init(BQ_Filter *bq)
{
    bq->m_z[1] = 0.0;
    bq->m_z[2] = 0.0;
    bq->m_z[3] = 0.0;
    bq->m_z[4] = 0.0;
}

static void BQ_Filter_set(BQ_Filter *bq,
                   LADSPA_Data K,
                   LADSPA_Data Q,
                   LADSPA_Data c)
{
    double K2 = K*K;
    double K3 = K2*K;
    double K4 = K2*K2;
    double Q2 = Q*Q;
    double l_a0 = (K3+K)*Q*c + (K4+2.0*K2+1.0)*Q2 + K2;
    double l_a1 = 2.0*(K-K3)*Q*c + 4.0*(1.0-K4)*Q2;
    double l_a2 = (6.0*K4-4.0*K2+6)*Q2 - 2.0*K2;
    double l_a3 = 2.0*(K3-K)*Q*c + 4.0*(1.0-K4)*Q2;
    double l_a4 = -(K3+K)*Q*c + (K4+2.0*K2+1.0)*Q2 + K2;
    bq->m_a1 = l_a1/l_a0;
    bq->m_a2 = l_a2/l_a0;
    bq->m_a3 = l_a3/l_a0;
    bq->m_a4 = l_a4/l_a0;
    bq->m_g  = K*K/l_a0;
}

static double BQ_Filter_eval(BQ_Filter *bq, double x)
{
    bq->m_z[0] = x - bq->m_a1*bq->m_z[1] - bq->m_a2*bq->m_z[2]
            -bq->m_a3*bq->m_z[3] - bq->m_a4*bq->m_z[4];
    double l_y = bq->m_z[0] - 2.0f*bq->m_z[2] + bq->m_z[4];
    l_y *= bq->m_g;
    bq->m_z[4]=bq->m_z[3];
    bq->m_z[3]=bq->m_z[2];
    bq->m_z[2]=bq->m_z[1];
    bq->m_z[1]=bq->m_z[0];
    return l_y;
}

#define N_BQ 5
#define N_ORDER_MAX 11

typedef struct {
    LADSPA_Data  m_sample_rate;
    LADSPA_Data *m_pport[PORT_NPORTS];
    SP_Filter    m_sp;
    BQ_Filter    m_bq[N_BQ];
    int          m_N_bq;
    int          m_sp_on;
} BW_BP_Data;

void BW_BP_set(BW_BP_Data *p_pBW_BP, int p_N, LADSPA_Data p_K,
               LADSPA_Data p_Q)
{
    if((p_N&1) == 0){
        // N even
        int N_bq = p_N/2;
        for(int m=1,bq=0;bq<N_bq;m+=2,bq++){
            LADSPA_Data c = 2.0f*cosf((float)m*M_PIf/2.0f/p_N);
            BQ_Filter_set(&p_pBW_BP->m_bq[bq], p_K, p_Q, c);
        }
        p_pBW_BP->m_N_bq = N_bq;
        p_pBW_BP->m_sp_on = 0;
    } else {
        // N odd
        SP_Filter_set(&p_pBW_BP->m_sp, p_K, p_Q);
        int K = (p_N-1)/2;
        p_pBW_BP->m_sp_on = 1;
        p_pBW_BP->m_N_bq = K;
        for(int k=1;k<=K;k++){
            LADSPA_Data c = 2.0f*cosf(k*M_PIf/p_N);
            BQ_Filter_set(&p_pBW_BP->m_bq[k-1], p_K, p_Q, c);
        }
    }
}

LADSPA_Data BW_BP_eval( BW_BP_Data *p_pBW_BP, LADSPA_Data x)
{
    double a = x;
    if(p_pBW_BP->m_sp_on)
        a = SP_Filter_eval(&p_pBW_BP->m_sp, a);
    for(int i=0;i<p_pBW_BP->m_N_bq;i++){
        a = BQ_Filter_eval(&p_pBW_BP->m_bq[i], a);
    }
    return (LADSPA_Data)a;
}

static LADSPA_Handle BW_BP_instantiate(
    const struct _LADSPA_Descriptor *p_pDescriptor,
    unsigned long p_sample_rate)
{
    BW_BP_Data *l_pBW_BP = malloc( sizeof(BW_BP_Data) );
    if(l_pBW_BP){
        l_pBW_BP->m_sample_rate = (float)p_sample_rate;
        SP_Filter_init(&l_pBW_BP->m_sp);
        for(int i=0;i<N_BQ;i++){
            BQ_Filter_init(&l_pBW_BP->m_bq[i]);
        }
    }
    return (LADSPA_Handle)l_pBW_BP;
}

static void BW_BP_connect_port(
        LADSPA_Handle p_pInstance,
        unsigned long p_port,
        LADSPA_Data *p_pdata)
{
    BW_BP_Data *l_pBW_BP = (BW_BP_Data*)p_pInstance;
    l_pBW_BP->m_pport[p_port] = p_pdata;
}

static void BW_BP_run(
        LADSPA_Handle p_pInstance,
        unsigned long p_sample_count)
{
    BW_BP_Data *l_pBW_BP = (BW_BP_Data*)p_pInstance;
    LADSPA_Data *l_psrc = l_pBW_BP->m_pport[PORT_IN];
    LADSPA_Data *l_pdst = l_pBW_BP->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;

    int N = (int)*l_pBW_BP->m_pport[PORT_N];
    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pBW_BP->m_pport[PORT_FREQUENCY];
    l_omega /= l_pBW_BP->m_sample_rate;
    LADSPA_Data l_K = 1.0f/tanf(l_omega/2.0f);
    LADSPA_Data l_Q = *l_pBW_BP->m_pport[PORT_Q];
    BW_BP_set(l_pBW_BP, N, l_K, l_Q);

    for(;l_psrc!=l_psrc_end;l_psrc++,l_pdst++){
        *l_pdst = BW_BP_eval(l_pBW_BP, *l_psrc);
    }
}

static void BW_BP_cleanup( LADSPA_Handle p_pInstance )
{
    free( p_pInstance );
}

static LADSPA_PortDescriptor BW_BP_PortDescriptors[]=
{
    LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char *BW_BP_PortNames[]=
{
    "Input",
    "Output",
    "N(Filter order)",
    "Frequency(Hertz)",
    "Q(wc/(w1-w0))"
};

static LADSPA_PortRangeHint BW_BP_PortRangeHints[]=
{
    {0,0.0f,0.0f},
    {0,0.0f,0.0f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_INTEGER | LADSPA_HINT_DEFAULT_MINIMUM,
     1.0f, 11.0f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
     10.0f,20.0e3f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
     0.1f,10.0f}
};

LADSPA_Descriptor BW_BP_Descriptor=
{
    5828,
    "BW_BP",
    LADSPA_PROPERTY_HARD_RT_CAPABLE,
    "Butterworth Band Pass",
    "Timothy William Krause",
    "None",
    PORT_NPORTS,
    BW_BP_PortDescriptors,
    BW_BP_PortNames,
    BW_BP_PortRangeHints,
    NULL,
    BW_BP_instantiate,
    BW_BP_connect_port,
    NULL,
    BW_BP_run,
    NULL,
    NULL,
    NULL,
    BW_BP_cleanup
};
