/*

twk.so is a set of LADSPA plugins.

Copyright 2024 Tim Krause

This file is part of twk.so.

twk.so is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published
by the Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

twk.so is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with twk.so. If not, see
<https://www.gnu.org/licenses/>.

Contact: tim.krause@twkrause.ca

*/
#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

/*
 *                    Q^2*s^4 + 2*Q^2*s^2 + Q^2
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
 * b0 = (K^4+2K^2+1)*Q^2
 * b1 = (4-4*K^4)*Q^2
 * b2 = (6*K^4-4K^2+6)*Q^2
 * b3 = b1
 * b4 = b0
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
 * Transfer function for a single real pole after band-stop
 * transformation
 *
 *              1/a0*(b0 + b1*z^-1 + b2*z^-2)
 *  Hsp(z) = ----------------------------------
 *             1 + (a1/a0)*z^-1 + (a2/a0)*z^2
 *
 * b0 = (K^2+1)*Q
 * b1 = (2-2*K^2)*Q
 * b2 = (K^2+1)*Q
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
    double m_b0;
    double m_b1;
    double m_b2;
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
    double l_b0 = (K2+1.0)*Q;
    double l_b1 = (2.0-2.0*K2)*Q;
    double l_b2 = l_b0;
    sp->m_a1 = l_a1/l_a0;
    sp->m_a2 = l_a2/l_a0;
    sp->m_b0 = l_b0/l_a0;
    sp->m_b1 = l_b1/l_a0;
    sp->m_b2 = l_b2/l_a0;
}

static double SP_Filter_eval(SP_Filter *sp, double x)
{
    sp->m_z[0] = x - sp->m_a1*sp->m_z[1] - sp->m_a2*sp->m_z[2];
    double l_y = sp->m_b0*sp->m_z[0] + sp->m_b1*sp->m_z[1]
            + sp->m_b2*sp->m_z[2];
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
    double m_b0;
    double m_b1;
    double m_b2;
    double m_b3;
    double m_b4;
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
    double l_b0 = (K4+2.0*K2+1.0)*Q2;
    double l_b1 = (4.0-4.0*K4)*Q2;
    double l_b2 = (6.0*K4-4.0*K2+6.0)*Q2;
    double l_b3 = l_b1;
    double l_b4 = l_b0;
    bq->m_a1 = l_a1/l_a0;
    bq->m_a2 = l_a2/l_a0;
    bq->m_a3 = l_a3/l_a0;
    bq->m_a4 = l_a4/l_a0;
    bq->m_b0 = l_b0/l_a0;
    bq->m_b1 = l_b1/l_a0;
    bq->m_b2 = l_b2/l_a0;
    bq->m_b3 = l_b3/l_a0;
    bq->m_b4 = l_b4/l_a0;
}

static double BQ_Filter_eval(BQ_Filter *bq, double x)
{
    bq->m_z[0] = x - bq->m_a1*bq->m_z[1] - bq->m_a2*bq->m_z[2]
            -bq->m_a3*bq->m_z[3] - bq->m_a4*bq->m_z[4];
    double l_y = bq->m_b0*bq->m_z[0] + bq->m_b1*bq->m_z[1]
            + bq->m_b2*bq->m_z[2] + bq->m_b3*bq->m_z[3]
            + bq->m_b4*bq->m_z[4];
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
} BW_BS_Data;

void BW_BS_set(BW_BS_Data *p_pBW_BS, int p_N, LADSPA_Data p_K,
               LADSPA_Data p_Q)
{
    if((p_N&1) == 0){
        // N even
        int N_bq = p_N/2;
        for(int m=1,bq=0;bq<N_bq;m+=2,bq++){
            LADSPA_Data c = 2.0f*cosf((float)m*M_PIf/2.0f/p_N);
            BQ_Filter_set(&p_pBW_BS->m_bq[bq], p_K, p_Q, c);
        }
        p_pBW_BS->m_N_bq = N_bq;
        p_pBW_BS->m_sp_on = 0;
    } else {
        // N odd
        SP_Filter_set(&p_pBW_BS->m_sp, p_K, p_Q);
        int K = (p_N-1)/2;
        p_pBW_BS->m_sp_on = 1;
        p_pBW_BS->m_N_bq = K;
        for(int k=1;k<=K;k++){
            LADSPA_Data c = 2.0f*cosf(k*M_PIf/p_N);
            BQ_Filter_set(&p_pBW_BS->m_bq[k-1], p_K, p_Q, c);
        }
    }
}

LADSPA_Data BW_BS_eval( BW_BS_Data *p_pBW_BS, LADSPA_Data x)
{
    double a = x;
    if(p_pBW_BS->m_sp_on)
        a = SP_Filter_eval(&p_pBW_BS->m_sp, a);
    for(int i=0;i<p_pBW_BS->m_N_bq;i++){
        a = BQ_Filter_eval(&p_pBW_BS->m_bq[i], a);
    }
    return (LADSPA_Data)a;
}

static LADSPA_Handle BW_BS_instantiate(
    const struct _LADSPA_Descriptor *p_pDescriptor,
    unsigned long p_sample_rate)
{
    BW_BS_Data *l_pBW_BS = malloc( sizeof(BW_BS_Data) );
    if(l_pBW_BS){
        l_pBW_BS->m_sample_rate = (float)p_sample_rate;
        SP_Filter_init(&l_pBW_BS->m_sp);
        for(int i=0;i<N_BQ;i++){
            BQ_Filter_init(&l_pBW_BS->m_bq[i]);
        }
    }
    return (LADSPA_Handle)l_pBW_BS;
}

static void BW_BS_connect_port(
        LADSPA_Handle p_pInstance,
        unsigned long p_port,
        LADSPA_Data *p_pdata)
{
    BW_BS_Data *l_pBW_BS = (BW_BS_Data*)p_pInstance;
    l_pBW_BS->m_pport[p_port] = p_pdata;
}

static void BW_BS_run(
        LADSPA_Handle p_pInstance,
        unsigned long p_sample_count)
{
    BW_BS_Data *l_pBW_BS = (BW_BS_Data*)p_pInstance;
    LADSPA_Data *l_psrc = l_pBW_BS->m_pport[PORT_IN];
    LADSPA_Data *l_pdst = l_pBW_BS->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;

    int N = (int)*l_pBW_BS->m_pport[PORT_N];
    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pBW_BS->m_pport[PORT_FREQUENCY];
    l_omega /= l_pBW_BS->m_sample_rate;
    LADSPA_Data l_K = 1.0f/tanf(l_omega/2.0f);
    LADSPA_Data l_Q = *l_pBW_BS->m_pport[PORT_Q];
    BW_BS_set(l_pBW_BS, N, l_K, l_Q);

    for(;l_psrc!=l_psrc_end;l_psrc++,l_pdst++){
        *l_pdst = BW_BS_eval(l_pBW_BS, *l_psrc);
    }
}

static void BW_BS_cleanup( LADSPA_Handle p_pInstance )
{
    free( p_pInstance );
}

static LADSPA_PortDescriptor BW_BS_PortDescriptors[]=
{
    LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char *BW_BS_PortNames[]=
{
    "Input",
    "Output",
    "N(Filter order)",
    "Frequency(Hertz)",
    "Q(wc/(w1-w0))"
};

static LADSPA_PortRangeHint BW_BS_PortRangeHints[]=
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

LADSPA_Descriptor BW_BS_Descriptor=
{
    5829,
    "BW_BS",
    LADSPA_PROPERTY_HARD_RT_CAPABLE,
    "Butterworth Band Stop",
    "Timothy William Krause",
    "None",
    PORT_NPORTS,
    BW_BS_PortDescriptors,
    BW_BS_PortNames,
    BW_BS_PortRangeHints,
    NULL,
    BW_BS_instantiate,
    BW_BS_connect_port,
    NULL,
    BW_BS_run,
    NULL,
    NULL,
    NULL,
    BW_BS_cleanup
};
