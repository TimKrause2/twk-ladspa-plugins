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
 *                    1
 *  Hlp(s) = --------------------
 *              s^2 + c*s + 1
 *
 * Transfer function for a single biquad filter
 *
 *		           1/a0*(b0 + b1*z^-1 + b2*z^-2)
 *   Hbq(z) = ---------------------------------------
 *                1 + (a1/a0)*z^-1 + (a2/a0)*z^-2
 *
 * b0 = 1
 * b1 = 2
 * b2 = 1
 * a0 = K^2 + K*c + 1
 * a1 = 2 - 2*K^2
 * a2 = K^2 - K*c + 1
 *
 * omega = 2 * pi * fc / fs
 * K = 1/tan(omega/2)
 *
 *
 * Transfer function for a single real pole
 *
 *              1/a0*(1 + z^-1)
 *  Hsp(z) = ------------------------
 *             1 + (a1/a0)*z^-1
 *
 * a0 = 1 + K
 * a1 = 1 - K
 */

enum {
    PORT_IN,
    PORT_OUT,
    PORT_N,
    PORT_FREQUENCY,
    PORT_NPORTS
};

typedef struct {
    LADSPA_Data m_z1;
    LADSPA_Data m_a1;
    LADSPA_Data m_g;
} SP_Filter;

static void SP_Filter_init(SP_Filter *sp)
{
    sp->m_z1 = 0.0;
}

static void SP_Filter_set(SP_Filter *sp, LADSPA_Data K)
{
    LADSPA_Data l_a0 = K + 1.0f;
    LADSPA_Data l_a1 = 1.0f - K;
    sp->m_a1 = l_a1/l_a0;
    sp->m_g = 1.0f/l_a0;
}

static LADSPA_Data SP_Filter_eval(SP_Filter *sp, LADSPA_Data x)
{
    LADSPA_Data l_m = x - sp->m_a1*sp->m_z1;
    LADSPA_Data l_y = l_m + sp->m_z1;
    l_y *= sp->m_g;
    sp->m_z1 = l_m;
    return l_y;
}

typedef struct {
    LADSPA_Data m_z1;
    LADSPA_Data m_z2;
    LADSPA_Data m_a1;
    LADSPA_Data m_a2;
    LADSPA_Data m_g;
} BQ_Filter;

static void BQ_Filter_init(BQ_Filter *bq)
{
    bq->m_z1 = 0.0f;
    bq->m_z2 = 0.0f;
}

static void BQ_Filter_set(BQ_Filter *bq,
                   LADSPA_Data K,
                   LADSPA_Data c)
{
    LADSPA_Data l_a0 = K*K + K*c + 1.0f;
    LADSPA_Data l_a1 = 2.0f - 2.0f*K*K;
    LADSPA_Data l_a2 = K*K - K*c + 1.0f;
    bq->m_a1 = l_a1/l_a0;
    bq->m_a2 = l_a2/l_a0;
    bq->m_g  = 1.0f/l_a0;
}

static LADSPA_Data BQ_Filter_eval(BQ_Filter *bq, LADSPA_Data x)
{
    LADSPA_Data l_m = x - bq->m_a1*bq->m_z1 - bq->m_a2*bq->m_z2;
    LADSPA_Data l_y = l_m + 2.0f*bq->m_z1 + bq->m_z2;
    l_y *= bq->m_g;
    bq->m_z2 = bq->m_z1;
    bq->m_z1 = l_m;
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
} BW_LP_Data;

void BW_LP_set(BW_LP_Data *p_pBW_LP, int p_N, LADSPA_Data p_K)
{
    if((p_N&1) == 0){
        // N even
        int N_bq = p_N/2;
        for(int m=1,bq=0;bq<N_bq;m+=2,bq++){
            LADSPA_Data c = 2.0f*cosf((float)m*M_PIf/2.0f/p_N);
            BQ_Filter_set(&p_pBW_LP->m_bq[bq], p_K, c);
        }
        p_pBW_LP->m_N_bq = N_bq;
        p_pBW_LP->m_sp_on = 0;
    } else {
        // N odd
        SP_Filter_set(&p_pBW_LP->m_sp, p_K);
        int K = (p_N-1)/2;
        p_pBW_LP->m_sp_on = 1;
        p_pBW_LP->m_N_bq = K;
        for(int k=1;k<=K;k++){
            LADSPA_Data c = 2.0f*cosf(k*M_PIf/p_N);
            BQ_Filter_set(&p_pBW_LP->m_bq[k-1], p_K, c);
        }
    }
}

LADSPA_Data BW_LP_eval( BW_LP_Data *p_pBW_LP, LADSPA_Data x)
{
    if(p_pBW_LP->m_sp_on)
        x = SP_Filter_eval(&p_pBW_LP->m_sp, x);
    for(int i=0;i<p_pBW_LP->m_N_bq;i++){
        x = BQ_Filter_eval(&p_pBW_LP->m_bq[i], x);
    }
    return x;
}

static LADSPA_Handle BW_LP_instantiate(
    const struct _LADSPA_Descriptor *p_pDescriptor,
    unsigned long p_sample_rate)
{
    BW_LP_Data *l_pBW_LP = malloc( sizeof(BW_LP_Data) );
    if(l_pBW_LP){
        l_pBW_LP->m_sample_rate = (float)p_sample_rate;
        SP_Filter_init(&l_pBW_LP->m_sp);
        for(int i=0;i<N_BQ;i++){
            BQ_Filter_init(&l_pBW_LP->m_bq[i]);
        }
    }
    return (LADSPA_Handle)l_pBW_LP;
}

static void BW_LP_connect_port(
        LADSPA_Handle p_pInstance,
        unsigned long p_port,
        LADSPA_Data *p_pdata)
{
    BW_LP_Data *l_pBW_LP = (BW_LP_Data*)p_pInstance;
    l_pBW_LP->m_pport[p_port] = p_pdata;
}

static void BW_LP_run(
        LADSPA_Handle p_pInstance,
        unsigned long p_sample_count)
{
    BW_LP_Data *l_pBW_LP = (BW_LP_Data*)p_pInstance;
    LADSPA_Data *l_psrc = l_pBW_LP->m_pport[PORT_IN];
    LADSPA_Data *l_pdst = l_pBW_LP->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;

    int N = (int)*l_pBW_LP->m_pport[PORT_N];
    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pBW_LP->m_pport[PORT_FREQUENCY];
    l_omega /= l_pBW_LP->m_sample_rate;
    LADSPA_Data l_K = 1.0f/tanf(l_omega/2.0f);
    BW_LP_set(l_pBW_LP, N, l_K);

    for(;l_psrc!=l_psrc_end;l_psrc++,l_pdst++){
        *l_pdst = BW_LP_eval(l_pBW_LP, *l_psrc);
    }
}

static void BW_LP_cleanup( LADSPA_Handle p_pInstance )
{
    free( p_pInstance );
}

static LADSPA_PortDescriptor BW_LP_PortDescriptors[]=
{
    LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char *BW_LP_PortNames[]=
{
    "Input",
    "Output",
    "N(Filter order)",
    "Frequency(Hertz)"
};

static LADSPA_PortRangeHint BW_LP_PortRangeHints[]=
{
    {0,0.0f,0.0f},
    {0,0.0f,0.0f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_INTEGER | LADSPA_HINT_DEFAULT_MINIMUM,
     1.0f, 11.0f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
     10.0f,20.0e3f}
};

LADSPA_Descriptor BW_LP_Descriptor=
{
    5826,
    "BW_LP",
    LADSPA_PROPERTY_HARD_RT_CAPABLE,
    "Butterworth Low Pass",
    "Timothy William Krause",
    "None",
    PORT_NPORTS,
    BW_LP_PortDescriptors,
    BW_LP_PortNames,
    BW_LP_PortRangeHints,
    NULL,
    BW_LP_instantiate,
    BW_LP_connect_port,
    NULL,
    BW_LP_run,
    NULL,
    NULL,
    NULL,
    BW_LP_cleanup
};
