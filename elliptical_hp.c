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
#include "ellip_coeff.h"

/*
 *
 * Transfer function for a single biquad filter
 *
 *		                1/a0*(b0 + b1*z^-1 + b2*z^-2)
 *   Hhp_stage(z) = ---------------------------------------
 *                     1 + (a1/a0)*z^-1 + (a2/a0)*z^-2
 *
 * b0 = K^2*cn0 + 1
 * b1 = 2 - 2*K^2*cn0
 * b2 = b0
 * a0 = K*cd1 + K^2*cd0 + 1
 * a1 = 2 - 2*K^2*cd0
 * a2 = -K*cd1 + K^2*cd0 + 1
 *
 * omega = 2 * pi * fc / fs
 * K = 1/tan(omega/2)
 */

enum {
    PORT_IN,
    PORT_OUT,
    PORT_FREQUENCY,
    PORT_NPORTS
};

typedef struct {
    double m_z[3];
    double m_a1;
    double m_a2;
    double m_b0;
    double m_b1;
    double m_b2;
} BQ_Data;

static void BQ_init( BQ_Data *bq)
{
    bq->m_z[1] = 0.0;
    bq->m_z[2] = 0.0;
}

static void BQ_set( BQ_Data *bq, double K, ec_stage *ec)
{
    double cd1 = ec->cden1;
    double cd0 = ec->cden0;
    double cn0 = ec->cnum0;
    double K2 = K*K;
    double a0 = K*cd1 + K2*cd0 + 1.0;
    double a1 = 2.0 - 2.0*K2*cd0;
    double a2 = -K*cd1 + K2*cd0 + 1.0;
    double b0 = K2*cn0 + 1.0;
    double b1 = 2.0 - 2.0*K2*cn0;
    double b2 = b0;
    bq->m_a1 = a1/a0;
    bq->m_a2 = a2/a0;
    bq->m_b0 = b0/a0;
    bq->m_b1 = b1/a0;
    bq->m_b2 = b2/a0;
}

static double BQ_eval(BQ_Data *bq, double x)
{
    bq->m_z[0] = x - bq->m_a1*bq->m_z[1] - bq->m_a2*bq->m_z[2];
    double y = bq->m_b0*bq->m_z[0] + bq->m_b1*bq->m_z[1]
            + bq->m_b2*bq->m_z[2];
    bq->m_z[2] = bq->m_z[1];
    bq->m_z[1] = bq->m_z[0];
    return y;
}

typedef struct {
    LADSPA_Data  m_sample_rate;
    LADSPA_Data *m_pport[PORT_NPORTS];
    BQ_Data      m_bqs[N_STAGES];
} Ellip_HP_Data;

static void Ellip_HP_set( Ellip_HP_Data *ed, double K)
{
    for(int i=0;i<N_STAGES;i++){
        BQ_set(&ed->m_bqs[i], K, &ec_stages[i]);
    }
}

static LADSPA_Data Ellip_HP_eval(
        Ellip_HP_Data *ed,
        LADSPA_Data x)
{
    double a = x;
    for(int i=0;i<N_STAGES;i++){
        a = BQ_eval(&ed->m_bqs[i], a);
    }
    a *= ec_gain;
    return (LADSPA_Data)a;
}

static LADSPA_Handle Ellip_HP_instantiate(
    const struct _LADSPA_Descriptor *p_pDescriptor,
    unsigned long p_sample_rate)
{
    Ellip_HP_Data *l_pEllip_HP = malloc( sizeof(Ellip_HP_Data) );
    if(l_pEllip_HP){
        l_pEllip_HP->m_sample_rate = (float)p_sample_rate;
        for(int i=0;i<N_STAGES;i++){
            BQ_init(&l_pEllip_HP->m_bqs[i]);
        }
    }
    return (LADSPA_Handle)l_pEllip_HP;
}

static void Ellip_HP_connect_port(
        LADSPA_Handle p_pInstance,
        unsigned long p_port,
        LADSPA_Data *p_pdata)
{
    Ellip_HP_Data *l_pEllip_HP = (Ellip_HP_Data*)p_pInstance;
    l_pEllip_HP->m_pport[p_port] = p_pdata;
}

static void Ellip_HP_run(
        LADSPA_Handle p_pInstance,
        unsigned long p_sample_count)
{
    Ellip_HP_Data *l_pEllip_HP = (Ellip_HP_Data*)p_pInstance;
    LADSPA_Data *l_psrc = l_pEllip_HP->m_pport[PORT_IN];
    LADSPA_Data *l_pdst = l_pEllip_HP->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;

    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pEllip_HP->m_pport[PORT_FREQUENCY];
    l_omega /= l_pEllip_HP->m_sample_rate;
    double l_K = 1.0/tan((double)l_omega/2.0);
    Ellip_HP_set(l_pEllip_HP, l_K);

    for(;l_psrc!=l_psrc_end;l_psrc++,l_pdst++){
        *l_pdst = Ellip_HP_eval(l_pEllip_HP, *l_psrc);
    }
}

static void Ellip_HP_cleanup( LADSPA_Handle p_pInstance )
{
    free( p_pInstance );
}

static LADSPA_PortDescriptor Ellip_HP_PortDescriptors[]=
{
    LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char *Ellip_HP_PortNames[]=
{
    "Input",
    "Output",
    "Frequency(Hertz)"
};

static LADSPA_PortRangeHint Ellip_HP_PortRangeHints[]=
{
    {0,0.0f,0.0f},
    {0,0.0f,0.0f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
     10.0f,20.0e3f}
};

LADSPA_Descriptor Ellip_HP_Descriptor=
{
    5831,
    "Ellip_HP",
    LADSPA_PROPERTY_HARD_RT_CAPABLE,
    "Elliptical High Pass",
    "Timothy William Krause",
    "None",
    PORT_NPORTS,
    Ellip_HP_PortDescriptors,
    Ellip_HP_PortNames,
    Ellip_HP_PortRangeHints,
    NULL,
    Ellip_HP_instantiate,
    Ellip_HP_connect_port,
    NULL,
    Ellip_HP_run,
    NULL,
    NULL,
    NULL,
    Ellip_HP_cleanup
};
