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
 *		             1/a0*(b0 + b1*z^-1 + b2*z^-2 + b3*z^-3 + b4*z^-4)
 *   Hbs_stage(z) = ---------------------------------------------------
 *                    1 + (a1/a0)*z^-1 + (a2/a0)*z^-2 + (a3/a0)*z^-3
 *                      + (a4/a0)*z^-4
 *
 * b0 = K^2 + (K^4+2*K^2+1)*Q^2*cn0
 * b1 = (4-4*K^4)*Q^2*cn0
 * b2 = (6*K^4-4*K^2+6)*Q^2*cn0 - 2*K^2
 * b3 = b1
 * b4 = b0
 * a0 = (K^3+K)*Q*cd1 + K^2 + (K^4+2*K^2+1)*Q^2*cd0
 * a1 = (2*K-2*K^3)*Q*cd1 + (4-4*K^4)*Q^2*cd0
 * a2 = (6*K^4-4*K^2+6)*Q^2*cd0 - 2*K^2
 * a3 = (2*K^3-2*K)*Q*cd1 + (4-4*K^4)*Q^2*cd0
 * a4 = -(K^3+K)*Q*cd1 + K^2 + (K^4+2K^2+1)*Q^2*cd0
 *
 * omega = 2 * pi * fc / fs
 * K = 1/tan(omega/2)
 * Q = fc/(f2-f1)
 * fc = sqrt(f1*f2)
 */

enum {
    PORT_IN,
    PORT_OUT,
    PORT_FREQUENCY,
    PORT_Q,
    PORT_NPORTS
};

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
} BQ_Data;

static void BQ_init( BQ_Data *bq)
{
    bq->m_z[1] = 0.0;
    bq->m_z[2] = 0.0;
    bq->m_z[3] = 0.0;
    bq->m_z[4] = 0.0;
}

static void BQ_set( BQ_Data *bq, double K, double Q, ec_stage *ec)
{
    double cd1 = ec->cden1;
    double cd0 = ec->cden0;
    double cn0 = ec->cnum0;
    double K2 = K*K;
    double K3 = K2*K;
    double K4 = K2*K2;
    double Q2 = Q*Q;
    double a0 = (K3+K)*Q*cd1 + K2 + (K4+2.0*K2+1.0)*Q2*cd0;
    double a1 = (2.0*K-2.0*K3)*Q*cd1 + (4.0-4.0*K4)*Q2*cd0;
    double a2 = (6.0*K4-4.0*K2+6.0)*Q2*cd0 - 2.0*K2;
    double a3 = (2.0*K3-2.0*K)*Q*cd1 + (4.0-4.0*K4)*Q2*cd0;
    double a4 = -(K3+K)*Q*cd1 + K2 + (K4+2.0*K2+1.0)*Q2*cd0;

    double b0 = K2 + (K4+2.0*K2+1.0)*Q2*cn0;
    double b1 = (4.0-4.0*K4)*Q2*cn0;
    double b2 = (6.0*K4-4.0*K2+6.0)*Q2*cn0 - 2.0*K2;
    double b3 = b1;
    double b4 = b0;
    bq->m_a1 = a1/a0;
    bq->m_a2 = a2/a0;
    bq->m_a3 = a3/a0;
    bq->m_a4 = a4/a0;
    bq->m_b0 = b0/a0;
    bq->m_b1 = b1/a0;
    bq->m_b2 = b2/a0;
    bq->m_b3 = b3/a0;
    bq->m_b4 = b4/a0;
}

static double BQ_eval(BQ_Data *bq, double x)
{
    bq->m_z[0] = x - bq->m_a1*bq->m_z[1] - bq->m_a2*bq->m_z[2]
            -bq->m_a3*bq->m_z[3] - bq->m_a4*bq->m_z[4];
    double y = bq->m_b0*bq->m_z[0] + bq->m_b1*bq->m_z[1]
            + bq->m_b2*bq->m_z[2] + bq->m_b3*bq->m_z[3]
            + bq->m_b4*bq->m_z[4];
    bq->m_z[4] = bq->m_z[3];
    bq->m_z[3] = bq->m_z[2];
    bq->m_z[2] = bq->m_z[1];
    bq->m_z[1] = bq->m_z[0];
    return y;
}

typedef struct {
    LADSPA_Data  m_sample_rate;
    LADSPA_Data *m_pport[PORT_NPORTS];
    BQ_Data      m_bqs[N_STAGES];
} Ellip_BS_Data;

static void Ellip_BS_set( Ellip_BS_Data *ed, double K, double Q)
{
    for(int i=0;i<N_STAGES;i++){
        BQ_set(&ed->m_bqs[i], K, Q, &ec_stages[i]);
    }
}

static LADSPA_Data Ellip_BS_eval(
        Ellip_BS_Data *ed,
        LADSPA_Data x)
{
    double a = x;
    for(int i=0;i<N_STAGES;i++){
        a = BQ_eval(&ed->m_bqs[i], a);
    }
    a *= ec_gain;
    return (LADSPA_Data)a;
}

static LADSPA_Handle Ellip_BS_instantiate(
    const struct _LADSPA_Descriptor *p_pDescriptor,
    unsigned long p_sample_rate)
{
    Ellip_BS_Data *l_pEllip_BS = malloc( sizeof(Ellip_BS_Data) );
    if(l_pEllip_BS){
        l_pEllip_BS->m_sample_rate = (float)p_sample_rate;
        for(int i=0;i<N_STAGES;i++){
            BQ_init(&l_pEllip_BS->m_bqs[i]);
        }
    }
    return (LADSPA_Handle)l_pEllip_BS;
}

static void Ellip_BS_connect_port(
        LADSPA_Handle p_pInstance,
        unsigned long p_port,
        LADSPA_Data *p_pdata)
{
    Ellip_BS_Data *l_pEllip_BS = (Ellip_BS_Data*)p_pInstance;
    l_pEllip_BS->m_pport[p_port] = p_pdata;
}

static void Ellip_BS_run(
        LADSPA_Handle p_pInstance,
        unsigned long p_sample_count)
{
    Ellip_BS_Data *l_pEllip_BS = (Ellip_BS_Data*)p_pInstance;
    LADSPA_Data *l_psrc = l_pEllip_BS->m_pport[PORT_IN];
    LADSPA_Data *l_pdst = l_pEllip_BS->m_pport[PORT_OUT];
    LADSPA_Data *l_psrc_end = l_psrc + p_sample_count;

    LADSPA_Data l_omega = 2.0f*M_PIf* *l_pEllip_BS->m_pport[PORT_FREQUENCY];
    l_omega /= l_pEllip_BS->m_sample_rate;
    double l_K = 1.0/tan((double)l_omega/2.0);
    LADSPA_Data l_Q = *l_pEllip_BS->m_pport[PORT_Q];
    Ellip_BS_set(l_pEllip_BS, l_K, l_Q);

    for(;l_psrc!=l_psrc_end;l_psrc++,l_pdst++){
        *l_pdst = Ellip_BS_eval(l_pEllip_BS, *l_psrc);
    }
}

static void Ellip_BS_cleanup( LADSPA_Handle p_pInstance )
{
    free( p_pInstance );
}

static LADSPA_PortDescriptor Ellip_BS_PortDescriptors[]=
{
    LADSPA_PORT_INPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_OUTPUT | LADSPA_PORT_AUDIO,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT | LADSPA_PORT_CONTROL
};

static const char *Ellip_BS_PortNames[]=
{
    "Input",
    "Output",
    "Frequency(Hertz)",
    "Q(fc/(fc2-fc1))"
};

static LADSPA_PortRangeHint Ellip_BS_PortRangeHints[]=
{
    {0,0.0f,0.0f},
    {0,0.0f,0.0f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
     10.0f,20.0e3f},
    {LADSPA_HINT_BOUNDED_BELOW | LADSPA_HINT_BOUNDED_ABOVE |
     LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
     0.1f,10.0f}
};

LADSPA_Descriptor Ellip_BS_Descriptor=
{
    5833,
    "Ellip_BS",
    LADSPA_PROPERTY_HARD_RT_CAPABLE,
    "Elliptical Band Stop",
    "Timothy William Krause",
    "None",
    PORT_NPORTS,
    Ellip_BS_PortDescriptors,
    Ellip_BS_PortNames,
    Ellip_BS_PortRangeHints,
    NULL,
    Ellip_BS_instantiate,
    Ellip_BS_connect_port,
    NULL,
    Ellip_BS_run,
    NULL,
    NULL,
    NULL,
    Ellip_BS_cleanup
};
