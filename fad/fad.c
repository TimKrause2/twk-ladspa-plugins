#include "fad.h"
#include "fad_table.h"
#include <math.h>

float FadSample( float *p_pBuf, long p_start, long p_Nbuf, float p_alpha )
{
    if(p_alpha < 0.0) p_alpha=0.0;
    if(p_alpha >=1.0) p_alpha=1.0-1.0f/FAD_FSS;
    register float *l_psrc = &p_pBuf[p_start];
	long l_Nloop1;
	long l_Nloop2;
	
	if( p_start + FAD_NWINDOW - 1 >= p_Nbuf ){
		l_Nloop1 = p_Nbuf - 1 - p_start;
		l_Nloop2 = FAD_NWINDOW - l_Nloop1;
	}else{
		l_Nloop1 = FAD_NWINDOW;
		l_Nloop2 = 0;
	}
	
	long i;
	register float l_r = 0.0;
	register float *l_psinc = &g_sinc[ (long)floorf(p_alpha*FAD_FSS) ][0];
	for( i = l_Nloop1; i; i--){
		l_r += *l_psrc++ * *l_psinc++;
	}
	if( l_Nloop2){
		l_psrc = p_pBuf;
		for( i = l_Nloop2; i; i--){
			l_r += *l_psrc++ * *l_psinc++;
		}
	}
	
	return l_r;
}

unsigned long FadNwindow( void )
{
	return FAD_NWINDOW;
}
