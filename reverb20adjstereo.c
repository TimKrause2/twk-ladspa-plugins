#include <ladspa.h>
#define _GNU_SOURCE
#include <math.h>
#include <stdlib.h>

#define N_ALLPASS 20
#define N_COMB 20

#define COMB_T0 0.0351
#define ALLPASS_T0 0.0007708

static int allpass_init_left[N_ALLPASS];
static int allpass_init_right[N_ALLPASS];
static int comb_init_left[N_COMB];
static int comb_init_right[N_COMB];

static void Init_allpass_times(LADSPA_Data sample_rate)
{
    for(int i=0;i<N_ALLPASS;i++){
        float xl = (float)i + (float)drand48()*0.5f;
        float xr = (float)i + (float)drand48()*0.5f;
        float tl = ALLPASS_T0*powf(2.0f, xl*2.1f/N_ALLPASS);
        float tr = ALLPASS_T0*powf(2.0f, xr*2.1f/N_ALLPASS);
        allpass_init_left[i] = tl*sample_rate;
        allpass_init_right[i] = tr*sample_rate;
    }
}

static void Init_comb_times(LADSPA_Data sample_rate)
{
    for(int i=0;i<N_COMB;i++){
        float xl = (float)i + (float)drand48()*0.25f;
        float xr = (float)i + (float)drand48()*0.25f;
        float tl = COMB_T0*powf(2.0f, xl/N_COMB);
        float tr = COMB_T0*powf(2.0f, xr/N_COMB);
        comb_init_left[i] = tl*sample_rate;
        comb_init_right[i] = tr*sample_rate;
    }
}

enum {
	PORT_IN_L,
	PORT_IN_R,
	PORT_OUT_L,
	PORT_OUT_R,
    PORT_WETDRY,
	PORT_ALLPASS_G,
	PORT_T60DB,
	PORT_N_ALLPASS,
	PORT_N_COMB,
	PORT_NPORTS
};

typedef struct {
	unsigned int N;
	LADSPA_Data *current;
	LADSPA_Data *end;
	LADSPA_Data *data;
} CyclicBuffer;

static CyclicBuffer *cb_new(unsigned int N)
{
	CyclicBuffer *cb = (CyclicBuffer*)malloc(sizeof(CyclicBuffer));
	if(!cb)return NULL;
	cb->N = N;
	cb->data = (LADSPA_Data*)malloc(sizeof(LADSPA_Data)*N);
	cb->current = cb->data;
	cb->end = &cb->data[N];
	unsigned int i;
	for(i=0;i<N;i++){
		cb->data[i]=0.0;
	}
	return cb;
}

static void cb_destroy(CyclicBuffer *cb)
{
	free(cb->data);
	free(cb);
}

static inline LADSPA_Data cb_read(CyclicBuffer *cb)
{
	return *cb->current;
}

static inline void cb_write(CyclicBuffer *cb,LADSPA_Data x)
{
	*cb->current = x;
	if(++cb->current == cb->end) cb->current=cb->data;
}

static void cb_zero(CyclicBuffer *cb)
{
	int n;
	for(n=0;n<cb->N;n++){
		cb->data[n]=0.0;
	}
}

typedef struct {
	CyclicBuffer *cb;
	LADSPA_Data g;
} APF;

static APF* apf_new(unsigned int N)
{
	APF *apf = (APF*)malloc(sizeof(APF));
	apf->cb = cb_new(N);
	apf->g = 0.5f;
	return apf;
}

static void apf_destroy(APF *apf)
{
	cb_destroy(apf->cb);
	free(apf);
}

static inline LADSPA_Data apf_evaluate(APF *apf,LADSPA_Data x_in)
{
	LADSPA_Data z=cb_read(apf->cb);
	LADSPA_Data s=x_in + apf->g*z;
	cb_write(apf->cb,s);
	return z - apf->g*s;
}

typedef struct {
	CyclicBuffer *cb;
	LADSPA_Data g;
} FBCF;

static FBCF* fbcf_new(unsigned int N)
{
	FBCF *fbcf = (FBCF*)malloc(sizeof(FBCF));
	if(!fbcf)return NULL;
	fbcf->cb = cb_new(N);
	fbcf->g = 0.5f;
	return fbcf;
}

static void fbcf_destroy(FBCF *fbcf)
{
	cb_destroy(fbcf->cb);
	free(fbcf);
}

static inline LADSPA_Data fbcf_evaluate(FBCF *fbcf,LADSPA_Data x_in)
{
	LADSPA_Data y=x_in + fbcf->g*cb_read(fbcf->cb);
	cb_write(fbcf->cb,y);
	return y;
}

typedef struct {
	unsigned long sample_rate;
	LADSPA_Data *port[PORT_NPORTS];
	APF *apfs_l[N_ALLPASS];
	APF *apfs_r[N_ALLPASS];
	FBCF *fbcfs_l[N_COMB];
	FBCF *fbcfs_r[N_COMB];
	unsigned int n_allpass_prev;
	unsigned int n_comb_prev;
}
 Reverb;

static LADSPA_Handle Reverb_instantiate(const struct _LADSPA_Descriptor * Descriptor,
                               unsigned long                     SampleRate)
{
	Reverb *r=(Reverb*)malloc(sizeof(Reverb));
	if(!r)return NULL;
    Init_allpass_times(SampleRate);
    Init_comb_times(SampleRate);

	r->sample_rate = SampleRate;
	int i;
	for(i=0;i<N_ALLPASS;i++){
		r->apfs_l[i] = apf_new(allpass_init_left[i]);
		r->apfs_r[i] = apf_new(allpass_init_right[i]);
	}
	for(i=0;i<N_COMB;i++){
		r->fbcfs_l[i] = fbcf_new(comb_init_left[i]);
		r->fbcfs_r[i] = fbcf_new(comb_init_right[i]);
	}
	r->n_allpass_prev = 0;
	r->n_comb_prev = 0;
	return (LADSPA_Handle)r;
}

static void Reverb_connect_port(LADSPA_Handle Instance,
                        unsigned long Port,
                        LADSPA_Data * DataLocation)
{
	Reverb *r=(Reverb*)Instance;
	r->port[Port] = DataLocation;
}

static void Reverb_run(LADSPA_Handle Instance,
              unsigned long SampleCount)
{
	Reverb *r=(Reverb*)Instance;
	LADSPA_Data *src_l = r->port[PORT_IN_L];
	LADSPA_Data *src_r = r->port[PORT_IN_R];
	LADSPA_Data *dst_l = r->port[PORT_OUT_L];
	LADSPA_Data *dst_r = r->port[PORT_OUT_R];
	LADSPA_Data g=-*r->port[PORT_ALLPASS_G];
	LADSPA_Data t60db=*r->port[PORT_T60DB];
    LADSPA_Data mix=*r->port[PORT_WETDRY];
	unsigned int n_allpass = (unsigned int)*r->port[PORT_N_ALLPASS];
	unsigned int n_comb = (unsigned int)*r->port[PORT_N_COMB];
	
	unsigned long i;

    LADSPA_Data A_dry=mix;
    LADSPA_Data A_wet=1.0-mix;
	
	// initialize filters coming online
	if(n_allpass > r->n_allpass_prev){
		for(i=r->n_allpass_prev;i<n_allpass;i++){
			cb_zero(r->apfs_l[i]->cb);
			cb_zero(r->apfs_r[i]->cb);
		}
	}
	
	if(n_comb > r->n_comb_prev){
		for(i=r->n_comb_prev;i<n_comb;i++){
			cb_zero(r->fbcfs_l[i]->cb);
			cb_zero(r->fbcfs_r[i]->cb);
		}
	}
	
	// initialize the gain coefficients for the active filters
	for(i=0;i<n_allpass;i++){
		r->apfs_l[i]->g = g;
		r->apfs_r[i]->g = g;
	}
	
	LADSPA_Data alpha = powf(10.0,-60/20);
	for(i=0;i<n_comb;i++){
		r->fbcfs_l[i]->g = -powf(alpha,(float)r->fbcfs_l[i]->cb->N/r->sample_rate/t60db);
		r->fbcfs_r[i]->g = -powf(alpha,(float)r->fbcfs_r[i]->cb->N/r->sample_rate/t60db);
	}
	
	for(i=SampleCount;i;i--){
		LADSPA_Data x_l=*src_l;
		LADSPA_Data x_r=*src_r;
		if(n_allpass){
			int a;
			APF **apf_l = r->apfs_l;
			APF **apf_r = r->apfs_r;
			for(a=n_allpass;a;a--){
				x_l = apf_evaluate(*apf_l,x_l);
				x_r = apf_evaluate(*apf_r,x_r);
				apf_l++;
				apf_r++;
			}
		}
		LADSPA_Data s_l=0.0;
		LADSPA_Data s_r=0.0;
		if(n_comb){
			int c;
			FBCF **fbcf_l = r->fbcfs_l;
			FBCF **fbcf_r = r->fbcfs_r;
			for(c=n_comb;c;c--){
				s_l+=fbcf_evaluate(*fbcf_l,x_l);
				s_r+=fbcf_evaluate(*fbcf_r,x_r);
				fbcf_l++;
				fbcf_r++;
			}
			s_l/=n_comb;
			s_r/=n_comb;
		}else{
			s_l = x_l;
			s_r = x_r;
		}
        *dst_l = s_l*A_wet + *src_l*A_dry;
        *dst_r = s_r*A_wet + *src_r*A_dry;
		src_l++;
		src_r++;
		dst_l++;
		dst_r++;
	}
	r->n_allpass_prev = n_allpass;
	r->n_comb_prev = n_comb;
}

static void Reverb_cleanup(LADSPA_Handle Instance)
{
	Reverb *r=(Reverb*)Instance;
	int i;
	for(i=0;i<N_ALLPASS;i++){
		apf_destroy(r->apfs_l[i]);
		apf_destroy(r->apfs_r[i]);
	}
	for(i=0;i<N_COMB;i++){
		fbcf_destroy(r->fbcfs_l[i]);
		fbcf_destroy(r->fbcfs_r[i]);
	}
	free(r);
}

static LADSPA_PortDescriptor Reverb_PortDescriptors[]=
{
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_INPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
	LADSPA_PORT_OUTPUT|LADSPA_PORT_AUDIO,
    LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
    LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL,
	LADSPA_PORT_INPUT|LADSPA_PORT_CONTROL
};

static const char *Reverb_PortNames[]=
{
	"Input left",
	"Input right",
	"Output left",
	"Output right",
    "Wet/Dry Mix",
	"Allpass g",
	"Comb decay time (t 60dB)",
	"Number of Allpass filters",
	"Number of Comb filters"
};

static LADSPA_PortRangeHint Reverb_PortRangeHints[]=
{
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
    {LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
        LADSPA_HINT_DEFAULT_MIDDLE,
        0.0,1.0},
    {LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MIDDLE,
		0.01,0.995},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_LOGARITHMIC|LADSPA_HINT_DEFAULT_MINIMUM,
		1.0,1000.0},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_INTEGER|LADSPA_HINT_DEFAULT_MAXIMUM,
		0.0,N_ALLPASS},
	{LADSPA_HINT_BOUNDED_BELOW|LADSPA_HINT_BOUNDED_ABOVE|
		LADSPA_HINT_INTEGER|LADSPA_HINT_DEFAULT_MAXIMUM,
		0.0,N_COMB},
};

LADSPA_Descriptor Reverb20AdjStereo_Descriptor=
{
	5824,
	"reverb20adjstereo",
	LADSPA_PROPERTY_HARD_RT_CAPABLE,
	"Reverb 20 adjustable stereo",
	"Timothy William Krause",
	"None",
	PORT_NPORTS,
	Reverb_PortDescriptors,
	Reverb_PortNames,
	Reverb_PortRangeHints,
	NULL,
	Reverb_instantiate,
	Reverb_connect_port,
	NULL,
	Reverb_run,
	NULL,
	NULL,
	NULL,
	Reverb_cleanup
};
