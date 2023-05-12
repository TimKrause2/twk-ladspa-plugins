
[homepath,scriptname]=get_absolute_file_path();
cd(homepath);

Nstages = 5;
N=2*Nstages;
fdesign="ellip";
rp=[0.01,0.0005];
omega=1.0;
[Hs,Hs_poles,Hs_zeros,Hs_gain] = analpf(N,fdesign,rp,omega);
fmin = 0.01;
fmax = 100.0;
Nsteps = 2000;
step = (log10(fmax)-log10(fmin))/Nsteps;
[f,repf] = repfreq(Hs, fmin, fmax, step);
clf();
bode(f,repf);

[fd,err] = mopen("ellip_coeff.h","w");
if err<>0 then
    disp("Could not open ellip_coeff.h.");
    halt;
end

mfprintf(fd,"#define N_STAGES %d\n",Nstages);
mfprintf(fd,"static double ec_gain=%.17e;\n",Hs_gain);
mfprintf(fd,"typedef struct {\n");
mfprintf(fd," double cden1;\n");
mfprintf(fd," double cden0;\n");
mfprintf(fd," double cnum0;\n");
mfprintf(fd,"} ec_stage;\n");
mfprintf(fd,"static ec_stage ec_stages[N_STAGES]={\n");
for i=1:Nstages do
    cden1 = -2*real(Hs_poles(i));
    cden0 = real(Hs_poles(i)*conj(Hs_poles(i)));
    cnum0 = real(Hs_zeros(i)*conj(Hs_zeros(i)));
    mfprintf(fd," {%.16e,%.16e,%.16e},\n",cden1,cden0,cnum0);
end
mfprintf(fd,"};\n");
mclose(fd);

