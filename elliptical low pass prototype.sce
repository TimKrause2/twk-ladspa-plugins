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

[homepath,scriptname]=get_absolute_file_path();
cd(homepath);

Nstages = 5;
N=2*Nstages;
fdesign="ellip";
ripple=[0.01,0.0005];
omega=1.0;
[Hs,Hs_poles,Hs_zeros,Hs_gain] = analpf(N,fdesign,ripple,omega);
fmin = 0.01/2/%pi;
fmax = 100.0/2/%pi;
Nsteps = 2000;
step = (log10(fmax)-log10(fmin))/Nsteps;
[f,repf] = repfreq(Hs, fmin, fmax, step);
clf();
bode(f,repf,"rad");

/*
The elliptic filter biquad stage prototype transfer function.

Each stage is the ratio of two quadratic functions where each
quadratic function is the product of two linear binomials.

               (s - zero(i))*(s - conj(zero(i)))
Hlp_stage(i) = ---------------------------------
               (s - pole(i))*(s - conj(pole(i)))

where:
    zero(i) = zero for stage i
    pole(i) = pole for stage i
    conj(a) = conjugate of a

Expanding one of the polynomials.

    (s - a)*(s - conj(a)) = s^2 -(a+conj(a))*s + a*conj(a)
                          = s^2 - 2*real(a)*s + a*conj(a)

    real(a) = real component of a

Since the zeros are purely imaginary for the elliptic filter the
coefficient for s is zero and the final form for the stage is:

                      s^2 + zero(i)*conj(zero(i))
Hlp_stage(i) = ------------------------------------------------
                s^2 - 2*real(pole(i))*s + pole(i)*conj(pole(i))

                   s^2 + cnum0
             = -------------------
                s^2 + cden1*s + cden0

    cnum0 = "coefficient numerator" power 0 = zero(i)*conj(zero(i))
    cden0 = "coefficient denominator" power 0 = pole(i)*conj(pole(i))
    cden1 = "coefficient denominator" power 1 = -2*real(pole(i))
*/

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

