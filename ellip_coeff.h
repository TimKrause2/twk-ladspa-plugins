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
#define N_STAGES 5
static double ec_gain=4.99999955577744314e-04;
typedef struct {
 double cden1;
 double cden0;
 double cnum0;
} ec_stage;
static ec_stage ec_stages[N_STAGES]={
 {7.8737555808994752e-01,2.2612518911682630e-01,2.2200334289017736e+01},
 {5.1505026950754629e-01,5.2384818097805030e-01,3.1479567020628543e+00},
 {2.5623536548281067e-01,8.0399425385984968e-01,1.6701585797026577e+00},
 {1.0696635942803545e-01,9.5743148258236688e-01,1.3088362651981131e+00},
 {2.8485290324588155e-02,1.0177492800942483e+00,1.2027898974572733e+00},
};
