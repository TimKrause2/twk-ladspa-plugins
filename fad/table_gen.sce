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

[filepath, filename] = get_absolute_file_path();
cd(filepath);
mode_scilab = getscilabmode();
if mode_scilab=="NWNI" then
    graphics = %f;
else
    graphics = %t;
end

Nwindow = 32; // Size of the window
Fss = 1024; // Super Sampling ratio

n=[-((Nwindow/2)-1):(Nwindow/2)];

//
// sinc((n-alpha)*%pi) = sin(n*%pi - alpha*%pi)/(n*%pi - alpha*%pi)
//
// sin(n*%pi - alpha*%pi) = sin(-alpha*%pi) for n even
//                        =-sin(-alpha*%pi) for n odd
//
function s=sinc_pi(n, alpha)
    sin_alpha = sin(-alpha*%pi);
    s = zeros(n);
    i_s = 1;
    for i=n
        if modulo(i,2)==0 then
            // i even
            if i==0 then
                if alpha*%pi < 1e-9 then
                    s(i_s) = 1.0;
                else
                    s(i_s) = sin_alpha/-alpha/%pi;
                end
            else
                s(i_s) = sin_alpha/((i-alpha)*%pi);
            end
        else
            // i odd
            s(i_s) = -sin_alpha/((i-alpha)*%pi);
        end
        i_s = i_s + 1; 
    end
endfunction

table = zeros(Fss,Nwindow);
f_window = window('hm',Nwindow);
if graphics then winH = waitbar("Calculating table"); end
for i=[0:Fss-1]
	alpha = i/Fss;
    if graphics then waitbar(alpha,winH); end
    //f_sinc = sinc( %pi*(n-alpha) );
    f_sinc = sinc_pi(n,alpha);
	table(i+1,:) = f_sinc .* f_window;
end

table = table/max(abs(table));
if graphics then waitbar(0,"Writing fad_table.h",winH); end
[fd,err] = mopen( "fad_table.h", "wt" );
if err~=0 then
	mprintf("Error opening fad_table.h. err=%d\r\n", err );
    if graphics then close(winH); end
	abort
end
mfprintf( fd, "#define FAD_NWINDOW %d\n", Nwindow );
mfprintf( fd, "#define FAD_FSS %d\n", Fss );

mfprintf( fd, "float g_sinc[%d][%d] =\n", Fss, Nwindow );
mfprintf( fd, "{\n" );

for row=1:Fss
    if graphics then waitbar((row-1)/Fss,winH); end
	mfprintf( fd, "\t{" );
	for col=1:Nwindow
		if col==Nwindow then
			mfprintf( fd, "%+.9e }", table(row,col) );
		else
			mfprintf( fd, "%+.9e,", table(row,col) );
		end
	end
	if row==Fss then
		mfprintf( fd, "\n};" );
	else
		mfprintf( fd, ",\n" );
	end
end

mclose( fd );
if graphics then close(winH); end
