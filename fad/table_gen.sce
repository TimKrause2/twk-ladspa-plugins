Nwindow = 32;
Fss = 1024;

n=[-(Nwindow/2):(Nwindow/2)-1];

for i=[0:Fss-1]
	alpha = i/Fss;
	table(i+1,1:Nwindow) = sinc( %pi*(n-alpha) ).*window('hm',Nwindow);
end

alpha = 1.0 / max(abs(table));
table = table * alpha;


[fd,err] = mopen( "fad_table.h", "wt" );
if err~=0 then
	mprintf("Error opening fad_table.h. err=%d\r\n", err );
	exit
end
mfprintf( fd, "#define FAD_NWINDOW %d\n", Nwindow );
mfprintf( fd, "#define FAD_FSS %d\n", Fss );

mfprintf( fd, "float g_sinc[%d][%d] =\n", Fss, Nwindow );
mfprintf( fd, "{\n" );

for row=1:Fss
	mfprintf( fd, "\t{" );
	for col=1:Nwindow
		if col==Nwindow then
			mfprintf( fd, "%15.13e }", table(row,col) );
		else
			mfprintf( fd, "%15.13e,", table(row,col) );
		end
	end
	if row==Fss then
		mfprintf( fd, "\n};" );
	else
		mfprintf( fd, ",\n" );
	end
end

mclose( fd );

