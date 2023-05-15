# twk-ladspa-plugins

## Compilation

In the source directory just type 'make'. This will compile
the plugin 'twk.so'.

## Usage

Copy 'twk.so' to a directory referenced by the environment
variable 'LADSPA_PATH'. For example:

	mkdir /home/tim/ladspa
	cp twk.so /home/tim/ladspa
	export LADSPA_PATH=/usr/lib/ladspa:/home/tim/ladspa
	
And then run your LADSPA program. To make the export permanent
put
	export LADSPA_PATH=/usr/lib/ladspa:/home/tim/ladspa
at the end of your .bashrc file.




