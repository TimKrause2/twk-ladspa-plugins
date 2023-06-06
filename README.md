# twk-ladspa-plugins

## Plugins

- 5801 Compressor/Expandor
- 5802 DC remove
- 5803 Delay
- 5804 Distortion
- 5805 Impulse train generator
- 5806 Voice control impulse train generator
- 5807 All pass filter with LFO control of delay interval
- 5808 Bandpass filter with LFO control of frequency
- 5809 Bandpass filter bank, 5 filters, with LFO control of frequency
- 5810 Delay with LFO control of interval
- 5811 Bandpass filter bank, 5 filters, with random control of frequency
- 5812 Linear prediction based vocoder
- 5813 Phaser based on digital integrator
- 5814 Phaser based on all pass filters
- 5815 RBJ band pass filter with bandwidth control
- 5816 RBJ band pass filter with Q control
- 5817 RBJ high pass filter with Q control
- 5818 RBJ high pass filter bank with Q control
- 5819 RBJ high shelf filter
- 5820 RBJ low pass filter with Q control
- 5821 RBJ low pass filter bank with Q control
- 5822 RBJ low shelf filter
- 5823 RBJ peaking EQ
- 5824 Adjustable reverb
- 5825 Sine wave generator
- 5826 Butterworth low pass filter
- 5827 Butterworth high pass filter
- 5828 Butterworth band pass filter
- 5829 Butterworth band stop filter
- 5830 Elliptical low pass filter
- 5831 Elliptical high pass filter
- 5832 Elliptical band pass filter
- 5833 Elliptical band stop filter'
- 5834 Pitch Shifter

RBJ = Robert Bristow-Johnson of [Audio-EQ-Cookbook.txt](https://github.com/TimKrause2/twk-ladspa-plugins/blob/main/Audio-EQ-Cookbook.txt)



## Compilation

In the source directory just type `make`. This will compile
the plugin `twk.so`.

### Other make targets

The makefile has a few other targets.

	$ make asm
	
To generate the assembly of the source files.

	$ make butt_formulas
	
To run maxima and display the Butterworth stages coefficients.

	$ make ellip_formulas
	
To rum maxima and display the elliptical stages coefficients.

## Usage

Copy `twk.so` to a directory referenced by the environment
variable `LADSPA_PATH`. For example:
```
mkdir /home/tim/ladspa
cp twk.so /home/tim/ladspa
export LADSPA_PATH=/usr/lib/ladspa:/home/tim/ladspa
```

And then run your LADSPA program. To make the export permanent
put	`export LADSPA_PATH=/usr/lib/ladspa:/home/tim/ladspa` 
at the end of your .bashrc file.

## Prerequists to compile

- GNU math library

    Uses GNU specific functions and constant

- ladspa-dev

    This is for ladspa.h

- make
- maxima

    If you want to verify the equations.
    
- scilab

    If you want to compute coefficients.




