# twk-ladspa-plugins ![GNU gpl3 logo!](https://www.gnu.org/graphics/gplv3-or-later.png)

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
- 5833 Elliptical band stop filter
- 5834 Pitch Shifter

RBJ = Robert Bristow-Johnson of [Audio-EQ-Cookbook.txt](https://github.com/TimKrause2/twk-ladspa-plugins/blob/main/Audio-EQ-Cookbook.txt)

## Download

All of the plugins reside in `twk.so` . The zip file is available for download [here](https://twkrause.ca/#twk-ladspa-plugins).

## Compilation

In the source directory just type `make`. This will compile
the plugin `twk.so`.

### Other make targets

The makefile has a few other targets.

	$ make asm
	
To generate the assembly of the source files.

	$ make butt_formulas
	
To run maxima and display the Butterworth stages coefficients formulas. Requires maxima to be installed.

	$ make ellip_formulas
	
To run maxima and display the elliptical stages coefficients formulas. Requires maxima to be installed.

## Usage

Copy `twk.so` to a directory referenced by the environment
variable `LADSPA_PATH`. For example:
```
mkdir $HOME/ladspa
cp twk.so $HOME/ladspa
export LADSPA_PATH=/usr/lib/ladspa:$HOME/ladspa
```

And then run your LADSPA program.

To make the export permanent
put	`export LADSPA_PATH=/usr/lib/ladspa:$HOME/ladspa` 
at the end of your .profile file.

## Prerequists to compile

The following packages are necessary for compiling tye plugins. The install examples assume that you are running Ubuntu.

- gcc & make `sudo apt install build-essential`

    Uses GNU specific functions and constants.

- ladspa-sdk `sudo apt install ladspa-sdk`

    This is for ladspa.h.

There are some optional packages depending on what you want to do.

- maxima `sudo apt install maxima`

    If you want to verify the coefficient equations for the Butterworth and elliptic filters.
    
- scilab `sudo apt install scilab`

    If you want to compute the low pass prototype filter coefficients and write the header file.

- jupyterlab & python3-ipywidgets `sudo apt install jupyterlab python3-ipywidgets`

	If you want to interactively design the elliptic low pass filter prototype and see the formulas for the digital filter stage coefficients.



