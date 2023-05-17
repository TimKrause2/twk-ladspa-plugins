# twk-ladspa-plugins

## Plugins

- Compressor/Expandor
- DC remove
- various delays
- various band pass filters
- various band stop filters
- various low pass filters
- various high pass filters
- low shelf
- high shelf
- peaking EQ
- Linear Prediction based vocoder
- phasers
- adjustable reverb
- sinewave generator
- various impulse train generators

## Compilation

In the source directory just type `make`. This will compile
the plugin `twk.so`.

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




