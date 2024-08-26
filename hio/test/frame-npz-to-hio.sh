#!/bin/bash

# Test new hio using just HDF5 C lib.  See:
#  https://github.com/WireCell/wire-cell-toolkit/issues/192
#
# This script requires "adc.npz" and "sig.npz" files in WCT "frame file format"
# (not tdm) to be in PWD.

mydir="$(dirname $(realpath $BASH_SOURCE))"

set -e

wire-cell -L debug -l adc.log --tla-code tags='["orig0"]' \
          -A digitize=yes -A infile=adc.npz -A outfile=adc.hdf \
          $mydir/test-frame-npz-to-hio.jsonnet
wire-cell -L debug -l sig.log --tla-code tags='["gauss0"]' \
          -A digitize=no -A infile=sig.npz -A outfile=sig.hdf \
          $mydir/test-frame-npz-to-hio.jsonnet

python $mydir/frame-npz-to-hio.py adc.hdf adc.pdf '100/frame_orig0'
python $mydir/frame-npz-to-hio.py sig.hdf sig.pdf '100/frame_gauss0'
python $mydir/frame-npz-to-hio.py adc.hdf adc.png '100/frame_orig0'
python $mydir/frame-npz-to-hio.py sig.hdf sig.png '100/frame_gauss0'
