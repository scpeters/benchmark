#!/bin/bash
base=`basename $1 .ogv`
tmpdir=`mktemp -d`
mplayer $1 -ao null $1 -vo jpeg:outdir=${tmpdir}
mogrify -resize 25% ${tmpdir}/*
convert ${tmpdir}/* -delay 2 ${base}.gif

