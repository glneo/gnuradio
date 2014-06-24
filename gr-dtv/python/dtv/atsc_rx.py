#!/usr/bin/env /usr/bin/python
#
# Copyright 2004, 2013-2014 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#
# This module starts the atsc processing chain taking the captured
# off-air signal created with:
#
#  uhd_rx_cfile.py  --samp-rate=6.4e6
#                   -f <center of tv signal channel freq>
#                   -g <appropriate gain for best signal / noise>
#                   -s output shorts
#
# This python script converts from interleaved shorts to the complex data type,
# then multiplies the sample rate by 3, from 6.4e6 to 19.2e6 
# complex samples / sec, then lowpass filters with a cutoff of 3.2MHz
# and a transition band width of .5MHz.  Center of the tv channels is
# now at 0 with edges at -3.2MHz and 3.2MHz. This puts the pilot at
# -3MHz + 309KHz. Next a root raised cosine filter is aplied to match the one
# in the transmitter and thus reduce ISI. The phased locked loop then locks to
# the pilot and outputs just the real part of the signal ( as information is
# not stored in the phase with atsc ), this is then feed to the bit lock
# loop, this looks for the bit sync marker put at the beginning of every segment
# field, this then adjusts the timing so the amplitude will be sampled at the
# correct sample ( sub-sample is used in this case ). 
#
# Output is an MPEG-TS.

from gnuradio import gr, blocks, filter, analog, dtv
import sys, os

ATSC_CHANNEL_BW   = 6.0e6
ATSC_SYMBOL_RATE  = 4.5e6/286*684 # ~10.76 Mbaud
ATSC_RRC_SYMS     = 8             # filter kernel extends over 2N+1 symbols

def graph (args):

	print os.getpid()

	nargs = len(args)
    	if nargs == 2:
		infile = args[0]
        	outfile = args[1]
    	else:
        	raise ValueError('usage: atsc_rx.py input_file output_file.ts\n')

	samp_rate = 6.4e6
	sps = (samp_rate*3)/ATSC_SYMBOL_RATE

	tb = gr.top_block()

	# Read from input file
	srcf = blocks.file_source(gr.sizeof_short, infile)

	# Convert interleaved shorts (I,Q,I,Q) to complex
	i2c = blocks.interleaved_short_to_complex()

	# Create matched RX filter with RRC response for fractional
        # interpolator.
        nfilts = 32
        output_rate = ATSC_SYMBOL_RATE*sps # Desired oversampled sample rate
        filter_rate = samp_rate*nfilts
        symbol_rate = ATSC_SYMBOL_RATE/2.0 # One-sided bandwidth of sideband
        excess_bw = 0.1152 #1.0-(0.5*ATSC_SYMBOL_RATE/ATSC_CHANNEL_BW) # ~10.3%
        ntaps = int((2*ATSC_RRC_SYMS+1)*sps*nfilts)
        interp = output_rate/samp_rate
        gain = nfilts*symbol_rate/filter_rate
        rrc_taps = filter.firdes.root_raised_cosine(gain,             # Filter gain
                                                    filter_rate,      # PFB filter prototype rate
                                                    symbol_rate,      # ATSC symbol rate
                                                    excess_bw,        # ATSC RRC excess bandwidth
                                                    ntaps)            # Length of filter

	# ATSC receiver filter/interpolator
        pfb = filter.pfb_arb_resampler_ccf(interp, rrc_taps, nfilts)

        # Lock on to pilot tone, shift to DC, then discard Q channel
        output_rate = ATSC_SYMBOL_RATE*sps
        pll = dtv.atsc_fpll(output_rate)

        # Remove pilot tone now at DC
        dcr = filter.dc_blocker_ff(1024)

        # Normalize signal to proper constellation amplitude
        agc = analog.agc_ff(1e-5, 4.0)

        # Synchronize bit and segement timing
        btl = dtv.atsc_sync(output_rate)

        # Check for correct field sync
        fsc = dtv.atsc_fs_checker()

        # Equalize channel using training sequences
        equ = dtv.atsc_equalizer()

        # Remove convolutional trellis coding
        vit = dtv.atsc_viterbi_decoder()

	# Remove convolutional interleaving
	dei = dtv.atsc_deinterleaver()

	# Reed-Solomon decode
	rsd = dtv.atsc_rs_decoder()

	# Derandomize MPEG2-TS packet
	der = dtv.atsc_derandomizer()

	# Remove padding from packet
	dep = dtv.atsc_depad()

	# Write to output file
	outf = blocks.file_sink(gr.sizeof_char, outfile)

        # Connect pipeline
        tb.connect(srcf, i2c, pfb, pll, dcr, agc, btl, fsc)
        tb.connect(fsc, equ, vit, dei, rsd, der, dep, outf)

	tb.run()

	print 'srcf:      ' + repr(srcf.pc_work_time()).rjust(15) + ' / ' + repr(srcf.pc_nproduced()).rjust(8)
	print 'i2c:       ' + repr(i2c.pc_work_time()).rjust(15) + ' / ' + repr(i2c.pc_nproduced()).rjust(8)
	print 'pfb:       ' + repr(pfb.pc_work_time()).rjust(15) + ' / ' + repr(pfb.pc_nproduced()).rjust(8)
	print 'pll:       ' + repr(pll.pc_work_time()).rjust(15) + ' / ' + repr(pll.pc_nproduced()).rjust(8)
	print 'dcr:       ' + repr(dcr.pc_work_time()).rjust(15) + ' / ' + repr(dcr.pc_nproduced()).rjust(8)
	print 'agc:       ' + repr(agc.pc_work_time()).rjust(15) + ' / ' + repr(agc.pc_nproduced()).rjust(8)
	print 'btl:       ' + repr(btl.pc_work_time()).rjust(15) + ' / ' + repr(btl.pc_nproduced()).rjust(8)
	print 'fsc:       ' + repr(fsc.pc_work_time()).rjust(15) + ' / ' + repr(fsc.pc_nproduced()).rjust(8)
	print 'equ:       ' + repr(equ.pc_work_time()).rjust(15) + ' / ' + repr(equ.pc_nproduced()).rjust(8)
	print 'vit:       ' + repr(vit.pc_work_time()).rjust(15) + ' / ' + repr(vit.pc_nproduced()).rjust(8)
	print 'dei:       ' + repr(dei.pc_work_time()).rjust(15) + ' / ' + repr(dei.pc_nproduced()).rjust(8)
	print 'rsd:       ' + repr(rsd.pc_work_time()).rjust(15) + ' / ' + repr(rsd.pc_nproduced()).rjust(8)
	print 'der:       ' + repr(der.pc_work_time()).rjust(15) + ' / ' + repr(der.pc_nproduced()).rjust(8)
	print 'dep:       ' + repr(dep.pc_work_time()).rjust(15) + ' / ' + repr(dep.pc_nproduced()).rjust(8)
	print 'outf:      ' + repr(outf.pc_work_time()).rjust(15) + ' / ' + repr(outf.pc_nproduced()).rjust(8)


if __name__ == '__main__':
	graph (sys.argv[1:])

