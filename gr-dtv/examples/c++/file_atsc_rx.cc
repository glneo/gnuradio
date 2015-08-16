#include <stdio.h>

// Include header files for each block used in flowgraph
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/blocks/interleaved_short_to_complex.h>
#include <gnuradio/filter/firdes.h>
#include <gnuradio/filter/pfb_arb_resampler_ccf.h>
#include <gnuradio/dtv/atsc_fpll.h>
#include <gnuradio/filter/dc_blocker_ff.h>
#include <gnuradio/analog/agc_ff.h>
#include <gnuradio/dtv/atsc_sync.h>
#include <gnuradio/dtv/atsc_fs_checker.h>
#include <gnuradio/dtv/atsc_equalizer.h>
#include <gnuradio/dtv/atsc_viterbi_decoder.h>
#include <gnuradio/dtv/atsc_deinterleaver.h>
#include <gnuradio/dtv/atsc_rs_decoder.h>
#include <gnuradio/dtv/atsc_derandomizer.h>
#include <gnuradio/dtv/atsc_depad.h>
#include <gnuradio/blocks/file_sink.h>

#define ATSC_CHANNEL_BW		(6.0e6)
#define ATSC_SYMBOL_RATE	(4.5e6/286*684) // ~10.76 Mbaud
#define ATSC_RRC_SYMS		(8) // filter kernel extends over 2N+1 symbols
#define SAMPLE_RATE		(6.4e6)
#define SPS			(1.3)

using namespace gr;

int main(int argc, char **argv)
{
	char *infile;
	char *outfile;
	
	if(argc == 3)
	{
		infile = argv[1];
		outfile = argv[2];
	}
    	else
    	{
    		printf("usage: %s input_file output_file.ts\n", argv[0]);
        	return -1;
        }
        
	// Construct a top block that will contain flowgraph blocks.
	top_block_sptr tb = make_top_block("file_atsc_rx");
	
	// Read from input file
	blocks::file_source::sptr srcf = blocks::file_source::make(sizeof(short), infile);

	// Convert interleaved shorts (I,Q,I,Q) to complex
	blocks::interleaved_short_to_complex::sptr i2c = blocks::interleaved_short_to_complex::make();

	float output_rate = ATSC_SYMBOL_RATE * SPS;
	
	// Create matched RX filter with RRC response for fractional interpolator.
        int nfilts = 16;
        float filter_rate = SAMPLE_RATE * nfilts;
        float symbol_rate = ATSC_SYMBOL_RATE / 2.0; // One-sided bandwidth of sideband
        float excess_bw = 0.1152; // 1.0 - ( 0.5 * ATSC_SYMBOL_RATE / ATSC_CHANNEL_BW ) // ~10.3%
        int ntaps = (int)( ( 2 * ATSC_RRC_SYMS + 1 ) * SPS * nfilts );
        float interp = output_rate / SAMPLE_RATE;
        float gain = nfilts * symbol_rate / filter_rate;
	std::vector<float> rrc_taps = filter::firdes::root_raised_cosine(gain,		// Filter gain
	                                                                 filter_rate,	// PFB filter prototype rate
	                                                                 symbol_rate,	// ATSC symbol rate
	                                                                 excess_bw,	// ATSC RRC excess bandwidth
	                                                                 ntaps);	// Length of filter

	// ATSC receiver filter/interpolator
        filter::pfb_arb_resampler_ccf::sptr pfb = filter::pfb_arb_resampler_ccf::make(interp, rrc_taps, nfilts);

	// Lock on to pilot tone, shift to DC, then discard Q channel
        dtv::atsc_fpll::sptr pll = dtv::atsc_fpll::make(output_rate);

	// Remove pilot tone now at DC
        filter::dc_blocker_ff::sptr dcr = filter::dc_blocker_ff::make(4096);

	// Normalize signal to proper constellation amplitude
        analog::agc_ff::sptr agc = analog::agc_ff::make(1e-5, 4.0);

	// Synchronize bit and segement timing
	dtv::atsc_sync::sptr btl = dtv::atsc_sync::make(output_rate);

	// Check for correct field sync
	dtv::atsc_fs_checker::sptr fsc = dtv::atsc_fs_checker::make();

	// Equalize channel using training sequences
	dtv::atsc_equalizer::sptr equ = dtv::atsc_equalizer::make();

	// Remove convolutional trellis coding
	dtv::atsc_viterbi_decoder::sptr vit = dtv::atsc_viterbi_decoder::make();

	// Remove convolutional interleaving
	dtv::atsc_deinterleaver::sptr dei = dtv::atsc_deinterleaver::make();

	// Reed-Solomon decode
	dtv::atsc_rs_decoder::sptr rsd = dtv::atsc_rs_decoder::make();

	// Derandomize MPEG2-TS packet
	dtv::atsc_derandomizer::sptr der = dtv::atsc_derandomizer::make();

	// Remove padding from packet
	dtv::atsc_depad::sptr dep = dtv::atsc_depad::make();

	// Write to output file
	blocks::file_sink::sptr outf = blocks::file_sink::make(sizeof(char), outfile);

	// Connect pipeline
	tb->connect(srcf, 0, i2c, 0);
	tb->connect(i2c, 0, pfb, 0);
	tb->connect(pfb, 0, pll, 0);
	tb->connect(pll, 0, dcr, 0);
	tb->connect(dcr, 0, agc, 0);
	tb->connect(agc, 0, btl, 0);
	tb->connect(btl, 0, fsc, 0);
	tb->connect(fsc, 0, equ, 0);
	tb->connect(equ, 0, vit, 0);
	tb->connect(vit, 0, dei, 0);
	tb->connect(dei, 0, rsd, 0);
	tb->connect(rsd, 0, der, 0);
	tb->connect(der, 0, dep, 0);
	tb->connect(dep, 0, outf, 0);

	tb->run();
	
	// Performace counters can be used like this:
	//printf("srcf:      %15f / %8f\n", srcf->pc_work_time(), srcf->pc_nproduced());
	
	return 0;
}
