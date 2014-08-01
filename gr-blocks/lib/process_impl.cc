/* -*- c++ -*- */
/*
 * Copyright 2014 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "process_impl.h"
#include <gnuradio/io_signature.h>
#include <stdexcept>
#include <stdio.h>

namespace gr {
  namespace blocks {

    process::sptr
    process::make(size_t i_itemsize, size_t o_itemsize, const char *command)
    {
      return gnuradio::get_initial_sptr
        (new process_impl(i_itemsize, o_itemsize, command));
    }

    process_impl::process_impl(size_t i_itemsize, size_t o_itemsize, const char *command)
      : block("process",
                      io_signature::make(1, 1, i_itemsize),
                      io_signature::make(1, 1, o_itemsize)),
        d_i_itemsize(i_itemsize), d_o_itemsize(o_itemsize), d_fp(0)
    {
      d_fp = popen(command, "r"); // open process and get pipe file
    }

    process_impl::~process_impl()
    {
      if(d_fp)
        pclose(d_fp);
    }

    int
    process_impl::general_work(int noutput_items,
                                gr_vector_int &ninput_items,
                                gr_vector_const_void_star &input_items,
                                gr_vector_void_star &output_items)
    {
      char *inbuf = (char*)input_items[0];
      char *outbuf = (char*)output_items[0];

      int n_input = ninput_items[0];
      int n_output = noutput_items;
      
      int n_written = 0;

      if(d_fp == NULL)
	throw std::runtime_error("work with process not open");

      while(n_written < noutput_items) {
        int count = fwrite(inbuf, d_i_itemsize, n_input - n_written, d_fp);
        if(count == 0) {
          if(ferror(d_fp)) {
            std::stringstream s;
            s << "process write failed with error " << fileno(d_fp) << std::endl;
            throw std::runtime_error(s.str());
          }
          else { // is EOF
            break;
          }
        }
        n_written += count;
        inbuf += count * d_i_itemsize;
      }

      fflush (d_fp);

      while(n_output) {
	int count = fread(outbuf, d_o_itemsize, n_output, d_fp);

	n_output -= count;
	outbuf += count * d_o_itemsize;

	if(n_output == 0)		// done
	  break;
      }

      if(n_output > 0) {	     		// EOF or error
	if(n_output == noutput_items)       // we didn't read anything; say we're done
	  return -1;
	return noutput_items - n_output;	// else return partial result
      }

      fflush (d_fp);

      return noutput_items;
    }

  } /* namespace blocks */
} /* namespace gr */
