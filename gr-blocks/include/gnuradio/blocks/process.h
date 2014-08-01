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

#ifndef INCLUDED_BLOCKS_PROCESS_H
#define INCLUDED_BLOCKS_PROCESS_H

#include <gnuradio/blocks/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace blocks {

    /*!
     * \brief Pipe data through an external process.
     * \ingroup processing
     */
    class BLOCKS_API process : virtual public block
    {
    public:
      // gr::blocks::process::sptr
      typedef boost::shared_ptr<process> sptr;

      /*!
       * \brief Make a process.
       * \param i_itemsize size of the input data items.
       * \param o_itemsize size of the output data items.
       * \param command name of the command to execute and pipe data through.
       */
      static sptr make(size_t i_itemsize, size_t o_itemsize, const char *command);
    };

  } /* namespace blocks */
} /* namespace gr */

#endif /* INCLUDED_BLOCKS_PROCESS_H */
