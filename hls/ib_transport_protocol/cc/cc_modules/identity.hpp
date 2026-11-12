/*
 * Copyright (c) 2019, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include "../../../axi_utils.hpp"
#include "../cc_types.hpp"

/**
 * Identity CC module: never throttles. Every QP stays at line rate (gap 0).
 *
 * On INIT it reports line rate, which keeps tx_rate_table at gap 0 (it already starts there) and keeps
 * the output stream driven. All other events are read and ignored.
 */
template <int INSTID>
void identity_cc_module(
    hls::stream<ccEvent>& cc_init,
    hls::stream<ccEvent>& cc_event,
    hls::stream<ccRateUpd>& cc_rate_upd,
    volatile ap_uint<32>& cycle_cnt
);

template <int INSTID = 0>
void identity_cc_module(
    hls::stream<ccEvent>& cc_init,       // in:  INIT events (from cc_event_fanout, never dropped)
    hls::stream<ccEvent>& cc_event,      // in:  all other signals (ignored)
    hls::stream<ccRateUpd>& cc_rate_upd, // out: line rate on INIT
    volatile ap_uint<32>& cycle_cnt      // in:  unused, part of the common module interface
) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=1

    ccEvent ev;

    // INIT first, then at most one signal (common module processing order)
    if (!cc_init.empty()) {
        cc_init.read(ev);
        cc_rate_upd.write(ccRateUpd(ev.qpn, CC_LINE_RATE_MBPS));
    } else if (!cc_event.empty()) {
        cc_event.read(ev);
    }
}
