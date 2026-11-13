/*
 * Copyright (c) 2026, Systems Group, ETH Zurich
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
#include "../../ib_transport_protocol/cc/cc_types.hpp"

/**
 * External identity CC module (ID CC_MOD_ID_EXT_IDENTITY = 16): the test module for the external CC path.
 *
 * Same behavior as the internal identity module: reports line rate on INIT, ignores every other event.
 * Selecting it at runtime (cyt_attr_cc_module = 16) therefore never throttles, but every INIT makes a
 * full round trip: stack -> cc_ext_modules -> this IP -> cc_ext_modules -> stack (cc_rate_mux).
 *
 * Interface contract for every external CC module (see custom-wiki/cc-architecture-plan.md, section 6):
 *   - s_axis_cc_init:  INIT events (ccEvent). Never dropped upstream, must be processed before later signals
 *   - s_axis_cc_event: all other signals (ccEvent), in arrival order. Dropped upstream if this module falls
 *                      behind, so the module must accept one event every <= 9 cycles
 *   - m_axis_cc_rate:  rate updates (ccExtRateUpd), tagged with the module's own ID
 *   - ap_ctrl_none, free running in nclk, active-low reset ap_rst_n
 */
void cc_ext_identity_top(
    hls::stream<ccEvent>& s_axis_cc_init,
    hls::stream<ccEvent>& s_axis_cc_event,
    hls::stream<ccExtRateUpd>& m_axis_cc_rate
) {
    #pragma HLS INTERFACE ap_ctrl_none port=return
    #pragma HLS INTERFACE axis register port=s_axis_cc_init
    #pragma HLS INTERFACE axis register port=s_axis_cc_event
    #pragma HLS INTERFACE axis register port=m_axis_cc_rate
    #pragma HLS aggregate variable=s_axis_cc_init compact=bit
    #pragma HLS aggregate variable=s_axis_cc_event compact=bit
    #pragma HLS aggregate variable=m_axis_cc_rate compact=bit
    #pragma HLS PIPELINE II=1 style=flp

    ccEvent ev;

    // INIT first, then at most one signal (common module processing order)
    if (!s_axis_cc_init.empty()) {
        s_axis_cc_init.read(ev);
        m_axis_cc_rate.write(ccExtRateUpd(CC_MOD_ID_EXT_IDENTITY, ev.qpn, CC_LINE_RATE_MBPS));
    } else if (!s_axis_cc_event.empty()) {
        s_axis_cc_event.read(ev);
    }
}
