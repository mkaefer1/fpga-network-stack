/************************************************
Copyright (c) 2019, Systems Group, ETH Zurich.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************/
#pragma once

#include "../../axi_utils.hpp"
#include "cc_signal_table.hpp"
#include "tx_rate_table.hpp"
#include "cc_modules/identity.hpp"

/**
 * Congestion-control unit: groups the CC components behind one call.
 *
 * Inlined, so after synthesis its processes are ordinary processes of the
 * enclosing dataflow region (same clock, all parallel). The port streams are
 * passed straight to the components; only the streams between the components
 * are declared here.
 *
 *   ccSignalTable_upd_req  ─┐
 *   ccSignalTable_init_req ─┴→ cc_signal_table ─→ identity_cc_module ─→ tx_rate_table ⇄ txRateTable_lrh_qpn_req / txRateTable_lrh_rsp (local_req_handler)
 *                                                     txRateTable_init_req ─┘    ⇄ txRateTable_hrr_qpn_req / txRateTable_hrr_rsp (handle_read_requests)
 */
template <int INSTID = 0>
void cc_unit(
    hls::stream<ccSignalReq>& ccSignalTable_upd_req,     // in:  congestion signals (e.g. CNPs from rx_ibh_fsm)
    hls::stream<ap_uint<16> >& ccSignalTable_init_req,   // in:  QP setup from qp_interface
    hls::stream<ap_uint<16> >& txRateTable_init_req,     // in:  QP setup from qp_interface
    hls::stream<ap_uint<16> >& txRateTable_lrh_qpn_req,  // in:  rate lookup (local_req_handler)
    hls::stream<bool>& txRateTable_lrh_rsp,              // out: rate lookup response (true = may send now)
    hls::stream<ap_uint<16> >& txRateTable_hrr_qpn_req,  // in:  rate lookup (handle_read_requests)
    hls::stream<bool>& txRateTable_hrr_rsp               // out: rate lookup response (true = may send now)
) {
    #pragma HLS INLINE

    // Streams between the CC components
    static hls::stream<ccSignalUpdEvent> ccSignalTable_upd_event("ccSignalTable_upd_event");
    static hls::stream<txRateReq> txRateTable_upd_req("txRateTable_upd_req");
    #pragma HLS STREAM depth=2 variable=ccSignalTable_upd_event
    #pragma HLS STREAM depth=2 variable=txRateTable_upd_req
    #if defined(__VITIS_HLS__)
        #pragma HLS aggregate variable=ccSignalTable_upd_event compact=bit
        #pragma HLS aggregate variable=txRateTable_upd_req compact=bit
    #else
        #pragma HLS DATA_PACK variable=ccSignalTable_upd_event
        #pragma HLS DATA_PACK variable=txRateTable_upd_req
    #endif

    cc_signal_table<INSTID>(
        ccSignalTable_upd_req,
        ccSignalTable_init_req,
        ccSignalTable_upd_event
    );

    identity_cc_module<INSTID>(
        ccSignalTable_upd_event,
        txRateTable_upd_req
    );

    tx_rate_table<INSTID>(
        txRateTable_upd_req,
        txRateTable_init_req,
        txRateTable_lrh_qpn_req,
        txRateTable_lrh_rsp,
        txRateTable_hrr_qpn_req,
        txRateTable_hrr_rsp
    );
}
