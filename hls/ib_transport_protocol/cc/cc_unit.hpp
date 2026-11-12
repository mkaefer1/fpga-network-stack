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
#include "cc_types.hpp"
#include "cc_signal_table.hpp"
#include "cc_event_fanout.hpp"
#include "cc_rate_to_gap.hpp"
#include "tx_rate_table.hpp"
#include "cc_modules/identity.hpp"
#include "cc_modules/dcqcn.hpp"
#include <rocev2_config.hpp> // CC_MODULE_<NAME>

#if (defined(CC_MODULE_DCQCN) + defined(CC_MODULE_IDENTITY)) != 1
#error "Exactly one CC module must be selected (CC_MODULE in rocev2/CMakeLists.txt: DCQCN or IDENTITY)"
#endif

/**
 * Congestion-control unit: groups the CC components behind one call.
 *
 * Inlined, so after synthesis its processes are ordinary processes of the enclosing dataflow region
 * (same clock, all parallel). The port streams are passed straight to the components; only the streams
 * between the components are declared here.
 *
 *   ccSignalTable_init_req ─┐                                       ┌─ cc_init ──┐
 *   ccSignalTable_cnp_req  ─┴→ cc_signal_table ─→ cc_event_fanout ─┤            ├→ CC module ─→ cc_rate_to_gap ─→ tx_rate_table ⇄ lookups
 *                                                                   └─ cc_event ─┘  (build time)                      txRateTable_init_req ─┘
 *
 * The CC module is chosen at build time (CC_MODULE). Phase 3 compiles several modules in and selects one at runtime.
 */
template <int INSTID = 0>
void cc_unit(
    hls::stream<ccEvent>& ccSignalTable_cnp_req,         // in:  CNPs (rx_ibh_fsm)
    hls::stream<ap_uint<16> >& ccSignalTable_init_req,   // in:  QP setup (qp_interface)
    hls::stream<ap_uint<16> >& txRateTable_init_req,     // in:  QP setup (qp_interface)
    hls::stream<ap_uint<16> >& txRateTable_lrh_qpn_req,  // in:  rate lookup (local_req_handler)
    hls::stream<bool>& txRateTable_lrh_rsp,              // out: rate lookup response (true = may send now)
    hls::stream<ap_uint<16> >& txRateTable_hrr_qpn_req,  // in:  rate lookup (handle_read_requests)
    hls::stream<bool>& txRateTable_hrr_rsp,              // out: rate lookup response (true = may send now)
    ap_uint<32>& regCcSignalInvalidCount,                // out: counter, signals dropped (QP invalid)
    ap_uint<32>& regRateDeniedCount,                     // out: counter, rate lookups answered "denied"
    volatile ap_uint<32>& cycle_cnt                      // in:  free-running cycle counter (nclk)
) {
    #pragma HLS INLINE

    // Streams between the CC components
    static hls::stream<ccEvent> cc_event_to_fanout("cc_event_to_fanout");
    static hls::stream<ccEvent> mod_cc_init("mod_cc_init");
    static hls::stream<ccEvent> mod_cc_event("mod_cc_event");
    static hls::stream<ccRateUpd> mod_cc_rate_upd("mod_cc_rate_upd");
    static hls::stream<txRateReq> txRateTable_upd_req("txRateTable_upd_req");
    #pragma HLS STREAM depth=4 variable=cc_event_to_fanout
    #pragma HLS STREAM depth=4 variable=mod_cc_init
    // mod_cc_event absorbs signal bursts at module II <= 9
    #pragma HLS STREAM depth=32 variable=mod_cc_event
    #pragma HLS STREAM depth=2 variable=mod_cc_rate_upd
    #pragma HLS STREAM depth=2 variable=txRateTable_upd_req
    #if defined(__VITIS_HLS__)
        #pragma HLS aggregate variable=cc_event_to_fanout compact=bit
        #pragma HLS aggregate variable=mod_cc_init compact=bit
        #pragma HLS aggregate variable=mod_cc_event compact=bit
        #pragma HLS aggregate variable=mod_cc_rate_upd compact=bit
        #pragma HLS aggregate variable=txRateTable_upd_req compact=bit
    #else
        #pragma HLS DATA_PACK variable=cc_event_to_fanout
        #pragma HLS DATA_PACK variable=mod_cc_init
        #pragma HLS DATA_PACK variable=mod_cc_event
        #pragma HLS DATA_PACK variable=mod_cc_rate_upd
        #pragma HLS DATA_PACK variable=txRateTable_upd_req
    #endif

    cc_signal_table<INSTID>(
        ccSignalTable_init_req,
        ccSignalTable_cnp_req,
        cc_event_to_fanout,
        regCcSignalInvalidCount,
        cycle_cnt
    );

    cc_event_fanout<INSTID>(
        cc_event_to_fanout,
        mod_cc_init,
        mod_cc_event
    );

#if defined(CC_MODULE_IDENTITY)
    identity_cc_module<INSTID>(
        mod_cc_init,
        mod_cc_event,
        mod_cc_rate_upd,
        cycle_cnt
    );
#else // CC_MODULE_DCQCN
    dcqcn_cc_module<INSTID>(
        mod_cc_init,
        mod_cc_event,
        mod_cc_rate_upd,
        cycle_cnt
    );
#endif

    cc_rate_to_gap<INSTID>(
        mod_cc_rate_upd,
        txRateTable_upd_req
    );

    tx_rate_table<INSTID>(
        txRateTable_upd_req,
        txRateTable_init_req,
        txRateTable_lrh_qpn_req,
        txRateTable_lrh_rsp,
        txRateTable_hrr_qpn_req,
        txRateTable_hrr_rsp,
        regRateDeniedCount,
        cycle_cnt
    );
}
