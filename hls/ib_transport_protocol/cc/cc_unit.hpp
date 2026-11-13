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
#include "cc_modules.hpp"
#include "cc_event_fanout.hpp"
#include "cc_rate_mux.hpp"
#include "cc_rate_to_gap.hpp"
#include "tx_rate_table.hpp"

/**
 * Congestion-control unit: groups the CC components behind one call (see custom-wiki/cc-architecture-plan.md).
 *
 * Inlined, so after synthesis its processes are ordinary processes of the enclosing dataflow region
 * (same clock, all parallel). The port streams are passed straight to the components; only the streams
 * between the components are declared here.
 *
 *   ccSignalTable_init_req    ─┐                                       ┌─ cc_init/cc_event ─→ DCQCN    ─┐
 *   ccSignalTable_cnp_req     ─┤                                       │                                │
 *   ccSignalTable_nak_req     ─┼→ cc_signal_table ─→ cc_event_fanout ─┼─ cc_init/cc_event ─→ IDENTITY ─┼→ cc_rate_mux ─→ cc_rate_to_gap ─→ tx_rate_table ⇄ lookups
 *   ccSignalTable_timeout_req ─┤                                       │                                │   (active module)               txRateTable_init_req ─┘
 *   ccSignalTable_ack_req     ─┘                                       └─ ...  (all of CC_MODULES)    ─┘
 *
 * All compiled-in modules (CC_MODULES) run in parallel on copies of every event. cc_module_sel picks the
 * one whose rates reach tx_rate_table (CC_MOD_SEL_DEFAULT = CC_DEFAULT_MODULE).
 */
template <int INSTID = 0>
void cc_unit(
    hls::stream<ccEvent>& ccSignalTable_cnp_req,         // in:  CNPs (rx_ibh_fsm)
    hls::stream<ccEvent>& ccSignalTable_nak_req,         // in:  NAKs (rx_exh_fsm)
    hls::stream<ccEvent>& ccSignalTable_timeout_req,     // in:  timeouts (transport_timer)
    hls::stream<ccEvent>& ccSignalTable_ack_req,         // in:  ACKs (retransmitter, or rx_ibh_fsm without RETRANS_EN)
    hls::stream<ap_uint<16> >& ccSignalTable_init_req,   // in:  QP setup (qp_interface)
    hls::stream<ap_uint<16> >& txRateTable_init_req,     // in:  QP setup (qp_interface)
    hls::stream<ap_uint<16> >& txRateTable_lrh_qpn_req,  // in:  rate lookup (local_req_handler)
    hls::stream<bool>& txRateTable_lrh_rsp,              // out: rate lookup response (true = may send now)
    hls::stream<ap_uint<16> >& txRateTable_hrr_qpn_req,  // in:  rate lookup (handle_read_requests)
    hls::stream<bool>& txRateTable_hrr_rsp,              // out: rate lookup response (true = may send now)
    ap_uint<32>& regCcSignalInvalidCount,                // out: counter, signals dropped (QP invalid)
    ap_uint<32>& regRateDeniedCount,                     // out: counter, rate lookups answered "denied"
    ap_uint<32>& regCcAckCount,                          // out: counter, ACK events
    ap_uint<32>& regCcAckEceCount,                       // out: counter, ACK events with ECN echo
    ap_uint<32>& regCcNakCount,                          // out: counter, NAK events
    ap_uint<32>& regCcTimeoutCount,                      // out: counter, TIMEOUT events
    ap_uint<32>& regCcEventDropCount,                    // out: counter, events dropped at the fanout input
    ap_uint<32>& regCcModuleDropCount,                   // out: counter, events dropped at the active module's signal FIFO
    volatile ap_uint<32>& cycle_cnt,                     // in:  free-running cycle counter (nclk)
    volatile ap_uint<8>& cc_module_sel                   // in:  active module ID, CC_MOD_SEL_DEFAULT = build default
) {
    #pragma HLS INLINE

    // Streams between the CC components. One INIT FIFO, signal FIFO and rate FIFO per compiled-in module (slot)
    static hls::stream<ccEvent> cc_event_to_fanout("cc_event_to_fanout");
    static hls::stream<ccEvent> mod_cc_init[CC_N_INT_MODULES];
    static hls::stream<ccEvent> mod_cc_event[CC_N_INT_MODULES];
    static hls::stream<ccRateUpd> mod_cc_rate_upd[CC_N_INT_MODULES];
    static hls::stream<ccRateUpd> cc_rate_upd("cc_rate_upd");
    static hls::stream<txRateReq> txRateTable_upd_req("txRateTable_upd_req");
    #pragma HLS STREAM depth=4 variable=cc_event_to_fanout
    #pragma HLS STREAM depth=4 variable=mod_cc_init
    // mod_cc_event absorbs signal bursts at module II <= 9
    #pragma HLS STREAM depth=32 variable=mod_cc_event
    #pragma HLS STREAM depth=2 variable=mod_cc_rate_upd
    #pragma HLS STREAM depth=2 variable=cc_rate_upd
    #pragma HLS STREAM depth=2 variable=txRateTable_upd_req
    #if defined(__VITIS_HLS__)
        #pragma HLS aggregate variable=cc_event_to_fanout compact=bit
        #pragma HLS aggregate variable=mod_cc_init compact=bit
        #pragma HLS aggregate variable=mod_cc_event compact=bit
        #pragma HLS aggregate variable=mod_cc_rate_upd compact=bit
        #pragma HLS aggregate variable=cc_rate_upd compact=bit
        #pragma HLS aggregate variable=txRateTable_upd_req compact=bit
    #else
        #pragma HLS DATA_PACK variable=cc_event_to_fanout
        #pragma HLS DATA_PACK variable=mod_cc_init
        #pragma HLS DATA_PACK variable=mod_cc_event
        #pragma HLS DATA_PACK variable=mod_cc_rate_upd
        #pragma HLS DATA_PACK variable=cc_rate_upd
        #pragma HLS DATA_PACK variable=txRateTable_upd_req
    #endif

    cc_signal_table<INSTID>(
        ccSignalTable_init_req,
        ccSignalTable_cnp_req,
        ccSignalTable_nak_req,
        ccSignalTable_timeout_req,
        ccSignalTable_ack_req,
        cc_event_to_fanout,
        regCcSignalInvalidCount,
        regCcAckCount,
        regCcAckEceCount,
        regCcNakCount,
        regCcTimeoutCount,
        regCcEventDropCount,
        cycle_cnt
    );

    cc_event_fanout<INSTID>(
        cc_event_to_fanout,
        mod_cc_init,
        mod_cc_event,
        cc_module_sel,
        regCcModuleDropCount
    );

    // All compiled-in modules, each on its own slot (cc_modules.hpp)
#ifdef CC_MOD_DCQCN
    dcqcn_cc_module<INSTID>(
        mod_cc_init[CC_SLOT_DCQCN],
        mod_cc_event[CC_SLOT_DCQCN],
        mod_cc_rate_upd[CC_SLOT_DCQCN],
        cycle_cnt
    );
#endif
#ifdef CC_MOD_IDENTITY
    identity_cc_module<INSTID>(
        mod_cc_init[CC_SLOT_IDENTITY],
        mod_cc_event[CC_SLOT_IDENTITY],
        mod_cc_rate_upd[CC_SLOT_IDENTITY],
        cycle_cnt
    );
#endif

    cc_rate_mux<INSTID>(
        mod_cc_rate_upd,
        cc_rate_upd,
        cc_module_sel
    );

    cc_rate_to_gap<INSTID>(
        cc_rate_upd,
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
