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
#include "../ib_transport_protocol.hpp"
#include "cc_types.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS

/**
 * Per-QP statistics. Not part of the events: modules keep their own state,
 * these counters are only for statistics and (later) per-QP readback by software.
 */
struct ccSignalTableEntry {
    ap_uint<1>  valid;          // set by INIT, signals for invalid QPs are dropped
    ap_uint<32> cnp_count;
    ap_uint<32> last_cnp_ts;    // cycle_cnt of the last CNP
};

template <int INSTID>
void cc_signal_table(
    hls::stream<ap_uint<16>>& ccSignalTable_init_req,
    hls::stream<ccEvent>& ccSignalTable_cnp_req,
    hls::stream<ccEvent>& cc_event_out,
    ap_uint<32>& regCcSignalInvalidCount,
    volatile ap_uint<32>& cycle_cnt
);

/**
 * Collects the congestion signals of all sources, one input stream per source.
 *
 * For every valid signal it emits one ccEvent (narrow: the signal only, timestamped here).
 *   - INIT:  written to cc_event_out blocking. A lost INIT would leave a module without the QP,
 *            and INITs only happen at QP setup, so a stall here never touches the datapath.
 *   - other: written non-blocking. The CC path may lose signals but must never stall the RX path.
 *
 * Input priority: INIT > CNP (> NAK/TIMEOUT > ACK, phase 2). Rare signals are never starved.
 */
template <int INSTID = 0>
void cc_signal_table(
    hls::stream<ap_uint<16>>& ccSignalTable_init_req,  // in:  QP setup (qp_interface, written blocking)
    hls::stream<ccEvent>& ccSignalTable_cnp_req,       // in:  CNPs (rx_ibh_fsm, written non-blocking)
    hls::stream<ccEvent>& cc_event_out,                // out: events to cc_event_fanout
    ap_uint<32>& regCcSignalInvalidCount,              // out: signals dropped, QP out of range or not initialized
    volatile ap_uint<32>& cycle_cnt                    // in:  free-running cycle counter (nclk)
) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    // The actual array that stores the entries
    static ccSignalTableEntry cc_signal_table[MAX_QPS];
    static ap_uint<32> invalidCount = 0;

    #if defined(__VITIS_HLS__)
        #pragma HLS bind_storage variable=cc_signal_table type=RAM_2P impl=BRAM
    #else
        #pragma HLS RESOURCE variable=cc_signal_table core=RAM_2P_BRAM
    #endif

    ap_uint<16> initReq;
    ccEvent ev;

    // Read once: every access to the volatile port is a separate read
    ap_uint<32> now = cycle_cnt;

    if (!ccSignalTable_init_req.empty()) {
        ccSignalTable_init_req.read(initReq);
        if (initReq >= MAX_QPS) return;

        // Init the entry for the specific QP
        ccSignalTableEntry entry;
        entry.valid = 1;
        entry.cnp_count = 0;
        entry.last_cnp_ts = 0;
        cc_signal_table[initReq] = entry;

        ccEvent initEv(CC_EV_INIT, initReq);
        initEv.ts = now;
        cc_event_out.write(initEv); // Blocking: INIT must never be lost
    }
    else if (!ccSignalTable_cnp_req.empty()) {
        ccSignalTable_cnp_req.read(ev);
        if (ev.qpn >= MAX_QPS) {
            invalidCount++;
            regCcSignalInvalidCount = invalidCount;
            return;
        }

        // Read => Modify => Write
        ccSignalTableEntry entry = cc_signal_table[ev.qpn];
        if (!entry.valid) { // Drop signals for uninitialized QPs (this also orders a QP's INIT before its signals)
            invalidCount++;
            regCcSignalInvalidCount = invalidCount;
            return;
        }
        entry.cnp_count++;
        entry.last_cnp_ts = now;
        cc_signal_table[ev.qpn] = entry;

        ev.type = CC_EV_CNP;
        ev.ts = now;
        // Non-blocking: when the fanout input is full the signal is dropped instead of stalling the RX path
        // (drop counter exported with the other per-signal counters in phase 2)
        if (!cc_event_out.full())
            cc_event_out.write(ev);
    }
}
