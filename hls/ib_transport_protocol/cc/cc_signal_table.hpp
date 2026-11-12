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
    ap_uint<32> ack_count;
    ap_uint<32> ece_count;      // ACKs with ECN echo (BECN)
    ap_uint<32> nak_count;
    ap_uint<32> timeout_count;
};

template <int INSTID>
void cc_signal_table(
    hls::stream<ap_uint<16>>& ccSignalTable_init_req,
    hls::stream<ccEvent>& ccSignalTable_cnp_req,
    hls::stream<ccEvent>& ccSignalTable_nak_req,
    hls::stream<ccEvent>& ccSignalTable_timeout_req,
    hls::stream<ccEvent>& ccSignalTable_ack_req,
    hls::stream<ccEvent>& cc_event_out,
    ap_uint<32>& regCcSignalInvalidCount,
    ap_uint<32>& regCcAckCount,
    ap_uint<32>& regCcAckEceCount,
    ap_uint<32>& regCcNakCount,
    ap_uint<32>& regCcTimeoutCount,
    ap_uint<32>& regCcEventDropCount,
    volatile ap_uint<32>& cycle_cnt
);

/**
 * Collects the congestion signals of all sources, one input stream per source (one writer per stream).
 *
 * For every valid signal it emits one ccEvent (narrow: the signal only, timestamped here).
 *   - INIT:  written to cc_event_out blocking. A lost INIT would leave a module without the QP,
 *            and INITs only happen at QP setup, so a stall here never touches the datapath.
 *   - other: written non-blocking. The CC path may lose signals but must never stall the datapath.
 *
 * Input priority: INIT > CNP > NAK > TIMEOUT > ACK. Rare signals are never starved by the per-ACK stream.
 * Adding a signal source = one more input stream and one more branch here; modules are not affected.
 */
template <int INSTID = 0>
void cc_signal_table(
    hls::stream<ap_uint<16>>& ccSignalTable_init_req,     // in:  QP setup (qp_interface, written blocking)
    hls::stream<ccEvent>& ccSignalTable_cnp_req,          // in:  CNPs (rx_ibh_fsm)
    hls::stream<ccEvent>& ccSignalTable_nak_req,          // in:  NAKs (rx_exh_fsm)
    hls::stream<ccEvent>& ccSignalTable_timeout_req,      // in:  retransmission timeouts (transport_timer)
    hls::stream<ccEvent>& ccSignalTable_ack_req,          // in:  ACKs with RTT and ECN echo (retransmitter, or rx_ibh_fsm without RETRANS_EN)
    hls::stream<ccEvent>& cc_event_out,                   // out: events to cc_event_fanout
    ap_uint<32>& regCcSignalInvalidCount,                 // out: signals dropped, QP out of range or not initialized
    ap_uint<32>& regCcAckCount,                           // out: ACK events
    ap_uint<32>& regCcAckEceCount,                        // out: ACK events with ECN echo
    ap_uint<32>& regCcNakCount,                           // out: NAK events
    ap_uint<32>& regCcTimeoutCount,                       // out: TIMEOUT events
    ap_uint<32>& regCcEventDropCount,                     // out: valid signals dropped because the fanout input was full
    volatile ap_uint<32>& cycle_cnt                       // in:  free-running cycle counter (nclk)
) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    // The actual array that stores the entries
    static ccSignalTableEntry cc_signal_table[MAX_QPS];
    static ap_uint<32> invalidCount = 0;
    static ap_uint<32> ackCount = 0;
    static ap_uint<32> ackEceCount = 0;
    static ap_uint<32> nakCount = 0;
    static ap_uint<32> timeoutCount = 0;
    static ap_uint<32> eventDropCount = 0;

    #if defined(__VITIS_HLS__)
        #pragma HLS bind_storage variable=cc_signal_table type=RAM_2P impl=BRAM
    #else
        #pragma HLS RESOURCE variable=cc_signal_table core=RAM_2P_BRAM
    #endif

    ap_uint<16> initReq;
    ccEvent ev;
    bool haveSignal = false;

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
        entry.ack_count = 0;
        entry.ece_count = 0;
        entry.nak_count = 0;
        entry.timeout_count = 0;
        cc_signal_table[initReq] = entry;

        ccEvent initEv(CC_EV_INIT, initReq);
        initEv.ts = now;
        cc_event_out.write(initEv); // Blocking: INIT must never be lost
        return;
    }

    // One signal per cycle, by priority. The source sets the type, this table only checks and timestamps it
    if (!ccSignalTable_cnp_req.empty()) {
        ccSignalTable_cnp_req.read(ev);
        haveSignal = true;
    } else if (!ccSignalTable_nak_req.empty()) {
        ccSignalTable_nak_req.read(ev);
        haveSignal = true;
    } else if (!ccSignalTable_timeout_req.empty()) {
        ccSignalTable_timeout_req.read(ev);
        haveSignal = true;
    } else if (!ccSignalTable_ack_req.empty()) {
        ccSignalTable_ack_req.read(ev);
        haveSignal = true;
    }
    if (!haveSignal) return;

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

    switch (unsigned(ev.type)) {
    case CC_EV_CNP:
        entry.cnp_count++;
        entry.last_cnp_ts = now;
        break;
    case CC_EV_ACK:
        entry.ack_count++;
        ackCount++;
        regCcAckCount = ackCount;
        if (ev.ece) {
            entry.ece_count++;
            ackEceCount++;
            regCcAckEceCount = ackEceCount;
        }
        break;
    case CC_EV_NAK:
        entry.nak_count++;
        nakCount++;
        regCcNakCount = nakCount;
        break;
    case CC_EV_TIMEOUT:
        entry.timeout_count++;
        timeoutCount++;
        regCcTimeoutCount = timeoutCount;
        break;
    default:
        break;
    }
    cc_signal_table[ev.qpn] = entry;

    ev.ts = now;
    // Non-blocking: when the fanout input is full the signal is dropped instead of stalling the datapath
    if (!cc_event_out.full()) {
        cc_event_out.write(ev);
    } else {
        eventDropCount++;
        regCcEventDropCount = eventDropCount;
    }
}
