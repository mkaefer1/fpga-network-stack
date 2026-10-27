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

#include "../axi_utils.hpp"
#include "ib_transport_protocol.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS

// Minimum time between two CNPs of the same QP: 50 us in nclk cycles (250 MHz, 4 ns per cycle)
static const ap_uint<32> CNP_INTERVAL_CYCLES = 12500;

struct cnpPacerTableEntry {
    ap_uint<1> valid;
    ap_uint<1> ever_sent; // a CNP has been sent on this QP since init, so latest_cnp_tx_timestamp is meaningful
    ap_uint<32> latest_cnp_tx_timestamp;
}

struct cnpGenReq {
    ap_uint<16> qpn;
    cnpGenReq() {}
    cnpGenReq(ap_uint<16> qpn)
        :qpn(qpn) {}
};

struct cnpEvent {
    ap_uint<16> qpn;
    cnpEvent() {}
    cnpEvent(ap_uint<16> qpn)
        :qpn(qpn) {}
};

template <int INSTID = 0>
void cnp_pacer(
    hls::stream<cnpGenReq>& rx2cnpGen_req, // in:  "CE seen on QP x" from rx_exh_fsm
    hls::stream<ap_uint<16>>& qpi2cnpGen_init, // in:  QP setup, reset rate-limit state
    hls::stream<cnpEvent>& cnpGen2metaMerger, // out: "send a CNP for QP x now"
    volatile ap_uint<32>& cycle_cnt // in:  free-running cycle counter (nclk)
) {
    #pragma HLS INLINE off
    #pragma HLS pipeline II=1

    static cnpPacerTableEntry cnp_tx_pacer_table[MAX_QPS];

    #if defined(__VITIS_HLS__)
        #pragma HLS bind_storage variable=cc_signal_table type=RAM_2P impl=BRAM
    #else
        #pragma HLS RESOURCE variable=cc_signal_table core=RAM_2P_BRAM
    #endif

    cnpGenReq req;
    ap_uint<16> init;

    if (!qpi2cnpGen_init.empty()) {
        qpi2cnpGen_init.read(init);
        if (initReq >= MAX_QPS) return;

        // Init the entry for the specific QP
        cnpPacerTableEntry entry;
        entry.valid = 1;
        entry.ever_sent = 0;
        entry.latest_cnp_tx_timestamp = 0;
        cnp_tx_pacer_table[init] = entry;
    }
    else if (!rx2cnpGen_req.empty()) {
        rx2cnpGen_req.read(req);
        if (req.qpn >= MAX_QPS) return;

        // Read the counter once: every access to the volatile port is a separate read
        ap_uint<32> now = cycle_cnt;

        cnpPacerTableEntry entry = cnp_tx_pacer_table[req.qpn];
        if (!entry.valid) return;

        // Cycles since the last CNP of this QP. cycle_cnt wraps every 2^32 cycles (~17.2 s at 250 MHz), but the
        // difference modulo 2^32 is still the real elapsed time as long as that is below 2^32 cycles.
        // It must be assigned to a 32-bit variable: ap_uint<32> - ap_uint<32> is a signed ap_int<33>, which would
        // be negative after a wrap if compared directly. After more than ~17.2 s without a CNP, one request may
        // be wrongly suppressed (probability CNP_INTERVAL_CYCLES / 2^32); the next one after the interval passes.
        ap_uint<32> elapsed = now - entry.latest_cnp_tx_timestamp;
        if (!entry.ever_sent || elapsed >= CNP_INTERVAL_CYCLES) {
            cnpGen2metaMerger.write(cnpEvent(req.qpn));
            entry.ever_sent = 1;
            entry.latest_cnp_tx_timestamp = now;
            cnp_tx_pacer_table[req.qpn] = entry;
        }
    }
}
