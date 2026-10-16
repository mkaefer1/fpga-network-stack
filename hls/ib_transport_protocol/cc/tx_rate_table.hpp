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

#include "../../axi_utils.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS

struct txRateEntry {
    ap_uint<32> gap;            // minimum cycles between two packets of this QP (0 = unlimited)
    // ap_uint<32> last_send_ts;   // cycle the last packet of this QP was let through (owned by this table)
};

struct txRateReq {
    ap_uint<16> qpn;
    ap_uint<32> gap;
    txRateReq() {}
    txRateReq(ap_uint<16> qpn, ap_uint<32> gap)
        :qpn(qpn), gap(gap) {}
};

template <int INSTID>
void tx_rate_table(
    hls::stream<txRateReq>& txRateTable_upd_req,
    hls::stream<ap_uint<16>>& txRateTable_init_req,
    hls::stream<ap_uint<16>>& txRateTable_lrh_qpn_req,
    hls::stream<bool>& txRateTable_lrh_rsp,
    hls::stream<ap_uint<16>>& txRateTable_hrr_qpn_req,
    hls::stream<bool>& txRateTable_hrr_rsp
);

/**
 * Per-QP TX rate table.
 *
 * Answers "may QP x send a packet now?" for two clients:
 *   local_req_handler     (txRateTable_lrh_qpn_req -> txRateTable_lrh_rsp)
 *   handle_read_requests  (txRateTable_hrr_qpn_req -> txRateTable_hrr_rsp)
 * For now the decision is based on the gap only (send-time tracking is commented out).
 */
template <int INSTID>
void tx_rate_table(
    hls::stream<txRateReq>& txRateTable_upd_req,
    hls::stream<ap_uint<16>>& txRateTable_init_req,
    hls::stream<ap_uint<16>>& txRateTable_lrh_qpn_req,
    hls::stream<bool>& txRateTable_lrh_rsp,
    hls::stream<ap_uint<16>>& txRateTable_hrr_qpn_req,
    hls::stream<bool>& txRateTable_hrr_rsp
) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=1

    static txRateEntry tx_rate_table[MAX_QPS];
    #if defined(__VITIS_HLS__)
        #pragma HLS bind_storage variable=tx_rate_table type=RAM_2P impl=BRAM
    #else
        #pragma HLS RESOURCE variable=tx_rate_table core=RAM_2P_BRAM
    #endif

    // MAXI_TODO: time source. Counts calls of this process, i.e. cycles only while it never stalls.
    // Only relevant for gap > 0 (the identity module always sets gap 0).
    // static ap_uint<32> now = 0;
    // now++;

    txRateReq upd_req;
    ap_uint<16> init_req;
    ap_uint<16> qpn_req;
    bool fromHrr;
    // Alternate between the two clients when both ask, so a client that keeps getting
    // denied (and keeps asking) cannot starve the other one
    static bool lastWasHrr = true;

    // Init is rare, so it goes first and cannot be starved by updates or lookups
    if (!txRateTable_init_req.empty()) {
        txRateTable_init_req.read(init_req);
        if (init_req >= MAX_QPS) return;

        txRateEntry entry;
        entry.gap = 0;
        // entry.last_send_ts = 0;
        tx_rate_table[init_req] = entry;

    } else if (!txRateTable_upd_req.empty()) {
        txRateTable_upd_req.read(upd_req);
        if (upd_req.qpn >= MAX_QPS) return;
        tx_rate_table[upd_req.qpn].gap = upd_req.gap;

    } else if (!txRateTable_lrh_qpn_req.empty() || !txRateTable_hrr_qpn_req.empty()) {
        fromHrr = txRateTable_lrh_qpn_req.empty() || (!txRateTable_hrr_qpn_req.empty() && !lastWasHrr);
        lastWasHrr = fromHrr;
        if (fromHrr)
            txRateTable_hrr_qpn_req.read(qpn_req);
        else
            txRateTable_lrh_qpn_req.read(qpn_req);

        bool allowed;
        if (qpn_req >= MAX_QPS) {
            // Unknown QP: never block it, and always answer (the requester waits for a response)
            allowed = true;
        } else {
            txRateEntry entry = tx_rate_table[qpn_req];
            // Gap only: gap > 0 means "denied" (the requester keeps asking) until send-time tracking is enabled
            allowed = (entry.gap == 0);

            // Read => Modify => Write
            // ap_uint<32> elapsed = now - entry.last_send_ts; // wrap-safe difference
            // bool allowed = (entry.gap == 0) || (elapsed >= entry.gap);
            // if (allowed) {
            //     entry.last_send_ts = now;
            //     tx_rate_table[qpn_req] = entry;
            // }
        }

        if (fromHrr)
            txRateTable_hrr_rsp.write(allowed);
        else
            txRateTable_lrh_rsp.write(allowed);
    }
}
