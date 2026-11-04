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
#include "../cc_signal_table.hpp"
#include "../tx_rate_table.hpp"

// DCQCN reaction point (Zhu et al., SIGCOMM 2015), timer-only (no byte counter, so no hyper increase).
// Times in nclk cycles (250 MHz). Alpha and rates in fixed point with 16 fraction bits; rates are fractions of line rate.

static const ap_uint<32> FIXED_POINT_ONE = 65536; // 1.0
static const unsigned G_SHIFT = 8; // g = 1/256
static const ap_uint<32> K_ALPHA_TIMER_CYCLES = 13750; // K: alpha decay period without CNP, 55 us
static const ap_uint<32> T_RATE_TIMER_CYCLES = 13750; // T: rate increase period without cut, 55 us
// N: min time between two rate cuts, 25 us. Only filters CNP bursts: the NP already paces CNPs to 50 us
// (CNP_INTERVAL_CYCLES in cnp_pacer.hpp), so N must stay well below that or jittered CNPs lose their cut
static const ap_uint<32> N_RATE_TIMER_CYCLES = 6250;
static const ap_uint<8>  F = 5; // F: fast recovery steps before additive increase

static const unsigned long long LINE_RATE_MBPS = 100000; // the only link-dependent constant
static const ap_uint<32> MAX_RATE_FRACTION = FIXED_POINT_ONE; // line rate
// 10 Mb/s floor (not in the paper), at least 1 so dcqcn_rate_to_gap never divides by 0
static const ap_uint<32> MIN_RATE_FRACTION = (10 * 65536ULL / LINE_RATE_MBPS > 0) ? 10 * 65536ULL / LINE_RATE_MBPS : 1ULL;
static const ap_uint<32> RATE_AI = 40 * 65536ULL / LINE_RATE_MBPS; // R_AI: 40 Mb/s

// Minimum cycles between two packets of a QP sending at `rate` (0 at line rate)
inline ap_uint<32> dcqcn_rate_to_gap(ap_uint<32> rate) {
    #pragma HLS INLINE
    if (rate >= MAX_RATE_FRACTION) return 0;
    // Wire time of a full PMTU packet at line rate, 16 fraction bits (cancel with the rate's):
    // 4096 B payload + 98 B (Eth+FCS+preamble+IFG 38, IPv4 20, UDP 8, BTH 12, RETH 16, ICRC 4), 83.9 cycles at 100G
    const ap_uint<32> pktCycles = 4194ULL * 8 * 250 * 65536 / LINE_RATE_MBPS;
    return pktCycles / ap_uint<17>(rate); // rate < line rate fits 17 bits: small divider
}

struct dcqcnTableEntry {
    ap_uint<1> valid;
    ap_uint<32> rc; // current rate
    ap_uint<32> rt; // target rate
    ap_uint<32> alpha;
    ap_uint<32> last_alpha_update; // compared with K
    ap_uint<32> last_rate_change; // compared with T
    ap_uint<32> last_rate_cut; // compared with N
    ap_uint<8>  inc_count; // increase events since the last cut, compared with F
    dcqcnTableEntry() {}
    dcqcnTableEntry(ap_uint<1> valid, ap_uint<32> rc, ap_uint<32> rt, ap_uint<32> alpha, ap_uint<32> last_alpha_update,
                    ap_uint<32> last_rate_change, ap_uint<32> last_rate_cut, ap_uint<8> inc_count)
        :valid(valid), rc(rc), rt(rt), alpha(alpha), last_alpha_update(last_alpha_update), last_rate_change(last_rate_change),
         last_rate_cut(last_rate_cut), inc_count(inc_count) {}
};

template <int INSTID>
void dcqcn_cc_module(
    hls::stream<ccSignalUpdEvent>& cc_signal_upd_event,
    hls::stream<ap_uint<16>>& dcqcn_init_req,
    hls::stream<txRateReq>& tx_rate_req,
    volatile ap_uint<32>& cycle_cnt
);

template <int INSTID = 0>
void dcqcn_cc_module(
    hls::stream<ccSignalUpdEvent>& cc_signal_upd_event, // in: CNP per QP (from cc_signal_table)
    hls::stream<ap_uint<16>>& dcqcn_init_req, // in: QP setup (from qp_interface)
    hls::stream<txRateReq>& tx_rate_req, // out: new gap per QP (to tx_rate_table)
    volatile ap_uint<32>& cycle_cnt // in: free-running cycle counter (nclk)
) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=1

    static dcqcnTableEntry dcqcn_table[MAX_QPS];
    #if defined(__VITIS_HLS__)
        #pragma HLS bind_storage variable=dcqcn_table type=RAM_2P impl=BRAM
    #else
        #pragma HLS RESOURCE variable=dcqcn_table core=RAM_2P_BRAM
    #endif

    static ap_uint<16> scanQpn = 0; // next QP for the timer scan

    ap_uint<16> initReq;
    ccSignalUpdEvent updEvent;
    dcqcnTableEntry entry;

    // Read once: every access to the volatile port is a separate read.
    // Elapsed times below are wrap-safe because they are assigned to ap_uint<32> (a raw difference is a signed ap_int<33>)
    ap_uint<32> now = cycle_cnt;

    if (!dcqcn_init_req.empty()) {
        dcqcn_init_req.read(initReq);
        if (initReq >= MAX_QPS) return;

        // Line rate, alpha = 1. last_rate_cut lies N in the past so the first CNP cuts at once.
        // No rate update: tx_rate_table inits the QP to gap 0 itself
        dcqcn_table[initReq] = dcqcnTableEntry(1, MAX_RATE_FRACTION, MAX_RATE_FRACTION, FIXED_POINT_ONE, now, now, now - N_RATE_TIMER_CYCLES, 0);

    } else if (!cc_signal_upd_event.empty()) {
        // CNP: cut the rate (at most once per N), raise alpha (every CNP)
        cc_signal_upd_event.read(updEvent);
        if (updEvent.qpn >= MAX_QPS) return;

        entry = dcqcn_table[updEvent.qpn];
        if (!entry.valid) return;

        ap_uint<32> sinceCut = now - entry.last_rate_cut;
        if (sinceCut >= N_RATE_TIMER_CYCLES) {
            // rt = rc, rc = rc * (1 - alpha/2) with the old alpha
            entry.rt = entry.rc;
            ap_uint<64> cut = entry.rc * (FIXED_POINT_ONE - (entry.alpha >> 1));
            entry.rc = ap_uint<32>(cut >> 16);
            if (entry.rc < MIN_RATE_FRACTION) entry.rc = MIN_RATE_FRACTION;

            // Back to fast recovery, T restarts
            entry.inc_count = 0;
            entry.last_rate_change = now;
            entry.last_rate_cut = now;

            tx_rate_req.write(txRateReq(updEvent.qpn, dcqcn_rate_to_gap(entry.rc)));
        }

        // alpha = (1 - g) * alpha + g, K restarts
        entry.alpha = entry.alpha - (entry.alpha >> G_SHIFT) + (FIXED_POINT_ONE >> G_SHIFT);
        entry.last_alpha_update = now;

        dcqcn_table[updEvent.qpn] = entry;

    } else {
        // Idle: check one QP per cycle. A pass over 500 QPs takes 2 us, well below K and T
        ap_uint<16> qpn = scanQpn;
        scanQpn = (scanQpn == MAX_QPS - 1) ? ap_uint<16>(0) : ap_uint<16>(scanQpn + 1);

        entry = dcqcn_table[qpn];
        if (!entry.valid) return;

        bool changed = false;

        // K without CNP: alpha = (1 - g) * alpha
        ap_uint<32> sinceAlpha = now - entry.last_alpha_update;
        if (sinceAlpha >= K_ALPHA_TIMER_CYCLES) {
            entry.alpha = entry.alpha - (entry.alpha >> G_SHIFT);
            entry.last_alpha_update = now;
            changed = true;
        }

        // T without cut: increase, unless already at line rate
        ap_uint<32> sinceChange = now - entry.last_rate_change;
        if (sinceChange >= T_RATE_TIMER_CYCLES && entry.rc < MAX_RATE_FRACTION) {
            if (entry.inc_count >= F) { // after F fast recovery steps, additive increase: rt += R_AI
                entry.rt = (entry.rt + RATE_AI > MAX_RATE_FRACTION) ? MAX_RATE_FRACTION : ap_uint<32>(entry.rt + RATE_AI);
            } else {
                entry.inc_count++;
            }
            // Every stage: rc halfway to rt. Rounds up so rc reaches rt instead of stopping 1 below
            entry.rc = (entry.rt + entry.rc + 1) >> 1;
            entry.last_rate_change = now;
            changed = true;

            tx_rate_req.write(txRateReq(qpn, dcqcn_rate_to_gap(entry.rc)));
        }

        if (changed) dcqcn_table[qpn] = entry;
    }
}
