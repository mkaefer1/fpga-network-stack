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
#pragma once

#include "../../axi_utils.hpp"
#include "cc_types.hpp"
#include "tx_rate_table.hpp"

/**
 * Converts the rate a CC module asks for into the inter-packet gap tx_rate_table enforces.
 *
 *   gap = CC_PKT_WIRE_BITS * CC_NCLK_MHZ / rate_mbps   (bits * MHz / Mb/s = cycles)
 *
 *   - rate_mbps >= CC_LINE_RATE_MBPS: gap 0 (unlimited)
 *   - rate_mbps <  CC_MIN_RATE_MBPS:  clamped to CC_MIN_RATE_MBPS (also rules out a division by 0,
 *                                     so a misbehaving module cannot stop a QP completely)
 *
 * Only this stage knows the packet size, line rate and clock frequency. The divider sits in its
 * own pipelined process so tx_rate_table keeps II = 1.
 */
template <int INSTID = 0>
void cc_rate_to_gap(
    hls::stream<ccRateUpd>& cc_rate_upd,  // in:  rate per QP from the CC module
    hls::stream<txRateReq>& tx_rate_req   // out: gap per QP to tx_rate_table
) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    ccRateUpd upd;

    if (!cc_rate_upd.empty()) {
        cc_rate_upd.read(upd);

        ap_uint<32> gap;
        if (upd.rate_mbps >= CC_LINE_RATE_MBPS) {
            gap = 0;
        } else {
            ap_uint<32> rate = (upd.rate_mbps < CC_MIN_RATE_MBPS) ? ap_uint<32>(CC_MIN_RATE_MBPS) : upd.rate_mbps;
            gap = ap_uint<32>(CC_GAP_NUMERATOR) / rate;
        }
        tx_rate_req.write(txRateReq(upd.qpn, gap));
    }
}
