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
#include "cc_modules.hpp"

/**
 * Forwards the rate updates of the active CC module to cc_rate_to_gap and drops those of all others.
 *
 * Every module output is drained every cycle (modules write their rates blocking), so inactive modules
 * keep running as shadows. Their rates are dropped here (logging them is phase 6).
 *
 * Runtime switch: the select register only changes which module's FUTURE updates are forwarded.
 * tx_rate_table keeps the gaps of the previous module until the new one sends updates (the hand-over
 * is out of scope, see cc-architecture-plan.md section 5.2).
 */
template <int INSTID = 0>
void cc_rate_mux(
    hls::stream<ccRateUpd> mod_cc_rate_upd[CC_N_INT_MODULES], // in:  rate updates of each internal module
    hls::stream<ccExtRateUpd>& ext_cc_rate_upd,                // in:  rate updates of the external modules (tagged with their ID)
    hls::stream<ccRateUpd>& cc_rate_upd,                       // out: rate updates of the active module
    volatile ap_uint<8>& cc_module_sel                         // in:  module select register (CC_MOD_SEL_DEFAULT = build default)
) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    ccRateUpd upd;
    ccExtRateUpd extUpd;
    bool forwarded = false;

    // Read once: every access to the volatile port is a separate read
    ap_uint<8> active = cc_active_module(cc_module_sel);

    for (int i = 0; i < CC_N_INT_MODULES; i++) {
        #pragma HLS UNROLL
        if (!mod_cc_rate_upd[i].empty()) {
            mod_cc_rate_upd[i].read(upd);
            if (CC_SLOT_ID[i] == active) {
                cc_rate_upd.write(upd); // at most one module is active, so at most one write per cycle
                forwarded = true;
            }
        }
    }

    // External modules: drained like the internal ones, forwarded if the update comes from the active module
    if (!forwarded && !ext_cc_rate_upd.empty()) {
        ext_cc_rate_upd.read(extUpd);
        if (extUpd.module_id == active) {
            cc_rate_upd.write(ccRateUpd(extUpd.qpn, extUpd.rate_mbps));
        }
    }
}
