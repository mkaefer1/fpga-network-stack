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

template <int INSTID>
void identity_cc_module(
    hls::stream<ccSignalUpdEvent>& cc_signal_upd_event,
    hls::stream<txRateReq>& tx_rate_req
);

template <int INSTID>
void identity_cc_module(
    hls::stream<ccSignalUpdEvent>& cc_signal_upd_event,
    hls::stream<txRateReq>& tx_rate_req
) {
    #pragma HLS INLINE off
    #pragma HLS PIPELINE II=1

    ccSignalUpdEvent updEvent;

    if (!cc_signal_upd_event.empty()) {
        cc_signal_upd_event.read(updEvent);

        //Identity (usually the rate for each qpn is stored in the module but for simplicity its assumed to always be 0)
        tx_rate_req.write(txRateReq(updEvent.qpn, 0)); // Blocking
    }
}
