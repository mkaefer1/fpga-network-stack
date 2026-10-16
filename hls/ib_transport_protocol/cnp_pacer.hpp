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
    hls::stream<cnpEvent>& cnpGen2metaMerger // out: "send a CNP for QP x now"
) {
    #pragma HLS INLINE off
    #pragma HLS pipeline II=1

    cnpGenReq req;
    ap_uint<16> initQpn;

    // Init is rare, so it goes first and cannot be starved by CNP requests.
    // Must always be consumed, otherwise qp_interface stalls on its blocking write.
    if (!qpi2cnpGen_init.empty()) {
        qpi2cnpGen_init.read(initQpn);
        // No-op for now: reset the per-QP rate-limit state here
    }
    else if (!rx2cnpGen_req.empty()) {
        rx2cnpGen_req.read(req);
        cnpGen2metaMerger.write(cnpEvent(req.qpn));
    }
}
