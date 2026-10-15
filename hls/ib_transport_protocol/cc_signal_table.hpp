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

#include "../axi_utils.hpp"
#include "../ib_transport_protocol/ib_transport_protocol.hpp"
#include <rocev2_config.hpp> //defines MAX_QPS

struct ccSignalTableEntry {
    ap_uint<1> valid; // Should be set by init

    ap_uint<32> cnp_count;
    ap_uint<32> latest_cnp_timestamp;
};

struct ccSignalReq {
    ap_uint<16> qpn;
    ap_uint<32> new_cnp_count; // How many cnp counts should be added
    ap_uint<32> new_latest_cnp_timestamp;
    ccSignalReq() {}
    ccSignalReq(ap_uint<16> qpn, ap_uint<32> new_cnp_count, ap_uint<32> new_latest_cnp_timestamp)
        :qpn(qpn), new_cnp_count(new_cnp_count), new_latest_cnp_timestamp(new_latest_cnp_timestamp) {}
};

struct ccSignalOutput {
    ap_uint<16> qpn;
    ap_uint<32> cnp_count;
    ap_uint<32> latest_cnp_timestamp;
    ccSignalOutput() {}
    ccSignalOutput(ap_uint<16> qpn, ap_uint<32> cnp_count, ap_uint<32> latest_cnp_timestamp)
        :qpn(qpn), cnp_count(cnp_count), latest_cnp_timestamp(latest_cnp_timestamp) {}
};

template <int INSTID>
void cc_signal_table(
    hls::stream<ccSignalReq>& ccSignalTable_upd_req,
    hls::stream<ap_uint<16>>& ccSignalTable_init_req,
    hls::stream<ccSignalOutput> & ccSignalTable_output
);

template <int INSTID = 0>
void cc_signal_table(
    hls::stream<ccSignalReq>& ccSignalTable_upd_req,
    hls::stream<ap_uint<16>>& ccSignalTable_init_req,
    hls::stream<ccSignalOutput> & ccSignalTable_output
) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    // The actual array that stores the entries
    static ccSignalTableEntry cc_signal_table[MAX_QPS];

    #if defined(__VITIS_HLS__)
        #pragma HLS bind_storage variable=cc_signal_table type=RAM_2P impl=BRAM
    #else
        #pragma HLS RESOURCE variable=cc_signal_table core=RAM_2P_BRAM
    #endif

    ccSignalReq ccReq;
    ap_uint<16> initReq;

    if (!ccSignalTable_init_req.empty()) {
        ccSignalTable_init_req.read(initReq);
        if (initReq >= MAX_QPS) return;

        // Init the entry for the specific QP
        ccSignalTableEntry entry;
        entry.valid = 1;
        entry.cnp_count = 0;
        entry.latest_cnp_timestamp = 0; //MAXI_TODO: Think this over when its clear how timestamps are handled
        cc_signal_table[initReq] = entry;
    }
    else if (!ccSignalTable_upd_req.empty()) {
        ccSignalTable_upd_req.read(ccReq);
        if (ccReq.qpn >= MAX_QPS) return;

        // Read => Modify => Write
        ccSignalTableEntry entry = cc_signal_table[ccReq.qpn];
        if (!entry.valid) return; // Drop updates for uninitialized entries
        entry.cnp_count += ccReq.new_cnp_count;
        entry.latest_cnp_timestamp = ccReq.new_latest_cnp_timestamp;
        cc_signal_table[ccReq.qpn] = entry;

        // Only output when queue is not full => otherwise this would stall the RX
        if (!ccSignalTable_output.full())
            ccSignalTable_output.write(ccSignalOutput(ccReq.qpn, entry.cnp_count, entry.latest_cnp_timestamp));
    }
}