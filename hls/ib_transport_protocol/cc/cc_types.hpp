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
#include <rocev2_config.hpp> // PMTU, CLOCK_PERIOD_NS, CC_LINE_RATE_MBPS

/**
 * Types shared by all congestion-control components.
 *
 *   signal sources ─ccEvent─► cc_signal_table ─ccEvent─► cc_event_fanout ─► CC module ─ccRateUpd─► cc_rate_to_gap ─txRateReq─► tx_rate_table
 *
 * Extension rule: new event types and fields are only appended, so existing modules keep working.
 */

// ------------------------------------------------------------------------------------------------
// Events: one per congestion signal
// ------------------------------------------------------------------------------------------------

enum ccEventType {
    CC_EV_INIT    = 0, // QP set up (from qp_interface). Delivered on the module's INIT input, never dropped
    CC_EV_CNP     = 1, // CNP received for the QP (from rx_ibh_fsm)
    CC_EV_ACK     = 2, // phase 2: ACK received (psn, n_acked, ece, rtt)
    CC_EV_NAK     = 3, // phase 2: NAK received (psn, aux = syndrome)
    CC_EV_TIMEOUT = 4  // phase 2: retransmission timeout (aux = retries)
    // new types are appended here
};

/**
 * One congestion signal of one QP. Narrow on purpose: it carries only the signal itself.
 * Per-QP state derived from the event history is kept by the modules that need it.
 */
struct ccEvent {
    ap_uint<16> qpn;
    ap_uint<4>  type;       // ccEventType
    ap_uint<32> ts;         // cycle_cnt when the signal was taken (set by cc_signal_table)

    // Sample of this signal, meaning depends on type
    ap_uint<24> psn;        // ACK, NAK
    ap_uint<16> n_acked;    // ACK: packets released
    ap_uint<1>  ece;        // ACK: ECN echo (BECN) set
    ap_uint<1>  rtt_valid;  // ACK
    ap_uint<32> rtt;        // ACK: cycles
    ap_uint<8>  aux;        // NAK: syndrome, TIMEOUT: retries

    ccEvent()
        :qpn(0), type(CC_EV_INIT), ts(0), psn(0), n_acked(0), ece(0), rtt_valid(0), rtt(0), aux(0) {}
    ccEvent(ccEventType type, ap_uint<16> qpn)
        :qpn(qpn), type(type), ts(0), psn(0), n_acked(0), ece(0), rtt_valid(0), rtt(0), aux(0) {}
};

// ------------------------------------------------------------------------------------------------
// Module IDs: fixed per module, so software can select modules by name (sw cDefs.hpp: CcModule)
// ------------------------------------------------------------------------------------------------

static const uint8_t CC_MOD_ID_DCQCN    = 0;
static const uint8_t CC_MOD_ID_IDENTITY = 1;
// 2-15: future internal (HLS) modules
// 16+: external modules (separate IPs behind the rocev2 cc ports, CC_EXT_MODULES)
static const uint8_t CC_MOD_ID_EXT_FIRST    = 16;
static const uint8_t CC_MOD_ID_EXT_IDENTITY = 16; // test module, hls/cc_ext/identity

// Module select register (shell CC_MODULE_SEL_REG): this value means "the build default" (CC_DEFAULT_MODULE).
// It is the reset value, so neither the shell nor software needs to know which module the build defaults to
static const uint8_t CC_MOD_SEL_DEFAULT = 0xFF;

// ID of the active module for a select register value
inline ap_uint<8> cc_active_module(ap_uint<8> sel) {
    #pragma HLS INLINE
    return (sel == CC_MOD_SEL_DEFAULT) ? ap_uint<8>(CC_DEFAULT_MODULE_ID) : sel;
}

// ------------------------------------------------------------------------------------------------
// Rates: the common output of every CC module
// ------------------------------------------------------------------------------------------------

struct ccRateUpd {
    ap_uint<16> qpn;
    ap_uint<32> rate_mbps;  // >= CC_LINE_RATE_MBPS: unlimited (gap 0). Below CC_MIN_RATE_MBPS: clamped by cc_rate_to_gap
    ccRateUpd() {}
    ccRateUpd(ap_uint<16> qpn, ap_uint<32> rate_mbps)
        :qpn(qpn), rate_mbps(rate_mbps) {}
};

/**
 * Rate update of an external module (port s_axis_cc_rate of rocev2). The module sets its own ID,
 * so the stack can forward only the active module's updates.
 */
struct ccExtRateUpd {
    ap_uint<8>  module_id;
    ap_uint<16> qpn;
    ap_uint<32> rate_mbps;
    ccExtRateUpd() {}
    ccExtRateUpd(ap_uint<8> module_id, ap_uint<16> qpn, ap_uint<32> rate_mbps)
        :module_id(module_id), qpn(qpn), rate_mbps(rate_mbps) {}
};

// Lowest rate a QP can be throttled to. Also guarantees cc_rate_to_gap never divides by 0
static const uint32_t CC_MIN_RATE_MBPS = 10;

// Wire size of a full PMTU packet: payload + 98 B (Eth+FCS+preamble+IFG 38, IPv4 20, UDP 8, BTH 12, RETH 16, ICRC 4)
static const uint32_t CC_PKT_OVERHEAD_BYTES = 98;
static const uint32_t CC_PKT_WIRE_BITS = (uint32_t(PMTU) + CC_PKT_OVERHEAD_BYTES) * 8;

// nclk frequency in MHz, rounded
static const uint32_t CC_NCLK_MHZ = uint32_t(1000.0 / CLOCK_PERIOD_NS + 0.5);

// gap = CC_PKT_WIRE_BITS * CC_NCLK_MHZ / rate_mbps (bits * MHz / Mb/s = cycles). 33552 * 250 = 8.4e6 fits 32 bits
static const uint32_t CC_GAP_NUMERATOR = CC_PKT_WIRE_BITS * CC_NCLK_MHZ;
