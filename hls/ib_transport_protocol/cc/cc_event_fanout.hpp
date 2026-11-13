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
 * Hands a copy of every event from cc_signal_table to every compiled-in CC module.
 * All modules run all the time, so a module switched to at runtime already has warm per-QP state.
 *
 * Each module owns two input FIFOs:
 *   - cc_init:  INIT events only, written blocking. A lost INIT would leave the module without the QP.
 *               INITs only happen at QP setup, so this stall never touches the datapath.
 *   - cc_event: all other signals in arrival order, written non-blocking. If a module falls behind,
 *               the signal is dropped for that module only and never back-pressures the stack or the
 *               other modules.
 *
 * Ordering: cc_signal_table emits a QP's INIT before any of its signals, and modules drain cc_init
 * first, so every module processes a QP's INIT before its first signal.
 */
template <int INSTID = 0>
void cc_event_fanout(
    hls::stream<ccEvent>& cc_event_in,                       // in:  events from cc_signal_table
    hls::stream<ccEvent> mod_cc_init[CC_N_INT_MODULES],      // out: per-module INIT FIFO (blocking)
    hls::stream<ccEvent> mod_cc_event[CC_N_INT_MODULES],     // out: per-module signal FIFO (non-blocking)
    volatile ap_uint<8>& cc_module_sel,                      // in:  module select register (CC_MOD_SEL_DEFAULT = build default)
    ap_uint<32>& regCcModuleDropCount                        // out: signals dropped for the ACTIVE module (its FIFO was full)
) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    static ap_uint<32> moduleDropCount = 0;
    ccEvent ev;

    // Read once: every access to the volatile port is a separate read
    ap_uint<8> active = cc_active_module(cc_module_sel);

    if (!cc_event_in.empty()) {
        cc_event_in.read(ev);
        if (ev.type == CC_EV_INIT) {
            for (int i = 0; i < CC_N_INT_MODULES; i++) {
                #pragma HLS UNROLL
                mod_cc_init[i].write(ev); // Blocking: INIT must never be lost
            }
        } else {
            bool activeDropped = false;
            for (int i = 0; i < CC_N_INT_MODULES; i++) {
                #pragma HLS UNROLL
                if (!mod_cc_event[i].full()) {
                    mod_cc_event[i].write(ev);
                } else if (CC_SLOT_ID[i] == active) {
                    activeDropped = true;
                }
                // Drops of inactive (shadow) modules are not counted
            }
            if (activeDropped) {
                moduleDropCount++;
                regCcModuleDropCount = moduleDropCount;
            }
        }
    }
}
