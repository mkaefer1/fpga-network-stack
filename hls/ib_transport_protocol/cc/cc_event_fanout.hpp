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

/**
 * Hands every event from cc_signal_table to the CC module(s). Each module owns two input FIFOs:
 *   - cc_init:  INIT events only, written blocking. A lost INIT would leave the module without the QP.
 *               INITs only happen at QP setup, so this stall never touches the datapath.
 *   - cc_event: all other signals in arrival order, written non-blocking. If the module falls behind,
 *               the signal is dropped for that module only and never back-pressures the stack.
 *
 * Ordering: cc_signal_table emits a QP's INIT before any of its signals, and modules drain cc_init
 * first, so a module always processes a QP's INIT before its first signal.
 *
 * Phase 1: one module (selected at build time). Phase 3 adds one FIFO pair per compiled-in module
 * plus the external port; every destination then gets its own copy of each event.
 */
template <int INSTID = 0>
void cc_event_fanout(
    hls::stream<ccEvent>& cc_event_in,   // in:  events from cc_signal_table
    hls::stream<ccEvent>& mod_cc_init,   // out: module INIT FIFO (blocking)
    hls::stream<ccEvent>& mod_cc_event   // out: module signal FIFO (non-blocking)
) {
    #pragma HLS PIPELINE II=1
    #pragma HLS INLINE off

    ccEvent ev;

    if (!cc_event_in.empty()) {
        cc_event_in.read(ev);
        if (ev.type == CC_EV_INIT) {
            mod_cc_init.write(ev); // Blocking: INIT must never be lost
        } else if (!mod_cc_event.full()) {
            mod_cc_event.write(ev);
        }
        // else: dropped for this module (per-module drop counter exported in phase 3)
    }
}
