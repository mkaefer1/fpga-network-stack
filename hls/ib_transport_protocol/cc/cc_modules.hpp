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

#include "cc_types.hpp"
#include <rocev2_config.hpp> // CC_MOD_<NAME>
#include "cc_modules/identity.hpp"
#include "cc_modules/dcqcn.hpp"

/**
 * Internal (HLS) CC modules compiled into this build (CC_MODULES in rocev2/CMakeLists.txt).
 *
 * Every compiled-in module gets a slot: a dense index 0..CC_N_INT_MODULES-1 into the per-module FIFO
 * arrays of cc_unit. The slot is a build detail; software only ever sees the fixed module ID (CC_SLOT_ID).
 *
 * Adding an internal module: give it an ID in cc_types.hpp, add it to CC_MODULES_SUPPORTED in
 * rocev2/CMakeLists.txt, and add it to the slot enum, CC_SLOT_ID and cc_unit below.
 */
enum ccModuleSlot {
#ifdef CC_MOD_DCQCN
    CC_SLOT_DCQCN,
#endif
#ifdef CC_MOD_IDENTITY
    CC_SLOT_IDENTITY,
#endif
    CC_N_INT_MODULES
};

static_assert(CC_N_INT_MODULES >= 1, "At least one internal CC module must be compiled in (CC_MODULES)");

// Module ID of each slot
static const uint8_t CC_SLOT_ID[CC_N_INT_MODULES] = {
#ifdef CC_MOD_DCQCN
    CC_MOD_ID_DCQCN,
#endif
#ifdef CC_MOD_IDENTITY
    CC_MOD_ID_IDENTITY,
#endif
};
