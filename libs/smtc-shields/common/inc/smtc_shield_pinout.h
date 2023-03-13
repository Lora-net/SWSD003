/**
 * @file      smtc_shield_pinout.h
 *
 * @brief
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2023. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SMTC_SHIELD_PINOUT_H
#define SMTC_SHIELD_PINOUT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef enum smtc_shield_pinout_e
{
    SMTC_SHIELD_PINOUT_D0,
    SMTC_SHIELD_PINOUT_D1,
    SMTC_SHIELD_PINOUT_D2,
    SMTC_SHIELD_PINOUT_D3,
    SMTC_SHIELD_PINOUT_D4,
    SMTC_SHIELD_PINOUT_D5,
    SMTC_SHIELD_PINOUT_D6,
    SMTC_SHIELD_PINOUT_D7,
    SMTC_SHIELD_PINOUT_D8,
    SMTC_SHIELD_PINOUT_D9,
    SMTC_SHIELD_PINOUT_D10,
    SMTC_SHIELD_PINOUT_D11,
    SMTC_SHIELD_PINOUT_D12,
    SMTC_SHIELD_PINOUT_D13,
    SMTC_SHIELD_PINOUT_D14,
    SMTC_SHIELD_PINOUT_D15,
    SMTC_SHIELD_PINOUT_A0,
    SMTC_SHIELD_PINOUT_A1,
    SMTC_SHIELD_PINOUT_A2,
    SMTC_SHIELD_PINOUT_A3,
    SMTC_SHIELD_PINOUT_A4,
    SMTC_SHIELD_PINOUT_A5,
    SMTC_SHIELD_PINOUT_NONE,
} smtc_shield_pinout_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // SMTC_SHIELD_PINOUT_H

/* --- EOF ------------------------------------------------------------------ */
