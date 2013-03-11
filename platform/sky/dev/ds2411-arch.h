/*
 * Copyright (c) 2013, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         Architecture-specific definitions for the DS2411 on Tmote Sky.
 * \author
 *         Niclas Finne <nfi@sics.se>
 */

#ifndef __DS2411_ARCH_H__
#define __DS2411_ARCH_H__

#include "contiki-conf.h"

/*
 * One wire DS2411 configuration for the TMote Sky.
 */

/* DS2411 1-wire is at P2.4 */
#define DS2411_PORT(type) P2##type
#define DS2411_PIN        4

/* 00:12:75    Moteiv    # Moteiv Corporation */
#define DS2411_OUI        { 0x00, 0x12, 0x75 }

#define DS2411_INIT() do {                                               \
    DS2411_PORT(DIR) &= ~BV(DS2411_PIN); /* input, resistor pull high */ \
    DS2411_PORT(OUT) &= ~BV(DS2411_PIN); /* Px.x == 0 but still input */ \
  } while(0)

/* Set 1-Wire low (output and Px.x == 0) */
#define DS2411_SET_LOW()   (DS2411_PORT(DIR) |= BV(DS2411_PIN))
/* Set 1-Wire high (input, external resistor pull high */
#define DS2411_SET_HIGH()  (DS2411_PORT(DIR) &= ~BV(DS2411_PIN))
/* Read one bit. */
#define DS2411_READ()      (DS2411_PORT(IN) & BV(DS2411_PIN))

/*
 * Delay for u microseconds on a MSP430 at 4MHz.
 *
 * The loop in clock_delay consists of one add and one jnz, i.e 3
 * cycles.
 *
 * 3 cycles at 4MHz ==> 0.75us = 3/4us.
 *
 * Call overhead is roughly 7 cycles and the loop 3 cycles, to
 * compensate for call overheads we make 7/3 fewer laps in the
 * loop.
 *
 * This macro will loose badly if not passed a constant argument, it
 * relies on the compiler doing the arithmetic during compile time!!
 */
#define DS2411_UDELAY(u) clock_delay((u * 4 - 7)/3)

/*
 * Where call overhead dominates, use a macro!
 * Note: assumes CPU speed of 4 Mhz
 */
#define DS2411_UDELAY_6() do {                                  \
    _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP(); _NOP();     \
  } while(0)

#endif /* __DS2411_ARCH_H__ */
