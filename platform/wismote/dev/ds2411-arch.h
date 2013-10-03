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
 *         Architecture-specific definitions for the DS2411 on WiSMote.
 * \author
 *         Niclas Finne <nfi@sics.se>
 */

#ifndef __DS2411_ARCH_H__
#define __DS2411_ARCH_H__

#include "contiki-conf.h"

/*
 * One wire DS2411 configuration
 */

/*
 * DS2411 pin configuration for the WiSMote
 * DS2411 1-Wire is at P1.1.
 */

#define DS2411_PORT(type) P1##type
#define DS2411_PIN        1

#define DS2411_INIT() DS2411_SET_INPUT()

/* Set 1-Wire low (output and Px.x == 0) */
#define DS2411_SET_LOW() do {                   \
    DS2411_PORT(DIR) |= BV(DS2411_PIN);         \
    DS2411_PORT(OUT) &= ~BV(DS2411_PIN);        \
    DS2411_PORT(REN) &= ~BV(DS2411_PIN);        \
  } while(0)

/* Set 1-Wire high (output and Px.x == 1) */
#define DS2411_SET_HIGH() do {                  \
    DS2411_PORT(DIR) |= BV(DS2411_PIN);         \
    DS2411_PORT(OUT) |= BV(DS2411_PIN);         \
    DS2411_PORT(REN) &= ~BV(DS2411_PIN);        \
  } while(0)

/* Set 1-Wire as input (input and Px.x == 1) */
#define DS2411_SET_INPUT() do {                 \
    DS2411_PORT(DIR) &= ~BV(DS2411_PIN);        \
    DS2411_PORT(OUT) |= BV(DS2411_PIN);         \
    DS2411_PORT(REN) |= BV(DS2411_PIN);         \
  } while(0)

/* Read one bit. */
#define DS2411_READ() (DS2411_PORT(IN) & BV(DS2411_PIN))

#define DS2411_UDELAY(u)					\
  __delay_cycles((u * (MSP430_CPU_SPEED / 1000L)) / 1000L)

#endif /* __DS2411_ARCH_H__ */
