/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 *
 * This file is part of the Contiki operating system.
 *
 */
/*
 * Device driver for the Dallas Semiconductor DS2411 chip. Heavily
 * based on the application note 126 "1-Wire Communications Through
 * Software".
 *
 * http://www.maxim-ic.com/appnotes.cfm/appnote_number/126
 */

/*
 * For now we stuff in Moteiv Corporation's unique OUI.
 * From http://www.ethereal.com/distribution/manuf.txt:
 * 00:12:75    Moteiv    # Moteiv Corporation
 *
 * The EUI-64 is a concatenation of the 24-bit OUI value assigned by
 * the IEEE Registration Authority and a 40-bit extension identifier
 * assigned by the organization with that OUI assignment.
 */

#include <string.h>

#include "contiki.h"
#include "dev/ds2411.h"
#include "dev/ds2411-arch.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

unsigned char ds2411_id[8];

#ifndef DS2411_SET_LOW
#error DS2411_SET_LOW needs to be set!
#endif

#ifndef DS2411_SET_HIGH
#error DS2411_SET_HIGH needs to be set!
#endif

#ifndef DS2411_READ
#error DS2411_READ needs to be set!
#endif

/* Set 1-Wire low */
#define OUTP_0() DS2411_SET_LOW()
/* Set 1-Wire high */
#define OUTP_1() DS2411_SET_HIGH()

#ifdef DS2411_SET_INPUT
#define INP() DS2411_SET_INPUT()
#else
#define INP() do {} while(0)
#endif
/* Read one bit. */
#define READP() DS2411_READ()

#ifdef DS2411_UDELAY
#define udelay(u) DS2411_UDELAY(u)
#else
#define udelay(u) clock_delay_usec(u)
#endif

/*
 * Where call overhead dominates, use a macro if supported by the platform!
 */
#ifdef DS2411_UDELAY_6
#define udelay_tA() DS2411_UDELAY_6()
#else
#define udelay_tA() DS2411_UDELAY(6)
#endif

/*
 * Recommended delay times in us.
 */
/*      tA 6			   max 15 */
#define tB 64
#define tC 60			/* max 120 */
#define tD 10
#define tE 9			/* max 12 */
#define tF 55
#define tG 0
#define tH 480
#define tI 70
#define tJ 410
/*---------------------------------------------------------------------------*/
static int
owreset(void)
{
  int result;
  OUTP_0();
  udelay(tH);
  OUTP_1();			/* Releases the bus */
  udelay(tI);
  INP();
  result = READP();
  udelay(tJ);
  return result;
}
/*---------------------------------------------------------------------------*/
static void
owwriteb(unsigned byte)
{
  int i = 7;
  do {
    if(byte & 0x01) {
      OUTP_0();
      udelay_tA();
      OUTP_1();			/* Releases the bus */
      udelay(tB);
    } else {
      OUTP_0();
      udelay(tC);
      OUTP_1();			/* Releases the bus */
      udelay(tD);
    }
    if(i == 0) {
      return;
    }
    i--;
    byte >>= 1;
  } while(1);
}
/*---------------------------------------------------------------------------*/
static unsigned
owreadb(void)
{
  unsigned result = 0;
  int i = 7;
  do {
    OUTP_0();
    udelay_tA();
    OUTP_1();			/* Releases the bus */
    udelay(tE);
    INP();
    if(READP()) {
      result |= 0x80;		/* LSbit first */
    }
    udelay(tF);
    if(i == 0) {
      return result;
    }
    i--;
    result >>= 1;
  } while(1);
}
/*---------------------------------------------------------------------------*/
/* Polynomial ^8 + ^5 + ^4 + 1 */
static unsigned
crc8_add(unsigned acc, unsigned byte)
{
  int i;
  acc ^= byte;
  for(i = 0; i < 8; i++) {
    if(acc & 1) {
      acc = (acc >> 1) ^ 0x8c;
    } else {
      acc >>= 1;
    }
  }
  return acc;
}
/*---------------------------------------------------------------------------*/
int
ds2411_init(void)
{
  int i;
  unsigned family, crc, acc;

#ifdef DS2411_INIT
  DS2411_INIT();

  clock_wait(1);
#endif

  if(owreset() == 0) {	/* Something pulled down 1-wire. */
    /*
     * Read MAC id with interrupts disabled.
     */
    int s = splhigh();
    owwriteb(0x33);		/* Read ROM command. */
    family = owreadb();
    /* We receive 6 bytes in the reverse order, LSbyte first. */
    for(i = 7; i >= 2; i--) {
      ds2411_id[i] = owreadb();
    }
    crc = owreadb();
    splx(s);

    /* Verify family and that CRC match. */
    if(family != 0x01) {
      PRINTF("ds2411: wrong family %d\n", family);
    } else {
      acc = crc8_add(0x0, family);
      for(i = 7; i >= 2; i--) {
        acc = crc8_add(acc, ds2411_id[i]);
      }
      if(acc == crc) {
#ifdef DS2411_OUI
        const unsigned char oui[] = DS2411_OUI;
        memcpy(ds2411_id, oui, sizeof(oui));
#endif /* DS2411_OUI */

#if DEBUG
        {
          int i;
          PRINTF("DS2411: ");
          for(i = 0; i < sizeof(ds2411_id) - 1; i++) {
            PRINTF("%02x.", ds2411_id[i]);
          }
          PRINTF("%02x\n", ds2411_id[i]);
        }
#endif /* DEBUG */

        return 1;			/* Success! */
      }
      PRINTF("ds2411: wrong crc\n");
    }
  }

  PRINTF("ds2411: failed\n");
  memset(ds2411_id, 0x0, sizeof(ds2411_id));
  return 0;			/* Fail! */
}
/*---------------------------------------------------------------------------*/
