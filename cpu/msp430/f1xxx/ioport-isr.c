/*
 * Copyright (c) 2013, Swedish Institute of Computer Science
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
 *         IO Port ISR supporting API
 * \author
 *         Niclas Finne <nfi@sics.se>
 */

#include "dev/ioport-isr.h"
#include "isr_compat.h"

#ifdef IOPORT_ISR_CONF_PORT1
#define IOPORT_ISR_PORT1 IOPORT_ISR_CONF_PORT1
#else
#define IOPORT_ISR_PORT1 1
#endif

#ifdef IOPORT_ISR_CONF_PORT2
#define IOPORT_ISR_PORT2 IOPORT_ISR_CONF_PORT2
#else
#define IOPORT_ISR_PORT2 1
#endif

#define MAX_PORTS 2
#define MAX_PINS 8
struct ioport_isr {
  ioport_isr_callback_t isr[MAX_PINS];
};
static struct ioport_isr ports[MAX_PORTS];
/*---------------------------------------------------------------------------*/
#if IOPORT_ISR_PORT1
ISR(PORT1, irq_port1)
{
  unsigned int index;
  uint8_t pin;

  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  for(pin = 1, index = 0; pin && !(pin & P1IFG); pin <<= 1, index++);

  if(pin) {
    P1IFG &= ~pin;
    if(ports[0].isr[index] != NULL && ports[0].isr[index]()) {
      LPM4_EXIT;
    }
  }

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
#endif /* IOPORT_ISR_PORT1 */
/*---------------------------------------------------------------------------*/
#if IOPORT_ISR_PORT2
ISR(PORT2, irq_port2)
{
  unsigned int index;
  uint8_t pin;

  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  for(pin = 1, index = 0; pin && !(pin & P2IFG); pin <<= 1, index++);

  if(pin) {
    P2IFG &= ~pin;
    if(ports[1].isr[index] != NULL && ports[1].isr[index]()) {
      LPM4_EXIT;
    }
  }

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
#endif /* IOPORT_ISR_PORT2 */
/*---------------------------------------------------------------------------*/
ioport_isr_callback_t
ioport_isr_get(uint8_t port, uint8_t pin)
{
  port--;
  if(pin >= MAX_PINS || port >= MAX_PORTS) {
    return NULL;
  }
  return ports[port].isr[pin];
}
/*---------------------------------------------------------------------------*/
int
ioport_isr_set(uint8_t port, uint8_t pin, ioport_isr_callback_t callback)
{
  int s;

  port--;
  if(pin >= MAX_PINS || port >= MAX_PORTS) {
    return 0;
  }
  s = splhigh();
  ports[port].isr[pin] = callback;
  splx(s);
  return 1;
}
/*---------------------------------------------------------------------------*/
