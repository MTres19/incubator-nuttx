/****************************************************************************
 * arch/arm/src/tiva/tiva_can.h
 *
 *   Copyright (C) 2020 Matthew Trescott
 *   Author: Matthew Trescott <matthewtrescott@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/ 

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_CAN_H
#define __ARCH_ARM_SRC_TIVA_TIVA_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_CAN) && (defined(CONFIG_TIVA_CAN0) || defined(CONFIG_TIVA_CAN1))

/* Tiva-specific Ioctl Commands *********************************************
 * 
 * CANIOC_TIVA_RXFILTER_FIFO_MAXDEPTH_GET
 *   Description:     Returns the maximum size of the filter's RX FIFO
 *                    (not the actual size of the FIFO). For Tiva CAN
 *                    modules, this is based on the number of message
 *                    objects allocated to the TX FIFO and the other RX
 *                    filter FIFOs.
 *   Argument:        The filter ID returned by CANIOC_ADD_STDFILTER or
 *                    CANIOC_ADD_EXTFILTER
 *   Returned value:  The filter's maximum possible filter depth, based
 *                    on the size of the TX FIFO and the other RX filter
 *                    FIFOs.
 * 
 * CANIOC_TIVA_RXFILTER_FIFO_DEPTH_GET
 *   Description:     Returns the current size of the filter's RX FIFO.
 *   Argument:        The filter ID returned by CANIOC_ADD_STDFILTER or
 *                    CANIOC_ADD_EXTFILTER
 *   Returned value:  The filter's FIFO depth, i.e. the number of messages
 *                    it can store.
 * 
 * CANIOC_TIVA_RXFILTER_FIFO_DEPTH_SET
 *   Description:     Set the FIFO depth for an RX filter.
 *   Argument:        The filter ID returned by CANIOC_ADD_STDFILTER or
 *                    CANIOC_ADD_EXTFILTER, with the requested FIFO depth
 *                    shifted left by 16 bits and ORed with filter ID.
 *   Returned value:  Zero on success, a negated errno on failure.
 * 
 * CANIOC_TIVA_TXFIFO_MAX_DEPTH_GET
 *   Description:     Returns the maximum size of the hardware TX FIFO
 *                    (not the actual size of the FIFO). For Tiva CAN modules,
 *                    this is based on the number of message objects allocated
 *                    to RX filter FIFOs.
 *   Argument:        Not used
 *   Returned value:  The TX FIFO's maximum size, i.e. the maximum number of
 *                    messages that can be queued for transmission in hardware.
 * 
 * CANIOC_TIVA_TXFIFO_DEPTH_GET
 *   Description:     Returns the current size of the TX FIFO.
 *   Argument:        Not used
 *   Returned value:  The current depth of the TX FIFO, i.e. the number of
 *                    messages that can be queued for transmission in hardware.
 * 
 * CANIOC_TIVA_TXFIFO_DEPTH_SET
 *   Description:     Set the depth of the hardware TX FIFO.
 *   Argument:        The requested TX FIFO depth
 *   Returned value:  Zero on success, a negated errno on failure.
 * 
 * CANIOC_TIVA_ENQUEUE_RTR_MESSAGE
 *   TODO
 */

#define CAN_TIVA_FIRST (CAN_FIRST + CAN_NCMDS)
#define CAN_TIVA_NCMDS 7

#define CANIOC_TIVA_RXFILTER_FIFO_MAXDEPTH_GET  _CANIOC(CAN_TIVA_FIRST + 0)
#define CANIOC_TIVA_RXFILTER_FIFO_DEPTH_GET     _CANIOC(CAN_TIVA_FIRST + 1)
#define CANIOC_TIVA_RXFILTER_FIFO_DEPTH_SET     _CANIOC(CAN_TIVA_FIRST + 2)
#define CANIOC_TIVA_TXFIFO_MAX_DEPTH_GET        _CANIOC(CAN_TIVA_FIRST + 3)
#define CANIOC_TIVA_TXFIFO_DEPTH_GET            _CANIOC(CAN_TIVA_FIRST + 4)
#define CANIOC_TIVA_TXFIFO_DEPTH_SET            _CANIOC(CAN_TIVA_FIRST + 5)
#define CANIOC_TIVA_ENQUEUE_RTR_MESSAGE         _CANIOC(CAN_TIVA_FIRST + 6)

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_can_initialize
 * 
 * Description:
 *   Initialize the selected CAN module
 * 
 * Input Parameters:
 *   Device path, a string of the form "/dev/can0" or "/dev/can1"
 *   Module number, for chips with multiple modules (typically 0 or 1)
 * 
 * Returned Value:
 *   Pointer to can_dev_s (CAN device structure), or NULL on failure.
 * 
 ****************************************************************************/

int tiva_can_initialize(FAR char *devpath, int modnum);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CAN && (CONFIG_TIVA_CAN0 || CONFIG_TIVA_CAN1) */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_CAN_H */
