/****************************************************************************
 * arch/arm/src/tiva/tiva_can.h
 * Classic (character-device) lower-half driver for the Tiva CAN modules.
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
 * TODO: None of these are implemented yet.
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
 */

#define CAN_TIVA_FIRST (CAN_FIRST + CAN_NCMDS)
#define CAN_TIVA_NCMDS 3

#define CANIOC_TIVA_RXFILTER_FIFO_MAXDEPTH_GET  _CANIOC(CAN_TIVA_FIRST + 0)
#define CANIOC_TIVA_RXFILTER_FIFO_DEPTH_GET     _CANIOC(CAN_TIVA_FIRST + 1)
#define CANIOC_TIVA_RXFILTER_FIFO_DEPTH_SET     _CANIOC(CAN_TIVA_FIRST + 2)

#define CAN_TIVA_FILTER_TYPE_STD 0
#define CAN_TIVA_FILTER_TYPE_EXT 1

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
