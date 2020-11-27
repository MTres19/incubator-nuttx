/****************************************************************************
 * arch/arm/src/tiva/common/tiva_can.c
 *
 *   Copyright (C) 2020 Matthew Trescott
 *   Author: Matthew Trescott <matthewtrescott@gmail.com>
 *
 * References:
 *
 *   TM4C123GH6PM Series Data Sheet
 *   TI Tivaware driverlib ADC sample code.
 *
 * The TivaWare sample code has a BSD compatible license that requires this
 * copyright notice:
 *
 * Copyright (c) 2005-2014 Texas Instruments Incorporated.
 * All rights reserved.
 * Software License Agreement
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This is part of revision 2.1.0.12573 of the Tiva Peripheral Driver
 * Library.
 *
 ****************************************************************************/ 

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/can/can.h>

#include "chip.h"
#include "tiva_can.h"
#include "hardware/tiva_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_TIVA_CAN) && (defined(CONFIG_TIVA_CAN0) || defined(CONFIG_TIVA_CAN1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents a CAN module on the microcontroller */

struct tiva_canmod_s
{
  uint32_t  base;     /* Registers base address */
  
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Callbacks for upper-half CAN driver */

static void tivacan_reset(FAR struct can_dev_s *dev);
static int  tivacan_setup(FAR struct can_dev_s *dev);
static void tivacan_shutdown(FAR struct can_dev_s *dev);
static void tivacan_rxintctl(FAR struct can_dev_s *dev, bool enable);
static void tivacan_txintctl(FAR struct can_dev_s *dev, bool enable);
static int  tivacan_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int  tivacan_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int  tivacan_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool tivacan_txready(FAR struct can_dev_s *dev);
static bool tivacan_txempty(FAR struct can_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* List of callbacks provided to can_register via g_tivacanNdev */

static const struct can_ops_s g_tivacanops
{
  .co_reset         = tivacan_reset;
  .co_setup         = tivacan_setup;
  .co_shutdown      = tivacan_shutdown;
  .co_rxint         = tivacan_rxintctl;
  .co_txint         = tivacan_txintctl;
  .co_ioctl         = tivacan_ioctl;
  .co_remoterequest = tivacan_remoterequest;
  .co_send          = tivacan_send;
  .co_txready       = tivacan_txready;
  .co_txempty       = tivacan_txempty;
};

#ifdef CONFIG_TIVA_CAN0
static struct tiva_canmod_s g_tivacan0priv
{
  .base             = TIVA_CAN0_BASE;
};

static struct can_dev_s g_tivacan0dev
{
  .cd_ops           = &g_tivacanops;
  .cd_priv          = &g_tivacan0priv;
};
#endif

#ifdef CONFIG_TIVA_CAN1
static struct tiva_canmod_s g_tivacan1priv
{
  .base             = TIVA_CAN1_BASE;
};

static struct can_dev_s g_tivacan1dev
{
  .cd_ops           = &g_tivacanops;
  .cd_priv          = &g_tivacan1priv;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tivacan_reset
 * 
 * Description:
 *   Software-reset the CAN module using the sysctl registers.
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Returned value: None
 ****************************************************************************/

static void tivacan_reset(FAR struct can_dev_s *dev)
{
  
}

/****************************************************************************
 * Name: tivacan_setup
 * 
 * Description:
 *   Set up the CAN module. Register the ISR and enable the interrupt in the
 *   NVIC. Set default bit-timing and set filters to a consistent state. On
 *   return, the CAN module is in run (not sleep) mode, the INIT bit is set
 *   in the CANCTL register, and interrupts are disabled on the side of the
 *   CAN module.
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Returned value:
 *   Zero on success, or a negated errno on failure.
 ****************************************************************************/

static int tivacan_setup(FAR struct can_dev_s *dev)
{
  
}

/****************************************************************************
 * Name: tivacan_shutdown
 * 
 * Description:
 *   Shut down the CAN module. This function does the reverse of
 *   tivacan_setup. The ISR is disabled in the NVIC, unregistered from the
 *   NuttX dispatcher, and the CAN module is reset.
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Returned value: None
 ****************************************************************************/
static void tivacan_shutdown(FAR struct can_dev_s *dev)
{
  
}

/****************************************************************************
 * Name: tivacan_rxintctl
 * 
 * Description:
 *   Enable or diable RX interrupts from the CAN module
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   enable - True to enable the interrupt, or false to disable
 * 
 * Returned value: None
 ****************************************************************************/

static void tivacan_rxintctl(FAR struct can_dev_s *dev, bool enable)
{
  
}

/****************************************************************************
 * Name tivacan_txintctl
 * 
 * Description:
 *   Enable or disable TX interrupts from the CAN module
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   enable - True to enable the interrupt, or false to disable
 * 
 * Returned value: None
 ****************************************************************************/
static void tivacan_txintctl(FAR struct can_dev_s *dev, bool enable)
{
  
}

/****************************************************************************
 * Name: tivacan_ioctl
 * 
 * Description:
 *   Perform the action requested by an ioctl syscall that was not handled by
 *   the "upper half" CAN driver.
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   cmd - One of the CANIOC_* ioctls
 *   arg - Second argument to the ioctl call.
 * 
 * Returned value:
 *   Zero on success; a negated errno on failure
 ****************************************************************************/

static int tivacan_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg)
{
  
}

/****************************************************************************
 * Name: tivacan_remoterequest
 * 
 * Description:
 *   Send a remote-request frame.
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   id - CAN message ID
 * 
 * Returned value;
 *   Zero on success; a negated errno on failure.
 ****************************************************************************/

static int tivacan_remoterequest(FAR struct can_dev_s *dev, uint16_t id)
{
  
}

/****************************************************************************
 * Name: tivacan_send
 * 
 * Description:
 *   Enqueue a message to transmit into the hardware FIFO. Returns
 *   immediately if the FIFO is full.
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   msg - The CAN message to enqueue
 * 
 * Returned value:
 *   Zero on success; a negated errorno on failure.
 */

static int tivacan_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg)
{
  
}

/****************************************************************************
 * Name: tivacan_txready
 * 
 * Description:
 *   Determine whether the hardware is ready to transmit another TX message.
 *   This checks for bus-off conditions as well as if the FIFO is full.
 * 
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Returned value:
 *   True if the hardware can accept another TX message, false otherwise.
 ****************************************************************************/

static bool tivacan_txready(FAR struct can_dev_s *dev)
{
  
}

/****************************************************************************
 * Name: tivacan_txempty
 * 
 * Description:
 *   Determines whether the TX FIFO is empty.
 *   
 *   A pointer to this function is passed to can_register.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Returned value:
 *   True if the FIFO is empty, false otherwise.
 ****************************************************************************/

static bool tivacan_txempty(FAR struct can_dev_s *dev)
{
  
}

/****************************************************************************
 * Public Functions
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

int tiva_can_initialize(char *devpath, int modnum)
{
  FAR struct can_dev_s *dev;
  int ret;
  caninfo("tiva_can_initialize module %d\n", modnum);
  
#ifndef CONFIG_TIVA_CAN0
  if (modnum == 0)
    {
      return -ENODEV;
    }
#endif
#ifndef CONFIG_TIVA_CAN1
  if (modnum == 1)
    {
      return -ENODEV;
    }
#endif

  tiva_can_enableclk(modnum);
  
#ifdef CONFIG_TIVA_CAN0
  if (modnum == 0)
    {
      dev = &g_tivacan0dev;
      tiva_configgpio(GPIO_CAN0_RX);
      tiva_configgpio(GPIO_CAN0_TX);
    }
#endif
#ifdef CONFIG_TIVA_CAN1
  if (modnum == 1)
    {
      dev = &g_tivacan1dev;
      tiva_configgpio(GPIO_CAN1_RX);
      tiva_configgpio(GPIO_CAN1_TX);
    }
#endif

  ret = can_register(devpath, dev);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
    }
  
  return ret;
}

#endif /* CONFIG_TIVA_CAN && (CONFIG_TIVA_CAN0 || CONFIG_TIVA_CAN1) */
