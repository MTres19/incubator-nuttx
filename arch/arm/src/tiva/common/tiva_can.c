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
#include <nuttx/sempahore.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>

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

/* This structure represents a set of CAN interface registers that can be
 * claimed by a thread */

struct tiva_can_iface_s
{
  int       num;    /* Interface number. Currently only 2 are suppported */
  int       modnum; /* CAN module number for accessing registers */
  bool      avail;  /* Whether the interface has been claimed */
  uint32_t  base;   /* Interface base address */
};

struct tiva_can_fifo_s
{
  /* Bitfield representing the message objects belonging to this FIFO */
  uint32_t  msg_nums;

  /* Filter assigned to this FIFO. Not used if this is the TX FIFO */
#ifdef CONFIG_CAN_EXTID
  bool      extended;
  struct canioc_extfilter_s xfilter;
#endif
  struct canioc_stdfilter_s sfilter;
};
  

/* This structure represents a CAN module on the microcontroller */

struct tiva_canmod_s
{
  int       modnum;    /* Module number, mostly for syscon register fields */
  uint32_t  base;      /* Registers base address */
  sem_t     iface_sem; /* Semaphore for queuing access to an interface */
  mutex_t   iface_mtx; /* Mutex for actually claiming an interface */
  struct tiva_can_iface_s *ifaces; /* Array of interfaces */
  
  /* FIFOs, including TX and RX filter FIFOs */
  mutex_t   fifo_mtx;
  struct tiva_can_fifo_s fifos[CONFIG_TIVA_CAN_FILTERS_MAX + 1];
};

/* This structure represents the CAN bit timing parameters */

struct tiva_can_timing_s
{
  int       prescaler;
  int       tseg1;
  int       tseg2;
  int       sjw;
};

/* This structure mirrors the CANIFnCRQ and CANIFnCMSK registers. */

struct tiva_can_readcmd_s
{
  int   msg_num;
  bool  read_mask;
  bool  read_arb;
  bool  read_ctl;
  bool  read_data;
  bool  clr_intpnd;
  bool  clr_newdat;
}
  

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

/* ISR */

static void tivacan_unified_isr(FAR struct can_dev_s *dev);

/* Internal utility functions */

static void tivacan_initmode_ctl(bool enable);
static struct tiva_can_timing_s tivacan_bittiming_get(FAR struct can_dev_s *dev);
static void tivacan_bittiming_set(FAR struct tiva_can_timing_s *timing,
                                  FAR struct can_dev_s *dev);

static struct tiva_can_iface_s *tivacan_claim_iface(FAR struct can_dev_s *dev);
static void tivacan_release_iface(FAR struct can_dev_s *dev, int num);

static struct tiva_can_fifo_s  *tivacan_claim_fifo(FAR struct can_dev_s *dev,
                                                    int depth);
static void tivacan_release_fifo(FAR struct can_dev_s *dev,
                                 FAR struct tiva_can_iface_s *iface,
                                 FAR struct tiva_can_fifo_s *fifo);
static void tivacan_disable_msg_obj(FAR struct tiva_can_iface_s *iface,
                                    int num);

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

static struct tiva_can_iface_s g_tivacan0ifaces[] = {
#  if TIVA_CAN_NUM_IFACES >= 1
  struct tiva_can_iface_s {
    .num    = 0;
    .avail  = true;
    .base   = TIVA_CAN_IFACE_BASE(0, 0);
  },
#  endif
#  if TIVA_CAN_NUM_IFACES >= 2
  struct tiva_can_iface_s {
    .num    = 1;
    .avail  = true;
    .base   = TIVA_CAN_IFACE_BASE(0, 1);
  }
#  else
#    error "Number of CAN interfaces out of range"
#  endif
};

static struct tiva_canmod_s g_tivacan0priv
{
  .modnum           = 0;
  .base             = TIVA_CAN_BASE(0);
  .ifaces           = g_tivacan0ifaces;
};

static struct can_dev_s g_tivacan0dev
{
  .cd_ops           = &g_tivacanops;
  .cd_priv          = &g_tivacan0priv;
};
#endif

#ifdef CONFIG_TIVA_CAN1

static struct tiva_can_iface_s g_tivacan1ifaces[] = {
#  if TIVA_CAN_NUM_IFACES >= 1
  struct tiva_can_iface_s {
    .num = 0;
    .avail  = true;
    .base = TIVA_CAN_IFACE_BASE(1,0);
  },
#  endif
#  if TIVA_CAN_NUM_IFACES >= 2
  struct tiva_can_iface_s {
    .num = 1;
    .avail = true;
    .base  = TIVA_CAN_IFACE_BASE(1,1);
  }
#  else
#    error "Number of CAN interfaces out of range"
#  endif
};

static struct tiva_canmod_s g_tivacan1priv
{
  .modnum           = 1;
  .base             = TIVA_CAN_BASE(1);
  .ifaces           = tivacan1ifaces;
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
 *   Software-reset the CAN module using the SYSCON registers.
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
#ifndef CONFIG_TIVA_CAN0
  if (dev->cd_priv->modnum == 0)
    {
      canerr("ERROR: tried to reset disabled module CAN0\n");
      return;
    }
#endif
#ifndef CONFIG_TIVA_CAN1
  if (dev->cd_priv->modnum == 1)
    {
      canerr("ERROR: tried to reset disabled module CAN1\n");
    }
#endif
  if (dev->cd_priv->modnum > 1)
    {
      canerr("ERROR: tried to reset nonexistant module CAN%d\n", dev->cd_priv->modnum);
    }
  
  modifyreg32(TIVA_SYSCON_SRCAN, 0, SYSCON_SRCAN(dev->cd_priv->modnum));
  modifyreg32(TIVA_SYSCON_SRCAN, SYSCON_SRCAN(dev->cd_priv->modnum), 0);
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
  uint32_t  irq;
  int       ret;
  struct tivacan_iface_s *iface;
  
  /* TODO: This does not belong here, different boards could have different
   * crystals!
   */
  const struct tiva_can_timing_s default_timing
  {
    .prescaler  = 4;
    .tseg1      = 6;
    .tseg2      = 3;
    .sjw        = 3;
  };
  
  /* Set default bit timing */
  tivacan_bittiming_set(&default_timing, dev);
  
  iface = tivacan_claim_iface(dev); /* TODO check for null? */
  
  /* Disable all message objects */
  for (int i = 0; i < 32; ++i)
    {
      tivacan_disable_msg_obj(dev, iface, i);
    }
  
  /* Register the ISR */
  switch (dev->cd_priv->modnum)
  {
  case 0:
    {
      irq = TIVA_IRQ_CAN0;
    }
    break;
  case 1:
    {
      irq = TIVA_IRQ_CAN1;
    }
    break;
  default:
    {
     canerr("ERROR: tried to register interrupt for nonexistant CAN module\n");
     return -ENNODEV;
    }
  }
  
  ret = irq_attach(irq, tivacan_unified_isr, dev);
  if (ret < 0)
    {
      canerr("ERROR: Failed to register CAN ISR.\n");
      return ret;
    }
  
  /* Enable the ISR */
  up_enable_irq(irq);
  return OK;
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
  switch(dev->cd_priv->modnum)
  {
  case 0:
    {
      irq = TIVA_IRQ_CAN0;
    }
    break;
  case 1:
    {
      irq = TIVA_IRQ_CAN1;
    }
    break;
  default:
      canerr("ERROR: Tried to disable interrupt for nonexistant CAN module\n");
  }
  
  up_disable_irq(irq);
  irq_detach(irq);
  
  tivacan_reset(dev);
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
 * Name: tivacan_unified_isr
 * 
 * Description:
 *   ISR executed for all interrupts generated by the CAN module. The Tiva
 *   CAN modules don't have separate ISRs for different events, hence the
 *   name "unified."
 *   
 *   This ISR is registered in tivacan_setup.
 * 
 * Returned value: None
 ****************************************************************************/

static void tivacan_unified_isr(FAR struct can_dev_s *dev)
{
  
}


/****************************************************************************
 * Name: tivacan_bittiming_get
 * 
 * Description:
 *   Get the CAN bit timing parameters from the hardware (prescaler, TSEG1,
 *   TSEG2, and SJW). The values returned are the actual values in time
 *   quanta, not the values stored in the register (which are off by one).
 *   
 *   This is a driver-internal utility function.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Returned value:
 *   struct tiva_can_timing_s containing the four parameters listed above.
 ****************************************************************************/

static struct tiva_can_timing_s tivacan_bittiming_get(FAR struct can_dev_s *dev)
{
  struct tiva_can_timing_s timing;
  
  uint32_t canbit;
  uint32_t brpe;
  canbit = getreg32(dev->cd_priv->base + TIVA_CAN_OFFSET_BIT);
  brpe   = getreg32(dev->cd_priv->base + TIVA_CAN_OFFSET_BRPE);
  
  timing.prescaler = (canbit & TIVA_CAN_BIT_BRP_MASK) >> TIVA_CAN_BIT_BRP_SHIFT;
  timing.prescaler |= ((brpe & TIVA_CAN_BRPE_BRPE_MASK)
                        >> TIVA_CAN_BRPE_BRPE_SHIFT)
                        << TIVA_CAN_BIT_BRP_LENGTH;
                        
  timing.tseg1 = (canbit & TIVA_CAN_BIT_TSEG1_MASK) >> TIVA_CAN_BIT_TSEG1_SHIFT;
  timing.tseg2 = (canbit & TIVA_CAN_BIT_TSEG2_MASK) >> TIVA_CAN_BIT_TSEG2_SHIFT;
  timing.sjw   = (canbit & TIVA_CAN_BIT_SJW_MASK) >> TIVA_CAN_BIT_SJW_SHIFT;
  
  return timing;
}

/****************************************************************************
 * Name: tivacan_bittiming_set
 * 
 * Description:
 *   Set the CAN bit timing parameters to the hardware registers. The values
 *   used should be the actual values in time quanta, not the values stored in
 *   the registers (which are off by one).
 *   
 *   This is a driver-internal utility function.
 * 
 * Input parameters:
 *   timing - An instance of tiva_can_timing_s, with the prescaler, TSEG1,
 *            TSEG2, and SJW set to their real (not off-by-one) values.
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Returned value: None
 ****************************************************************************/

static void tivacan_bittiming_set(FAR struct tiva_can_timing_s *timing,
                                  FAR struct can_dev_s *dev)
{
  irqstate_t flags;
  uint32_t   canbit;
  uint32_t   brpe;
  
  DEBUGASSERT(timing->prescaler > TIVA_CAN_PRESCALER_MIN &&
              timing->prescaler < TIVA_CAN_PRESCALER_MAX);
  DEBUGASSERT(timing->tseg1 > TIVA_CAN_TSEG1_MIN &&
              timing->tseg1 < TIVA_CAN_PRESCALER_MAX);
  DEBUGASSERT(timing->tseg2 > TIVA_CAN_TSEG2_MIN &&
              timing->tseg2 < TIVA_CAN_TSEG2_MIN);
  DEBUGASSERT(timing->sjw > TIVA_CAN_SJW_MIN &&
              timing->sjw < TIVA_CAN_SJW_MAX);
  
  flags = enter_critical_section();
  
  canbit = getreg32(dev->cd_priv->base + TIVA_CAN_OFFSET_BIT);
  brpe   = getreg32(dev->cd_priv->base + TIVA_CAN_OFFSET_BRPE);
  
  canbit &= ~(TIVA_CAN_BIT_BRP_MASK | TIVA_CAN_BIT_TSEG1_MASK
              | TIVA_CAN_BIT_TSEG2_MASK | TIVA_CAN_BIT_SJW_MASK);
  
  /* Enable writes to CANBIT and BRPE (set Configuration Change Enable) */
  modifyreg32(dev->cd_priv->base + TIVA_CAN_OFFSET_CTL, 0, TIVA_CAN_CTL_CCE);
  
  /* Masking bits that belong in the baud rate prescaler extension register */
  
  canbit |= ((timing->prescaler - 1) << TIVA_CAN_BIT_BRP_SHIFT)
              & TIVA_CAN_BIT_BRP_MASK;
  brpe   |= ((timing->prescaler - 1) >> TIVA_CAN_BIT_BRP_LENGTH)
                                      << TIVA_CAN_BIT_BRPE_SHIFT;
  
  canbit |= (timing->tseg1 - 1) << TIVA_CAN_BIT_TSEG1_SHIFT;
  canbit |= (timing->tseg2 - 1) << TIVA_CAN_BIT_TSEG2_SHIFT;
  canbit |= (timing->sjw - 1) << TIVA_CAN_BIT_SJW_SHIFT;
  
  putreg32(canbit, dev->cd_priv->base + TIVA_CAN_OFFSET_BIT);
  putreg32(brpe, dev->cd_priv->base + TIVA_CAN_OFFSET_BRPE);
  
  /* Lock changes to CANBIT and BRPE (clear Configuration Change Enable) */
  modifyreg32(dev->cd_priv + TIVA_CAN_OFFSET_CTL, TIVA_CAN_CTL_CCE, 0);
  
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tivacan_claim_iface
 * 
 * Description:
 *   Claim exclusive access to a set of CANIF registers
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 * 
 * Return value:
 *   A pointer to a tiva_can_iface_s that is guaranteed to be free, NULL on
 *   failure.
 ****************************************************************************/
static struct tiva_can_iface_s *tivacan_claim_iface(FAR struct can_dev_s *dev)
{
  struct tiva_can_iface_s *iface;
  
  nxsem_wait(dev->cd_priv->iface_sem);
  nxmutex_lock(&dev->cd_priv->iface_mtx);
  
  for (int i = 0; i < TIVA_CAN_NUM_IFACES; ++i)
    {
      if (can_dev_s->cd_priv->ifaces[i].avail)
        {
          can_dev_s->cd_priv->ifaces[i].avail = false;
          nxmutex_unlock(&dev->cd_priv_iface_mtx);
          return &can_dev_s->cd_priv->ifaces[i];
        }
    }
    return NULL;
}

/****************************************************************************
 * Name: tivacan_release_iface
 * 
 * Description:
 *   Release a set of CANIF registers for another thread to use
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   num - The interface number, from tiva_can_iface_s.num
 * 
 * Return value: None
 ****************************************************************************/

static void tivacan_release_iface(FAR struct can_dev_s *dev, int num)
{
  nxmutex_lock(&dev->cd_priv->iface_mtx);
  dev->cd_priv->ifaces[num].avail = true;
  nxmutex_unlock(&dev->cd_priv->iface_mtx);
  nxsem_post(dev->cd_priv->iface_sem);
}

/****************************************************************************
 * Name: tivacan_alloc_fifo
 * 
 * Description:
 *   Try to allocate a FIFO of the specified depth in the CAN module SRAM.
 *   (FIFOs in the Tiva CAN modules don't need to be contiguous)
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   depth - The desired FIFO depth to allocate
 * 
 * Return value: A pointer to tiva_can_fifo_s, which contains a bit-field
 *               of the message objects that may be used.
 ****************************************************************************/
static struct tiva_can_fifo_s  *tivacan_alloc_fifo(FAR struct can_dev_s *dev,
                                                    int depth)
{
  uint32_t claimed = 0;
  int i;
  int numclaimed = 0;
  int free_fifo_idx;
  
  nxmutex_lock(&dev->cd_priv->fifo_mtx);
  
  /* Message objects allocated other FIFOs. Filter RX FIFOs plus the TX FIFO */
  
  for (i = 0; i < CONFIG_TIVA_CAN_FILTERS_MAX + 1; ++i)
    {
      if (dev->cd_priv->fifos[i]->msg_nums == 0)
        {
          free_fifo_idx = i;
        }
      
      claimed |= dev->cd_priv->fifos[i].msg_nums;
    }
  
  /* Message objects that can be allocated to this FIFO */
  claimed = ~claimed;
  
  for (i = 31; i > 0 && numclaimed < depth; --i) /* TODO remove hardcoded 31 */
    {
      if (claimed & 1 << i)
        {
          ++numclaimed;
        }
    }
  
  if (numclaimed != depth)
    {
      nxmutex_unlock(&dev->cd_priv->fifo_mtx);
      return NULL;
    }
  else
    {
      /* Clear the bits for message objects we don't need to claim right now */
      claimed &= 0xffffffff >> i;
      dev->cd_priv->fifos[free_fifo_idx].msg_nums;
      
      nxmutex_unlock(&dev->cd_priv->fifo_mtx);
      return &dev->cd_priv->fifos[free_fifo_idx];
    }
}

/****************************************************************************
 * Name: tivacan_free_fifo
 * 
 * Description:
 *   Disable all the message objects in the specified FIFO and make them
 *   available for use by another filter. This function waits for pending
 *   transmissions to complete (for the TX FIFO) or new messages to be
 *   read (for RX filter FIFOs) before disabling a message object.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   iface - An instance of tiva_can_iface_s obtained with tivacan_claim_iface
 *   fifo - The FIFO to free
 * 
 * Return value: None
 ****************************************************************************/
static void tivacan_free_fifo(FAR struct can_dev_s *dev,
                              FAR struct tiva_can_iface_s *iface,
                                 FAR struct tiva_can_fifo_s *fifo)
{
  int modnum = dev->cd_priv->modnum;
  
  for (int i = 0; i < 32; ++i)
    {
      if (fifo->msg_nums & 1 << i)
        {
          /* Wait for pending transmisions or reads to complete */
          
          while ((TIVA_CAN_TXRQ1(modnum) | TIVA_CAN_TXRQ2(modnum) << 16
                  | TIVA_CAN_NWDA1(modnum) | TIVA_CAN_NWDA2(modnum) << 16)
                  & 1 << i);
          
          tivacan_disable_msg_obj(dev, iface, i);
        }
    }
}

/****************************************************************************
 * Name: tivacan_disable_msg_obj
 * 
 * Description:
 *   Disable a message object by clearing the MSGVAL bit. This function
 *   doesn't do any checks, it just disables it. Don't call it without
 *   checking whether there's a pending message in the object! A slower but
 *   more thorough version is tivacan_command_submit.
 * 
 * Input parameters:
 *   iface - An instance of tiva_can_iface_s claimed by calling
 *           tivacan_claim_iface
 *   num - The message object number to disable
 * 
 * Return value: none
 ****************************************************************************/

static void tivacan_disable_msg_obj(FAR struct tiva_can_iface_s *iface,
                                    int num)
{
  /* No need to use modifyreg32 here, we have exclusive interface access */
  uint32_t reg;
  
  /* Signal a write (WRNRD) to the message SRAM touching the ARB registers */
  reg = getreg32(iface->base + TIVA_CANIF_CMSK_OFFSET);
  reg |= TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_ARB;
  putreg32(reg, iface->base + TIVA_CANIF_CMSK_OFFSET);
  
  /* Mark message object as invalid */
  reg = getreg32(iface->base + TIVA_CANIF_ARB2_OFFSET);
  reg &= ~(TIVA_CANIF_ARB2_MSGVAL);
  putreg32(reg, iface->base + TIVA_CANIF_ARB2_OFFSET);
  
  /* Submit request */
  putreg32(num, iface->base + TIVA_CANIF_CRQ_OFFSET);
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

int tiva_can_initialize(FAR char *devpath, int modnum)
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
      ret = tiva_configgpio(GPIO_CAN0_RX);
      if (ret < 0)
        {
          goto configgpio_error;
        }
      
      ret = tiva_configgpio(GPIO_CAN0_TX);
      if (ret < 0)
        {
          goto configgpio_error;
        }
    }
#endif
#ifdef CONFIG_TIVA_CAN1
  if (modnum == 1)
    {
      dev = &g_tivacan1dev;
      ret = tiva_configgpio(GPIO_CAN1_RX);
      if (ret < 0)
        {
          goto configgpio_error;
        }
      
      ret = tiva_configgpio(GPIO_CAN1_TX);
      if (ret < 0)
        {
          goto configgpio_error;
        }
    }
#endif
  
  /* Initialize concurrancy objects for accessing interfaces */
  ret = nxsem_init(&dev->cd_priv->iface_sem, 0, TIVA_CAN_NUM_IFACES);
  if (ret < 0)
    {
      canerr("ERROR: failed to initialize semaphore: %d\n", ret);
      return ret;
    }
  
  ret = nxmutex_init(&dev->cd_priv->iface_mtx);
  if (ret < 0)
    {
      canerr("ERROR: failed to initialize mutex: %d\n", ret);
      return ret;
    }
  
  ret = nxmutex_init(&dev->cd_priv->fifo_mtx)
  if (ret < 0)
    {
      canerr("ERROR: failed to initialize mutex: %d\n", ret);
      return ret;
    }
  
  ret = can_register(devpath, dev);
  if (ret < 0)
    {
      canerr("ERROR: can_register failed: %d\n", ret);
    }
  
  return ret;
  
configgpio_error:
  canerr("ERROR: failed to configure CAN GPIO pin.\n");
  return ret;
}

#endif /* CONFIG_TIVA_CAN && (CONFIG_TIVA_CAN0 || CONFIG_TIVA_CAN1) */
