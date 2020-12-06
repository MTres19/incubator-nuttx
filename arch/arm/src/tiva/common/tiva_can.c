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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <nuttx/can/can.h>

#include "arm_arch.h"
#include "chip.h"
#include "tiva_can.h"
#include "tiva_enableclks.h"
#include "tiva_gpio.h"
#include "hardware/tiva_pinmap.h"
#include "hardware/tiva_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_TIVA_CAN) && (defined(CONFIG_TIVA_CAN0) || defined(CONFIG_TIVA_CAN1))


/* Convenience macro for combining two 16-bit registers into a single 32 bit
 * value
 */
#define tivacan_readsplitreg32(r1, r2) (getreg32(r1) | getreg32(r2) << 16)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tiva_can_fifo_s
{
  /* Bitfield representing the message objects belonging to this FIFO. The
   * high bit (mask: 1 << 31) corresponds to message object 31, the low bit
   * (mask: 1 << 0) corresponds to message object 0.
   * 
   * TODO: If a new chip came along with more message objects, this would
   * need to be reworked.
   */
  uint32_t  msg_nums;

  /* Filter assigned to this FIFO. Not used if this is the TX FIFO
   * TODO: Is this necessary? */
#ifdef CONFIG_CAN_EXTID
  bool      extended;
  struct canioc_extfilter_s xfilter;
#endif
  struct canioc_stdfilter_s sfilter;
};
  

/* This structure represents a CAN module on the microcontroller */

struct tiva_canmod_s
{
  int       modnum;         /* Module number, mostly for syscon register fields */
  uint32_t  base;           /* Registers base address */
  uint32_t  status_reg;     /* Contents of the CANSTS reg, updated in ISR */
  
  mutex_t   thd_iface_mtx;  /* Mutex for threads accessing the interface registers */
  uint32_t  thd_iface_base; /* Interface registers for threads */
  uint32_t  isr_iface_base; /* Interface registers for the ISR */
  
  /* FIFOs, including TX and RX filter FIFOs */
  mutex_t   fifo_mtx;
  struct tiva_can_fifo_s fifos[CONFIG_TIVA_CAN_FILTERS_MAX + 1];
  
  /* Pointers to default FIFOs initialized by the driver */
  FAR struct tiva_can_fifo_s *tx_fifo;
  FAR struct tiva_can_fifo_s *rxdefault_fifo;
};

/* This structure represents the CAN bit timing parameters */

struct tiva_can_timing_s
{
  int       prescaler;
  int       tseg1;
  int       tseg2;
  int       sjw;
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

/* ISR */

static int  tivacan_unified_isr(int irq, FAR void *context, FAR void *dev);

/* Internal utility functions */

static struct tiva_can_timing_s tivacan_bittiming_get(FAR struct can_dev_s *dev);
static void tivacan_bittiming_set(FAR struct tiva_can_timing_s *timing,
                                  FAR struct can_dev_s *dev);

static struct tiva_can_fifo_s  *tivacan_alloc_fifo(FAR struct can_dev_s *dev,
                                                    int depth);
static void tivacan_free_fifo(FAR struct can_dev_s *dev,
                                 FAR struct tiva_can_fifo_s *fifo);
static void tivacan_disable_msg_obj(uint32_t iface_base, int num);
static int  tivacan_initfilter_std(FAR struct can_dev_s          *dev,
                                   FAR struct tiva_can_fifo_s    *fifo,
                                   FAR struct canioc_stdfilter_s *filter);

#ifdef CONFIG_CAN_EXTID
static int  tivacan_initfilter_ext(FAR struct can_dev_s          *dev,
                                   FAR struct tiva_can_fifo_s    *fifo,
                                   FAR struct canioc_extfilter_s *filter);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* List of callbacks provided to can_register via g_tivacanNdev */

static const struct can_ops_s g_tivacanops =
{
  .co_reset         = tivacan_reset,
  .co_setup         = tivacan_setup,
  .co_shutdown      = tivacan_shutdown,
  .co_rxint         = tivacan_rxintctl,
  .co_txint         = tivacan_txintctl,
  .co_ioctl         = tivacan_ioctl,
  .co_remoterequest = tivacan_remoterequest,
  .co_send          = tivacan_send,
  .co_txready       = tivacan_txready,
  .co_txempty       = tivacan_txempty,
};

#ifdef CONFIG_TIVA_CAN0

static struct tiva_canmod_s g_tivacan0priv =
{
  .modnum           = 0,
  .base             = TIVA_CAN_BASE(0),
  .thd_iface_base   = TIVA_CAN_IFACE_BASE(0, 0),
  .isr_iface_base   = TIVA_CAN_IFACE_BASE(0, 1),
  .tx_fifo          = NULL,
  .rxdefault_fifo   = NULL,
};

static struct can_dev_s g_tivacan0dev =
{
  .cd_ops           = &g_tivacanops,
  .cd_priv          = &g_tivacan0priv,
};

#endif /* CONFIG_TIVA_CAN0 */

#ifdef CONFIG_TIVA_CAN1

static struct tiva_canmod_s g_tivacan1priv =
{
  .modnum           = 1,
  .base             = TIVA_CAN_BASE(1),
  .thd_iface_base   = TIVA_CAN_IFACE(BASE(1, 0),
  .isr_iface_base   = TIVA_CAN_IFACE_BASE(1, 1),
  .tx_fifo          = NULL,
  .rxdefault_fifo   = NULL,
};

static struct can_dev_s g_tivacan1dev =
{
  .cd_ops           = &g_tivacanops,
  .cd_priv          = &g_tivacan1priv,
};
#endif /* CONFIG_TIVA_CAN1 */

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
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  int modnum = canmod->modnum;
#ifndef CONFIG_TIVA_CAN0
  if (modnum == 0)
    {
      canerr("ERROR: tried to reset disabled module CAN0\n");
      return;
    }
#endif
#ifndef CONFIG_TIVA_CAN1
  if (modnum == 1)
    {
      canerr("ERROR: tried to reset disabled module CAN1\n");
    }
#endif
  if (modnum > 1)
    {
      canerr("ERROR: tried to reset nonexistant module CAN%d\n", canmod->modnum);
    }
  
  modifyreg32(TIVA_SYSCON_SRCAN, 0, SYSCON_SRCAN(modnum));
  modifyreg32(TIVA_SYSCON_SRCAN, SYSCON_SRCAN(modnum), 0);
}

/****************************************************************************
 * Name: tivacan_setup
 * 
 * Description:
 *   Set up the CAN module. Register the ISR and enable the interrupt in the
 *   NVIC. Set default bit-timing and set filters to a consistent state. On
 *   return, the INIT bit is cleared in the CANCTL register, and interrupts
 *   are enabled on the side of the CAN module. (can_ops_s.txint and
 *   can_ops_s.rxint are unsuitable because the Tiva CAN modules do not allow
 *   disabling TX and RX interrupts separately).
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
  FAR struct tiva_canmod_s    *canmod = dev->cd_priv;
  struct canioc_stdfilter_s default_filter =
  {
    .sf_id1   = 0x123,
    .sf_id2   = 0,
    .sf_type  = CAN_FILTER_MASK,
    .sf_prio  = CAN_MSGPRIO_HIGH,
  };
  
  /* TODO: This does not belong here, different boards could have different
   * crystals!
   */
  struct tiva_can_timing_s default_timing =
  {
    .prescaler  = 4,
    .tseg1      = 6,
    .tseg2      = 3,
    .sjw        = 3,
  };
  
  /* Set default bit timing */
  tivacan_bittiming_set(&default_timing, dev);
  
  nxmutex_lock(&canmod->thd_iface_mtx);
  
  /* Disable all message objects */
  for (int i = 0; i < TIVA_CAN_NUM_MSG_OBJS; ++i)
    {
      tivacan_disable_msg_obj(canmod->thd_iface_base, i);
    }
  
  nxmutex_unlock(&canmod->thd_iface_mtx);
  
  /* Register the ISR */
  switch (canmod->modnum)
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
     return -ENODEV;
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
  
  /* Exit init mode and enable interrupts from the CAN module */
  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL, TIVA_CAN_CTL_INIT,
              TIVA_CAN_CTL_EIE | TIVA_CAN_CTL_SIE | TIVA_CAN_CTL_IE);
  
  canmod->tx_fifo = tivacan_alloc_fifo(dev,
                                        CONFIG_TIVA_CAN_DEFAULT_FIFO_DEPTH);
  canmod->rxdefault_fifo = tivacan_alloc_fifo(dev,
                                        CONFIG_TIVA_CAN_DEFAULT_FIFO_DEPTH);
  
  /* TODO: Check return values */
  
  tivacan_initfilter_std(dev, canmod->rxdefault_fifo, &default_filter);
  
  
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
  int irq;
  FAR struct tiva_canmod_s    *canmod = dev->cd_priv;
  
  switch(canmod->modnum)
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
  
  /* Free all allocated FIFOs (RX filters + 1 for the TX FIFO) */
  
  for (int i = 0; i < CONFIG_TIVA_CAN_FILTERS_MAX + 1; ++i)
  {
    if (canmod->fifos[i].msg_nums)
    {
      tivacan_free_fifo(dev, &canmod->fifos[i]);
    }
  }
  
  /* Disable interrupts from the CAN module */
  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
              TIVA_CAN_CTL_EIE | TIVA_CAN_CTL_SIE | TIVA_CAN_CTL_IE, 0);
  
  up_disable_irq(irq);
  irq_detach(irq);
  
  tivacan_reset(dev);
}

/****************************************************************************
 * Name: tivacan_rxintctl
 * 
 * Description:
 *   The Tiva CAN modules do not allow individual control of TX
 *   and RX interrupts, so this function does nothing. Interrupts
 *   are enabled and disabled in tivacan_setup and tivacan_shutdown.
 *   
 *   A pointer to this function is passed to can_register
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   enable - True to enable the interrupt, or false to disable
 * 
 * Returned value: None
 ****************************************************************************/

static void tivacan_rxintctl(FAR struct can_dev_s *dev, bool enable)
{
  return;
}

/****************************************************************************
 * Name tivacan_txintctl
 * 
 * Description:
 *   The Tiva CAN modules do not allow individual control of TX
 *   and RX interrupts, so this function does nothing. Interrupts
 *   are enabled and disabled in tivacan_setup and tivacan_shutdown.
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
  return;
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

static int  tivacan_unified_isr(int irq, FAR void *context, FAR void *dev)
{
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t canint;
  struct can_msg_s msg;
  
  while (canint = getreg32(canmod->base + TIVA_CAN_OFFSET_INT)
                                      & TIVA_CAN_INT_INTID_MASK)
  {
    if (canint == TIVA_CAN_INT_INTID_STATUS)
      {
        /* Clear the status interrupt and report the errors */
        canmod->status_reg = getreg32(canmod->base + TIVA_CAN_OFFSET_STS);
#ifdef CONFIG_CAN_ERRORS
        switch (canmod->status_reg & TIVA_CAN_STS_LEC_MASK)
        {
        case TIVA_CAN_STS_LEC_NONE:
          {
            
          }
        break;
        case TIVA_CAN_STS_LEC_STUFF:
          {
            
            
          }
        break;
        case TIVA_CAN_STS_LEC_FORM:
          {
            
          }
        break;
        case TIVA_CAN_STS_LEC_ACK:
          {
            
          }
        break;
        case TIVA_CAN_STS_LEC_BIT1:
          {
            
          }
        break;
        case TIVA_CAN_STS_LEC_BIT0:
          {
            
          }
        break;
        case TIVA_CAN_STS_LEC_CRC:
          {
            
          }
        break;
        case TIVA_CAN_STS_LEC_NOEVENT:
          {
            
          }
        default:
        }
#endif
      }
    else if (canint > 1 && canint < TIVA_CAN_NUM_MSG_OBJS)
      {
        /* Process the transmitted or received message */
      }
    else
      {
        /* The interrupt handler was called, but there is no interrupt...! */
      }
  }
  
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
  uint32_t canbase = ((struct tiva_canmod_s *)dev->cd_priv)->base;
  
  uint32_t canbit;
  uint32_t brpe;
  canbit = getreg32(canbase + TIVA_CAN_OFFSET_BIT);
  brpe   = getreg32(canbase + TIVA_CAN_OFFSET_BRPE);
  
  timing.prescaler = (canbit & TIVA_CAN_BIT_BRP_MASK) >> TIVA_CAN_BIT_BRP_SHIFT;
  timing.prescaler |= ((brpe & TIVA_CAN_BRPE_BRPE_MASK)
                        >> TIVA_CAN_BRPE_BRPE_SHIFT)
                        << TIVA_CAN_BIT_BRP_LENGTH;
                        
  timing.tseg1 = (canbit & TIVA_CAN_BIT_TSEG1_MASK) >> TIVA_CAN_BIT_TSEG1_SHIFT;
  timing.tseg2 = (canbit & TIVA_CAN_BIT_TSEG2_MASK) >> TIVA_CAN_BIT_TSEG2_SHIFT;
  timing.sjw   = (canbit & TIVA_CAN_BIT_SJW_MASK) >> TIVA_CAN_BIT_SJW_SHIFT;
  
  /* Values stored in registers are 1 less than the values used */
  
  ++timing.prescaler;
  ++timing.tseg1;
  ++timing.tseg2;
  ++timing.sjw;
  
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
  uint32_t    canbit;
  uint32_t    brpe;
  uint32_t    ctl;
  bool        init_mode;
  uint32_t    canbase = ((struct tiva_canmod_s *)dev->cd_priv)->base;
  
  DEBUGASSERT(timing->prescaler > TIVA_CAN_PRESCALER_MIN &&
              timing->prescaler < TIVA_CAN_PRESCALER_MAX);
  DEBUGASSERT(timing->tseg1 > TIVA_CAN_TSEG1_MIN &&
              timing->tseg1 < TIVA_CAN_PRESCALER_MAX);
  DEBUGASSERT(timing->tseg2 > TIVA_CAN_TSEG2_MIN &&
              timing->tseg2 < TIVA_CAN_TSEG2_MIN);
  DEBUGASSERT(timing->sjw > TIVA_CAN_SJW_MIN &&
              timing->sjw < TIVA_CAN_SJW_MAX);
  
  flags = enter_critical_section();
  
  ctl = getreg32(canbase + TIVA_CAN_OFFSET_CTL);
  
  /* Save the current state of the init bit */
  init_mode = ctl & TIVA_CAN_CTL_INIT;
  
  /* Enable writes to CANBIT and BRPE - set INIT and Configuration Change
   * Enable (CCE)
   */
  ctl |= TIVA_CAN_CTL_INIT | TIVA_CAN_CTL_CCE;
  putreg32(ctl, canbase + TIVA_CAN_OFFSET_CTL);
  
  canbit = getreg32(canbase + TIVA_CAN_OFFSET_BIT);
  brpe   = getreg32(canbase + TIVA_CAN_OFFSET_BRPE);
  
  canbit &= ~(TIVA_CAN_BIT_BRP_MASK | TIVA_CAN_BIT_TSEG1_MASK
              | TIVA_CAN_BIT_TSEG2_MASK | TIVA_CAN_BIT_SJW_MASK);
  
  /* Masking bits that belong in the baud rate prescaler extension register.
   * Values stored in registers are 1 less than the value used.
   */
  
  canbit |= ((timing->prescaler - 1) << TIVA_CAN_BIT_BRP_SHIFT)
              & TIVA_CAN_BIT_BRP_MASK;
  brpe   |= ((timing->prescaler - 1) >> TIVA_CAN_BIT_BRP_LENGTH)
                                      << TIVA_CAN_BRPE_BRPE_SHIFT;
  
  canbit |= (timing->tseg1 - 1) << TIVA_CAN_BIT_TSEG1_SHIFT;
  canbit |= (timing->tseg2 - 1) << TIVA_CAN_BIT_TSEG2_SHIFT;
  canbit |= (timing->sjw - 1) << TIVA_CAN_BIT_SJW_SHIFT;
  
  putreg32(canbit, canbase + TIVA_CAN_OFFSET_BIT);
  putreg32(brpe, canbase + TIVA_CAN_OFFSET_BRPE);
  
  /* Lock changes to CANBIT and BRPE (clear Configuration Change Enable) */
  ctl &= ~TIVA_CAN_CTL_CCE;
  
  /* Exit init mode if the baud rate was changed on-the-fly */
  if (!init_mode)
    {
      ctl &= ~TIVA_CAN_CTL_INIT;
    }
  
  putreg32(ctl, canbase + TIVA_CAN_OFFSET_CTL);
  
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: tivacan_alloc_fifo
 * 
 * Description:
 *   Try to allocate a FIFO of the specified depth in the CAN module SRAM.
 *   (FIFOs in the Tiva CAN modules don't need to be contiguous)
 *   
 *   This is a driver-internal utility function.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   depth - The desired FIFO depth to allocate
 * 
 * Return value: A pointer to tiva_can_fifo_s, which contains a bit-field
 *               of the message objects that may be used, or NULL if there
 *               is not enough room in the CAN module SRAM.
 ****************************************************************************/

static struct tiva_can_fifo_s  *tivacan_alloc_fifo(FAR struct can_dev_s *dev,
                                                    int depth)
{
  uint32_t claimed = 0;
  int i;
  int numclaimed = 0;
  int free_fifo_idx;
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  
  nxmutex_lock(&canmod->fifo_mtx);
  
  /* Message objects allocated other FIFOs. Filter RX FIFOs plus the TX FIFO */
  
  for (i = 0; i < CONFIG_TIVA_CAN_FILTERS_MAX + 1; ++i)
    {
      if (canmod->fifos[i].msg_nums == 0)
        {
          free_fifo_idx = i;
        }
      
      claimed |= canmod->fifos[i].msg_nums;
    }
  
  /* Message objects that can be allocated to this FIFO */
  claimed = ~claimed;
  
  for (i = 0; i < TIVA_CAN_NUM_MSG_OBJS && numclaimed < depth; ++i)
    {
      if (claimed & 1 << i)
        {
          ++numclaimed;
        }
    }
  
  if (numclaimed != depth)
    {
      nxmutex_unlock(&canmod->fifo_mtx);
      return NULL;
    }
  else
    {
      /* Clear the bits for message objects we don't need to claim right now.
       * TODO: If a new chip came along with more than 32 message objects,
       *       this would need to be reworked.
       */
      claimed &= 0xffffffff >> (31 - i);
      canmod->fifos[free_fifo_idx].msg_nums = claimed;
      
      nxmutex_unlock(&canmod->fifo_mtx);
      return &canmod->fifos[free_fifo_idx];
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
 *   This is a driver-internal utility function.
 * 
 * Input parameters:
 *   dev - An instance of the "upper half" CAN driver structure
 *   fifo - The FIFO to free
 * 
 * Return value: None
 ****************************************************************************/
static void tivacan_free_fifo(FAR struct can_dev_s *dev,
                                 FAR struct tiva_can_fifo_s *fifo)
{
  FAR struct tiva_canmod_s * canmod = dev->cd_priv;
  int modnum = canmod->modnum;
  nxmutex_lock(&canmod->thd_iface_mtx);
  
  for (int i = 0; i < 32; ++i)
    {
      if (fifo->msg_nums & 1 << i)
        {
          /* Wait for pending transmisions or reads to complete */
          
          while ((TIVA_CAN_TXRQ1(modnum) | TIVA_CAN_TXRQ2(modnum) << 16
                  | TIVA_CAN_NWDA1(modnum) | TIVA_CAN_NWDA2(modnum) << 16)
                  & 1 << i);
          
          tivacan_disable_msg_obj(canmod->thd_iface_base, i);
        }
    }
  
  nxmutex_unlock(&canmod->thd_iface_mtx);
}

/****************************************************************************
 * Name: tivacan_disable_msg_obj
 * 
 * Description:
 *   Disable a message object by clearing the MSGVAL bit. This function
 *   doesn't do any checks, it just disables it. Don't call it without
 *   checking whether there's a pending message in the object!
 *   
 *   This is a driver-internal utility function.
 * 
 * Input parameters:
 *   iface_base - The interface registers base address to use
 *   num - The message object number to disable
 * 
 * Return value: none
 ****************************************************************************/

static void tivacan_disable_msg_obj(uint32_t iface_base, int num)
{
  /* No need to use modifyreg32 here, we have exclusive interface access */
  uint32_t reg;
  
  /* Signal a write (WRNRD) to the message SRAM touching the ARB registers */
  putreg32(TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_ARB,
           iface_base + TIVA_CANIF_OFFSET_CMSK);
  
  /* Mark message object as invalid */\
  putreg32(~(TIVA_CANIF_ARB2_MSGVAL), iface_base + TIVA_CANIF_OFFSET_ARB2);
  
  /* Make sure the interface is not busy */
  while (getreg32(iface_base + TIVA_CANIF_OFFSET_CRQ) & TIVA_CANIF_CRQ_BUSY);
  
  /* Submit request */
  putreg32(num, iface_base + TIVA_CANIF_OFFSET_CRQ);
}

/****************************************************************************
 * Name: tivacan_initfilter_std
 * 
 * Description:
 *   Initialize the message objects in the specified FIFO to store messages
 *   matching the given filter. On return, the FIFO will generate interrupts
 *   on matching messages (if the CAN hardware is initialized).
 *   
 *   Note that standard filters only match standard messages.
 *   
 *   This is a driver-internal utility function.
 * 
 * Input parameters:
 *   dev   - An instance of the "upper half" CAN driver structure
 *   fifo  - A FIFO in the message SRAM obtained by calling tivacan_alloc_fifo
 *   filter - A canioc_stdfilter_s of type CAN_FILTER_MASK
 * 
 * Return value:
 ****************************************************************************/

static int  tivacan_initfilter_std(FAR struct can_dev_s          *dev,
                                   FAR struct tiva_can_fifo_s    *fifo,
                                   FAR struct canioc_stdfilter_s *filter)
{
  uint32_t  reg;
  uint32_t  msgval;  /* Message valid bitfield */
  uint32_t  newdat;  /* New data bitfield (for RX message objects) */
  uint32_t  txrq;    /* Pending transmission bitfield */
  
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t  canbase = canmod->base;
  uint32_t  iface_base = canmod->thd_iface_base;
  bool      eob_set = false; /* Whether the End of Buffer bit has been set yet */
  
  nxmutex_lock(&canmod->thd_iface_mtx);
  
  for (int i = TIVA_CAN_NUM_MSG_OBJS - 1; i >= 0; --i)
  {
    if (fifo->msg_nums & (1 << i))
    {
      do
        {
          msgval = tivacan_readsplitreg32(
                      canbase + TIVA_CAN_OFFSET_MSG1VAL,
                      canbase + TIVA_CAN_OFFSET_MSG2VAL);
          newdat = tivacan_readsplitreg32(
                      canbase + TIVA_CAN_OFFSET_NWDA1,
                      canbase + TIVA_CAN_OFFSET_NWDA2);
          txrq   = tivacan_readsplitreg32(
                      canbase + TIVA_CAN_OFFSET_TXRQ1,
                      canbase + TIVA_CAN_OFFSET_TXRQ2);
        }
      /* Wait for any remaining reads or transmissions to complete */
      while (msgval & (newdat | txrq) & 1 << i); /* TODO: This could cause a lock-up if the interrupt handler can't claim an iface */
      
      /* Command Mask Register
       * 
       * Write to message SRAM and access mask, control, and arbitration
       * bits. Clear any interrupts (which might be spurious due to
       * the hardware being uninitialized
       */
      reg = TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_MASK | TIVA_CANIF_CMSK_CONTROL
             | TIVA_CANIF_CMSK_ARB | TIVA_CANIF_CMSK_CLRINTPND;
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_CMSK);
      
      /* Mask 2 Register
       * Filter out extended messages, but allow remote request frames through.
       * This is done by disabling filter-on-direction (MDIR = 0), but setting
       * the DIR bit in the ARB2 register to 1 (TX).
       */
      reg = TIVA_CANIF_MSK2_MXTD;
      reg |= filter->sf_id2 << TIVA_CANIF_MSK2_IDMSK_STD_SHIFT;
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MSK2);
      
      /* Arbitration 2 Register
       * Mark the message object as valid. Set the DIR bit to allow remote
       * frames. Leave the XTD bit clear to indicate standard frames (we
       * filter on this in MSK2.
       */
      reg = TIVA_CANIF_ARB2_MSGVAL | TIVA_CANIF_ARB2_DIR;
      reg |= filter->sf_id1 << TIVA_CANIF_ARB2_ID_STD_SHIFT;
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_ARB2);
      
      /* Message Control Register
       * Use acceptance filter masks, enable RX interrupts for this message
       * object. DLC is initialized to zero but updated by received frames.
       */
      reg = TIVA_CANIF_MCTL_UMASK | TIVA_CANIF_MCTL_RXIE;
      if (!eob_set)
        {
          reg |= TIVA_CANIF_MCTL_EOB;
          eob_set = true;
        }
      
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MCTL);
      
      /* Request the registers be transferred to the message RAM and wait
       * for the transfer to complete.
       */
      putreg32(i, iface_base + TIVA_CANIF_OFFSET_CRQ);
      while (getreg32(iface_base + TIVA_CANIF_OFFSET_CRQ) & TIVA_CANIF_CRQ_BUSY);
    }
  }
  
  nxmutex_unlock(&canmod->thd_iface_mtx);
  
  return OK;
}

#ifdef CONFIG_CAN_EXTID

/****************************************************************************
 * Name: tivacan_initfilter_ext
 * 
 * Description:
 *   Initialize the message objects in the specified FIFO to store messages
 *   matching the given filter. On return, the FIFO will generate interrupts
 *   on matching messages (if the CAN hardware is initialized).
 * 
 *   Note that standard filters only match standard messages, but extended
 *   filters will match both standard and extended message IDs.
 *   TODO: Implement an ioctl to control the behavior of this.
 *   
 *   This is a driver-internal utility function.
 * 
 * Input parameters:
 *   dev   - An instance of the "upper-half" CAN driver structure
 *   fifo  - A FIFO in the message SRAM obtained by calling tivacan_alloc_fifo
 *   filter - A canioc_stdfilter_s of type CAN_FILTER_MASK
 * 
 * Return value:
 ****************************************************************************/

static int  tivacan_initfilter_ext(FAR struct can_dev_s          *dev,
                                   FAR struct tiva_can_fifo_s    *fifo,
                                   FAR struct canioc_extfilter_s *filter)
{
  
  uint32_t  reg;
  uint32_t  msgval;  /* Message valid bitfield */
  uint32_t  newdat;  /* New data bitfield (for RX message objects) */
  uint32_t  txrq;    /* Pending transmission bitfield */
  
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t  canbase = canmod->base;
  uint32_t  iface_base = canmod->thd_iface_base;
  bool      eob_set = false; /* Whether the End of Buffer bit has been set yet */
  
  for (int i = TIVA_CAN_NUM_MSG_OBJS - 1; i >= 0; --i)
  {
    if (fifo->msg_nums & (1 << i))
    {
      do
        {
          msgval = tivacan_readsplitreg32(
                      canbase + TIVA_CAN_OFFSET_MSG1VAL,
                      canbase + TIVA_CAN_OFFSET_MSG2VAL);
          newdat = tivacan_readsplitreg32(
                      canbase + TIVA_CAN_OFFSET_NWDA1,
                      canbase + TIVA_CAN_OFFSET_NWDA2);
          txrq   = tivacan_readsplitreg32(
                      canbase + TIVA_CAN_OFFSET_TXRQ1,
                      canbase + TIVA_CAN_OFFSET_TXRQ2);
        }
      /* Wait for any remaining reads or transmissions to complete */
      while (msgval & (newdat | txrq) & 1 << i); /* TODO: This could cause a lock-up if the interrupt handler can't claim an iface */
      
      /* Command Mask Register
       * 
       * Write to message SRAM and access mask, control, and arbitration
       * bits. Clear any interrupts (which might be spurious due to
       * the hardware being uninitialized
       */
      reg = TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_MASK | TIVA_CANIF_CMSK_CONTROL
             TIVA_CANIF_CMSK_ARB | TIVA_CANIF_CMSK_CLRINTPND;
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_CMSK);
      
      /* Mask 1 Register - lower 16 bits of extended ID mask */
      putreg32(filter->xf_id2 & TIVA_CANIF_MSK1_IDMSK_EXT_MASK);
      
      /* Mask 2 Register
       * Allow remote request frames through. This is done by disabling
       * filter-on-direction (MDIR = 0), but setting the DIR bit in the ARB2
       * register to 1 (TX).
       */
      reg = filter->xf_id2 & TIVA_CANIF_MSK2_IDMSK_EXT_MSK;
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MSK2);
      
      /* Arbitration 1 Register - lower 16 bits of extended ID */
      putreg32(filter->xf_id1 & TIVA_CANIF_ARB1_ID_EXT_MASK);
      
      /* Arbitration 2 Register
       * Mark the message object as valid. Set the DIR bit to allow remote frames.
       */
      reg = TIVA_CANIF_ARB2_MSGVAL | TIVA_CANIF_ARB2_DIR;
      reg |= filter->xf_id1 & TIVA_CANIF_ARB2_ID_EXT_MASK;
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_ARB2);
      
      /* Message Control Register
       * Use acceptance filter masks, enable RX interrupts for this message
       * object. DLC is initialized to zero but updated by received frames.
       */
      reg = TIVA_CANIF_MCTL_UMASK | TIVA_CANIF_MCTL_RXIE;
      if (!eob_set)
        {
          reg |= TIVA_CANIF_MCTL_EOB;
          eob_set = true;
        }
      
      putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MCTL);
      
      /* Request the registers be transferred to the message RAM and wait
       * for the transfer to complete.
       */
      putreg32(i, iface_base + TIVA_CANIF_OFFSET_CRQ);
      while (getreg32(iface_base + TIVA_CANIF_OFFSET_CRQ) & TIVA_CANIF_CRQ_BUSY);
    }
  }
  
  nxmutex_unlock(&canmod->thd_iface_mtx);
  
  return OK;
}

#endif /* CONFIG_CAN_EXTID */

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
  FAR struct tiva_canmod_s *canmod;
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
  
  canmod = dev->cd_priv;
  
  /* Initialize concurrancy objects for accessing interfaces */
  
  ret = nxmutex_init(&canmod->thd_iface_mtx);
  if (ret < 0)
    {
      canerr("ERROR: failed to initialize mutex: %d\n", ret);
      return ret;
    }
  
  ret = nxmutex_init(&canmod->fifo_mtx);
  if (ret < 0)
    {
      canerr("ERROR: failed to initialize mutex: %d\n", ret);
      return ret;
    }
  
  /* Register the driver */
  
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
