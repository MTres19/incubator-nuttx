/****************************************************************************
 * arch/arm/src/tiva/common/tiva_can.c
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
#include "hardware/tiva_can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_TIVA_CAN) && (defined(CONFIG_TIVA_CAN0) || defined(CONFIG_TIVA_CAN1))

/* Convenience macro for combining two 16-bit registers into a single 32 bit
 * value
 */
#define tivacan_readsplitreg32(base, r1, r2) ((getreg32((base) + (r1)) & 0xff) | (getreg32((base) + (r2)) << 16))

/* Only the last mailbox is used for TX to ensure that responses to remote
 * frames we send are always collected by an RX FIFO and not by the mailbox
 * used to send the remote frame. (The Tiva hardware uses the mailbox with
 * the lowest number first.)
 */
#define TIVA_CAN_TX_MBOX_MASK 0x80000000
#define TIVA_CAN_TX_MBOX_NUM  32

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tiva_can_fifo_s
{
  /* Bitfield representing the message objects belonging to this FIFO. The
   * high bit (mask: 1 << 31) corresponds to message object 31, the low bit
   * (mask: 1 << 0) corresponds to message object 0.
   * 
   * NOTE: If a new chip came along with more message objects, this would
   * need to be reworked.
   */
  uint32_t  msg_nums;
};
  

/* This structure represents a CAN module on the microcontroller */

struct tiva_canmod_s
{
  int       modnum;         /* Module number, mostly for syscon register fields */
  uint32_t  base;           /* Registers base address */
  uint32_t  status;         /* Contents of the CANSTS reg, updated in ISR */
  uint32_t  errors;         /* Contents of the CANERR reg, updated in ISR */
  bool      autorecover;    /* Whether to autorecover from bus-off conditions */
  
  mutex_t   thd_iface_mtx;  /* Mutex for threads accessing the interface registers */
  uint32_t  thd_iface_base; /* Interface registers for threads */
  uint32_t  isr_iface_base; /* Interface registers reserved for the ISR */
  
  /* RX filter FIFOs */
  mutex_t   fifo_mtx;
  struct tiva_can_fifo_s fifos[CONFIG_TIVA_CAN_FILTERS_MAX];
  
  /* Pointer to default FIFO initialized by the driver. Only software FIFOs
   * are used for TX because the hardware does not support arbitration on
   * outbound messages and this driver wants to be portable to the SocketCAN
   * model which (I think) performs software arbitration.
   * 
   * The driver is hardcoded to use message object 1 (or zero, depending on
   * how you look at it) for TX.
   */
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
static void tivacan_bittiming_set(FAR struct can_dev_s *dev,
                                  FAR struct tiva_can_timing_s *timing);

int tivacan_alloc_fifo(FAR struct can_dev_s *dev, int depth);
static void tivacan_free_fifo(FAR struct can_dev_s *dev,
                                 FAR struct tiva_can_fifo_s *fifo);
static void tivacan_disable_msg_obj(uint32_t iface_base, int num);
static int  tivacan_initfilter(FAR struct can_dev_s          *dev,
                               FAR struct tiva_can_fifo_s    *fifo,
                               FAR void                      *filter,
                               int                            type);

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
  .thd_iface_base   = TIVA_CAN_IFACE_BASE(1, 0),
  .isr_iface_base   = TIVA_CAN_IFACE_BASE(1, 1),
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
      canerr("ERROR: tried to reset nonexistant module CAN%d\n",
             canmod->modnum);
    }
  
  modifyreg32(TIVA_SYSCON_SRCAN, 0, SYSCON_SRCAN(modnum));
  
  /* Spin for a few clock cycles */
  for (int i = 0; i < 16; ++i);
  
  modifyreg32(TIVA_SYSCON_SRCAN, SYSCON_SRCAN(modnum), 0);
  
  /* Wait for peripheral-ready signal */
  while (!(getreg32(TIVA_SYSCON_PRCAN) & (1 << modnum)));
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
  
#ifdef CONFIG_CAN_EXTID
  struct canioc_extfilter_s default_filter =
  {
    .xf_id1   = 0x123,
    .xf_id2   = 0,
    .xf_type  = CAN_FILTER_MASK,
    .xf_prio  = CAN_MSGPRIO_HIGH,
  };
#else
  struct canioc_stdfilter_s default_filter =
  {
    .sf_id1   = 0x123,
    .sf_id2   = 0,
    .sf_type  = CAN_FILTER_MASK,
    .sf_prio  = CAN_MSGPRIO_HIGH,
  };
#endif
  
  /* TODO: This does not belong here, different boards could have different
   * crystals!
   */
  struct tiva_can_timing_s default_timing =
  {
    .prescaler  = 8,
    .tseg1      = 6,
    .tseg2      = 3,
    .sjw        = 3,
  };
  
  /* Clear the status interrupt, just in case */
  getreg32(canmod->base + TIVA_CAN_OFFSET_STS);
  
  /* Set default bit timing */
  tivacan_bittiming_set(dev, &default_timing);
  
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
  
  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL, 0, TIVA_CAN_CTL_TEST);
  modifyreg32(canmod->base + TIVA_CAN_OFFSET_TST, 0xffffffff, 0);
  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL, TIVA_CAN_CTL_TEST, 0);
  
  /* Exit init mode and enable interrupts from the CAN module */
  modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL, TIVA_CAN_CTL_INIT,
              TIVA_CAN_CTL_EIE | TIVA_CAN_CTL_SIE | TIVA_CAN_CTL_IE);
  
  ret = tivacan_alloc_fifo(dev, CONFIG_TIVA_CAN_DEFAULT_FIFO_DEPTH);
  if (ret < 0)
    {
      canerr("ERROR: Failed to allocate default RX FIFO.\n");
      return ret;
    }
  else
    {
      canmod->rxdefault_fifo = &canmod->fifos[ret];
    }
  
  /* TODO: Check return values */
  
#ifdef CONFIG_CAN_EXTID
  tivacan_initfilter(dev,
                     canmod->rxdefault_fifo,
                     &default_filter,
                     CAN_TIVA_FILTER_TYPE_EXT);
#else
  tivacan_initfilter(dev,
                     canmod->rxdefault_fifo,
                     &default_filter,
                     CAN_TIVA_FILTER_TYPE_STD);
#endif
  
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
  
  /* Free all allocated RX filter FIFOs */
  
  for (int i = 0; i < CONFIG_TIVA_CAN_FILTERS_MAX; ++i)
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
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  int ret;
  
  switch (cmd)
    {
      case CANIOC_GET_BITTIMING:
      {
        FAR struct canioc_bittiming_s *bt =
          (FAR struct canioc_bittiming_s *)arg;
        struct tiva_can_timing_s timing;
        
        timing = tivacan_bittiming_get(dev);
        bt->bt_tseg1 = timing.tseg1;
        bt->bt_tseg2 = timing.tseg2;
        bt->bt_sjw   = timing.sjw;
        bt->bt_baud  = (SYSCLK_FREQUENCY / (unsigned long)timing.prescaler)
                        / (timing.tseg1 + timing.tseg2 + 1);
        ret = OK;
      }
      break;
      
      case CANIOC_SET_BITTIMING:
        {
          /* TODO: This is VERY crude--there's no guarantees that rounding
           * won't cause problems with the prescaler selection.
           * However, NuttX's interface here is kinda sub-optimal anyway--
           * the choice of TSEG1, TSEG2, and SJW depend on the system clock
           * and the prescaler value, so there should be an ioctl for
           * the driver to take physical parameters like delay time and
           * crystal frequency tolerances into account and calculate the
           * timing parameters automagically... This could be implemented
           * in the upper half driver.
           */
          FAR const struct canioc_bittiming_s *bt =
            (FAR struct canioc_bittiming_s *)arg;
          struct tiva_can_timing_s timing;
          
          timing.tseg1 = bt->bt_tseg1;
          timing.tseg2 = bt->bt_tseg2;
          timing.sjw   = bt->bt_sjw;
          timing.prescaler = SYSCLK_FREQUENCY 
            / (bt->bt_baud * (bt->bt_tseg1 + bt->bt_tseg2 + 1));
          DEBUGASSERT(timing.tseg1 <= 16 && timing.tseg1 >= 1);
          DEBUGASSERT(timing.tseg2 <= 8 && timing.tseg1 >= 1);
          DEBUGASSERT(timing.sjw <= 4 && timing.sjw >= 1);
          DEBUGASSERT(timing.prescaler <= 1024 && timing.prescaler >= 1);
          
          tivacan_bittiming_set(dev, &timing);
          ret = OK;
        }
        break;
      
      case CANIOC_GET_CONNMODES:
        {
          ret = -ENOSYS;
        }
        break;
      
      case CANIOC_SET_CONNMODES:
        {
          ret = -ENOSYS;
        }
        break;
      
      case CANIOC_ADD_EXTFILTER:
      case CANIOC_ADD_STDFILTER:
        {
          int fifo_idx = tivacan_alloc_fifo(dev,
                                            CONFIG_TIVA_CAN_DEFAULT_FIFO_DEPTH);
          
          if (fifo_idx < 0)
            {
              /* Probably -ENOSPC or -ENOANO */
              ret = fifo_idx;
              break;
            }
          
          if (cmd == CANIOC_ADD_EXTFILTER)
            {
              tivacan_initfilter(dev, &canmod->fifos[fifo_idx],
                                  (FAR void *)arg, CAN_TIVA_FILTER_TYPE_EXT);
            }
          else
            {
              tivacan_initfilter(dev, &canmod->fifos[fifo_idx],
                                  (FAR void *)arg, CAN_TIVA_FILTER_TYPE_STD);
            }
          
          if (canmod->rxdefault_fifo != NULL)
            {
              tivacan_free_fifo(dev, canmod->rxdefault_fifo);
              canmod->rxdefault_fifo = NULL;
            }
          
          ret = fifo_idx;
        }
        break;
      
      case CANIOC_DEL_EXTFILTER:
      case CANIOC_DEL_STDFILTER:
        {
          if (arg < 0 || arg >= CONFIG_TIVA_CAN_FILTERS_MAX)
            {
              ret = -EINVAL;
            }
          else
            {
              tivacan_free_fifo(dev, &canmod->fifos[arg]);
              ret = OK;
            }
        }
        break;
      
      case CANIOC_BUSOFF_RECOVERY:
        {
          uint32_t init = getreg32(canmod->base + TIVA_CAN_OFFSET_CTL)
                          & TIVA_CAN_CTL_INIT;
          uint32_t boff = canmod->status & TIVA_CAN_STS_BOFF;
          
          if (!boff && !init)
            {
              caninfo("The CAN module is not in bus-off mode.\n");
              ret = -ENMFILE;
            }
          else if (!boff && init)
            {
              canwarn("WARN: The CAN module is in INIT mode but not bus-off! "
                      "Exiting init mode.\n");
              modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
                          TIVA_CAN_CTL_INIT, 0);
              ret = -EIO;
            }
          else if (boff && !init)
            {
              caninfo("Already recovering from bus-off.\n");
              ret = -EINPROGRESS;
            }
          else
            {
              modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL,
                          TIVA_CAN_CTL_INIT, 0);
              ret = OK;
            }
        }
        break;
      
      case CANIOC_SET_NART:
        {
          modifyreg32(canmod->base + TIVA_CAN_OFFSET_CTL, 0, TIVA_CAN_CTL_DAR);
          ret = OK;
        }
        break;
      
      case CANIOC_SET_ABOM:
        {
          canmod->autorecover = true;
          ret = OK;
        }
        break;
      default:
        canerr("ERROR: Unrecognized ioctl\n");
        ret = -ENOSYS;
        break;
    }
  
  return ret;
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
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t reg = 0;
  
  if (canmod->status & TIVA_CAN_STS_BOFF)
    {
      return -ENETDOWN;
    }
  
  if (!tivacan_txempty(dev))
    {
      canerr("Cannot send message—TX mailbox in use.\n");
      return -EBUSY;
    }
    
  nxmutex_lock(&canmod->thd_iface_mtx);
  
  /* NOTE: being extra careful here and setting TXRQST in the command mask
   * register even though we also set it in the message control register.
   */
  reg = TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_ARB | TIVA_CANIF_CMSK_CONTROL
        | TIVA_CANIF_CMSK_CLRINTPND | TIVA_CANIF_CMSK_TXRQST
        | TIVA_CANIF_CMSK_DATAA | TIVA_CANIF_CMSK_DATAB;
  putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_CMSK);
  
  /* Arbitration registers (ARB1 and ARB2) */
  
#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      reg = msg->cm_hdr.ch_id & TIVA_CANIF_ARB1_ID_EXT_MASK;
      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB1);
      
      reg = (msg->cm_hdr.ch_id >> TIVA_CANIF_ARB2_ID_EXT_PRESHIFT)
            & TIVA_CANIF_ARB2_ID_EXT_MASK;
      reg |= TIVA_CANIF_ARB2_MSGVAL | TIVA_CANIF_ARB2_XTD;
      
    #ifdef CONFIG_CAN_USE_RTR
      reg |= (msg->cm_hdr.ch_rtr ? 0 : TIVA_CANIF_ARB2_DIR);
    #else
      reg |= TIVA_CANIF_ARB2_DIR;
    #endif
      
      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB2);
    }
  else
    {
#endif /* CONFIG_CAN_EXTID */
      reg = (msg->cm_hdr.ch_id << TIVA_CANIF_ARB2_ID_STD_SHIFT)
            & TIVA_CANIF_ARB2_ID_STD_MASK;
      
      reg |= TIVA_CANIF_ARB2_MSGVAL;
      
    #ifdef CONFIG_CAN_USE_RTR
      reg |= (msg->cm_hdr.ch_rtr ? 0 : TIVA_CANIF_ARB2_DIR);
    #else
      reg |= TIVA_CANIF_ARB2_DIR;
    #endif
      
      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_ARB2);
#ifdef CONFIG_CAN_EXTID
    }
#endif
  
  /* Message Control (MCTL) register */
  
  reg = msg->cm_hdr.ch_dlc & TIVA_CANIF_MCTL_DLC_MASK;
  
  /* NOTE: Not sure whether the End Of Buffer bit means anything here */
  reg |= TIVA_CANIF_MCTL_NEWDAT | TIVA_CANIF_MCTL_TXIE
          | TIVA_CANIF_MCTL_TXRQST | TIVA_CANIF_MCTL_EOB;
  putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_MCTL);
  
  /* Data registers, there are four of them. */
  for (int i = 0; i < (msg->cm_hdr.ch_dlc + 1) / 2; ++i)
    {
      reg = (uint32_t)msg->cm_data[i * 2] << TIVA_CANIF_DATA_HBYTE_SHIFT;
      reg |= (uint32_t)msg->cm_data[i * 2 + 1] << TIVA_CANIF_DATA_LBYTE_SHIFT;
      putreg32(reg, canmod->thd_iface_base + TIVA_CANIF_OFFSET_DATA(i));
    }
  
  /* Submit request */
  putreg32(TIVA_CAN_TX_MBOX_NUM,
           canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ);
  
  /* Wait for completion */
  while (getreg32(canmod->thd_iface_base + TIVA_CANIF_OFFSET_CRQ)
          & TIVA_CANIF_CRQ_BUSY);
  
  nxmutex_unlock(&canmod->thd_iface_mtx);
  return OK;
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
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  
  if (canmod->status & TIVA_CAN_STS_BOFF)
  {
    return false;
  }
  
  return tivacan_txempty(dev);
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
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t txrq = tivacan_readsplitreg32(canmod->base, TIVA_CAN_OFFSET_TXRQ1,
                                         TIVA_CAN_OFFSET_TXRQ2);
  
  return !(txrq & TIVA_CAN_TX_MBOX_MASK);
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
  FAR struct tiva_canmod_s *canmod = ((FAR struct can_dev_s *)dev)->cd_priv;
  struct can_msg_s msg;
  uint32_t cansts;
  uint32_t canerr;
  uint32_t canint;
  
#ifdef CONFIG_CAN_ERRORS
  int      errcnt;
  int      prev_errcnt;
#endif
  
  while((canint = getreg32(canmod->base + TIVA_CAN_OFFSET_INT)
                                      & TIVA_CAN_INT_INTID_MASK))
    {
      msg = (const struct can_msg_s){0};
      
      if (canint == TIVA_CAN_INT_INTID_STATUS)
        {
          /* Clear the status interrupt by reading CANSTS */
          cansts = getreg32(canmod->base + TIVA_CAN_OFFSET_STS);
          
          /* Get the contents of the error counter register */
          canerr = getreg32(canmod->base + TIVA_CAN_OFFSET_ERR);
          
#ifdef CONFIG_CAN_ERRORS
          msg.cm_hdr.ch_error = true;
          msg.cm_hdr.ch_dlc   = CAN_ERROR_DLC;
#endif
          
          /* This is a new bus-off event, include the cause of the "last straw"
           * error along with the bus-off indication error message to the app.
           * Clear the INIT bit if autorecovery is enabled.
           */
          if (cansts & TIVA_CAN_STS_BOFF
              && ! (canmod->status & TIVA_CAN_STS_BOFF))
            {
              if (canmod->autorecover)
                {
                  modreg32(0, TIVA_CAN_CTL_INIT,
                           canmod->base + TIVA_CAN_OFFSET_CTL);
                }
#ifdef CONFIG_CAN_ERRORS
              msg.cm_hdr.ch_id = CAN_ERROR_BUSOFF;
#endif
            }
#ifdef CONFIG_CAN_ERRORS

          /* If this an existing bus-off event and the module is recovering,
           * don't do anything. The Last Error Code will be reset but no error
           * message will be sent to the app. (All we have is a fake BIT0
           * error that indicates the CAN module is recovering.)
           */
          else if (cansts & TIVA_CAN_STS_BOFF
                    && canmod->status & TIVA_CAN_STS_BOFF)
            {
              ;
            }
          
          /* The module has just finished recovering from bus-off, indicate
           * this to the application. Skip checking the LEC because it will
           * just be a BIT0 error (since the CAN module sets this error for
           * every 11 sequential recessive bits seen during bus-off recovery.)
           */
          else if (! (cansts & TIVA_CAN_STS_BOFF)
                      && canmod->status & TIVA_CAN_STS_BOFF)
            {
              msg.cm_hdr.ch_id = CAN_ERROR_RESTARTED;
            }
          
          /* Rules for sending error messages to the application for error
           * counters: (1) Reaching the warning or passive level on their own
           * don't generate error messages, but the information will be
           * be included in the error message when an actual event has
           * occurred. (2) The application will be notified if the error
           * goes away by an error message of type CAN_ERROR_RESTARTED.
           */
          errcnt = (canerr & (TIVA_CAN_ERR_RP | TIVA_CAN_ERR_REC_MASK))
                    >> TIVA_CAN_ERR_REC_SHIFT;
          prev_errcnt = (canmod->errors
                        & (TIVA_CAN_ERR_RP | TIVA_CAN_ERR_REC_MASK))
                        >> TIVA_CAN_ERR_REC_SHIFT;
          
          if (errcnt >= 96)
            {
              msg.cm_hdr.ch_id |= CAN_ERROR_CONTROLLER;
              msg.cm_data[1] |= CAN_ERROR1_RXWARNING;
              
              if (errcnt >= 128)
                {
                  msg.cm_data[1] |= CAN_ERROR1_RXPASSIVE;
                }
              else if (prev_errcnt >= 128)
                {
                  msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
                }
            }
          else if (prev_errcnt >= 96)
            {
              msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
            }
          
          errcnt = (canerr & TIVA_CAN_ERR_TEC_MASK) >> TIVA_CAN_ERR_TEC_SHIFT;
          prev_errcnt = (canmod->errors & TIVA_CAN_ERR_TEC_MASK)
                        >> TIVA_CAN_ERR_TEC_SHIFT;
          
          if (errcnt >= 96)
            {
              msg.cm_hdr.ch_id |= CAN_ERROR_CONTROLLER;
              msg.cm_data[1] |= CAN_ERROR1_TXWARNING;
              
              if (errcnt >= 128)
                {
                  msg.cm_data[1] |= CAN_ERROR1_TXPASSIVE;
                }
              else if (prev_errcnt >= 128)
                {
                  msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
                }
            }
          else if (prev_errcnt >= 96)
            {
              msg.cm_hdr.ch_id |= CAN_ERROR_RESTARTED;
            }
          
          /* This is not a bus-off event, or it is a brand-new bus-off event
           * that we need to indicate the cause of the error for.
           */
          switch (canmod->status & TIVA_CAN_STS_LEC_MASK)
          {
          case TIVA_CAN_STS_LEC_NONE:
            {
              /* Don't reset the LEC or return an error message - this is a
               * normal condition and resetting it would just cause another
               * status interrupt upon successful message transmission or
               * receipt.
               */
            }
          break;
          case TIVA_CAN_STS_LEC_STUFF:
            {
              msg.cm_hdr.ch_id  |= CAN_ERROR_PROTOCOL;
              msg.cm_data[2]    = CAN_ERROR2_STUFF;
            }
          break;
          case TIVA_CAN_STS_LEC_FORM:
            {
              msg.cm_hdr.ch_id  |= CAN_ERROR_PROTOCOL;
              msg.cm_data[2]    = CAN_ERROR2_FORM;
            }
          break;
          case TIVA_CAN_STS_LEC_ACK:
            {
              msg.cm_hdr.ch_id  |= CAN_ERROR_NOACK;
            }
          break;
          case TIVA_CAN_STS_LEC_BIT1:
            {
              msg.cm_hdr.ch_id  |= CAN_ERROR_PROTOCOL;
              msg.cm_data[2]    = CAN_ERROR2_BIT | CAN_ERROR2_BIT1;
            }
          break;
          case TIVA_CAN_STS_LEC_BIT0:
            {
              /* Note: this error code is set every time there is a sequence
               * of 11 recessive bits during bus-off recovery, so ignore if
               * the bus-off event is "old news."
               */
              if (! (canmod->status & TIVA_CAN_STS_BOFF))
                {
                  msg.cm_hdr.ch_id |= CAN_ERROR_PROTOCOL;
                  msg.cm_data[2]   = CAN_ERROR2_BIT | CAN_ERROR2_BIT0;
                }
            }
          break;
          case TIVA_CAN_STS_LEC_CRC:
            {
              msg.cm_hdr.ch_id  |= CAN_ERROR_PROTOCOL;
              msg.cm_data[2]    = CAN_ERROR2_UNSPEC;
              msg.cm_data[3]    = CAN_ERROR3_CRCSEQ;
            }
          break;
          case TIVA_CAN_STS_LEC_NOEVENT:
            {
              /* This shouldn't happen, but it's probably not worth sending
               * an error message to the application about.
               */
              canerr("Status interrupt generated but no event occurred.\n");
            }
          break;
          default:
            {
              /* This shouldn't happen but it's probably not worth sending
               * an error message to the application about.
               */
              canerr("Invalid value for Last Error Code (LEC).\n");
            }
          }
          
          /* If there was a good reason to send an error message, send it */
          if (msg.cm_hdr.ch_id)
            {
              can_receive(dev, &msg.cm_hdr, msg.cm_data);
            }
          
          /* Reset the Last Error Code on abnormal events. During normal
           * operation, the application should be notified of every error
           * that occurs. For bus-off recovery, we want the Last Error Code
           * to be reset so that we get a meaningful result if an error occurs
           * immediately after the controller turns back on. When a status
           * interrupt is generated because the LEC switches to "no error"
           * this is bypassed with a direct return statement. This
           * is also bypassed when error messages are disabled (due to
           * the ifdef).
           */
          if ((cansts & TIVA_CAN_STS_LEC_MASK) != TIVA_CAN_STS_LEC_NONE
              && (cansts & TIVA_CAN_STS_LEC_MASK) != TIVA_CAN_STS_LEC_NOEVENT)
          {
            putreg32(cansts | TIVA_CAN_STS_LEC_NOEVENT,
                                  canmod->base + TIVA_CAN_OFFSET_STS);
          }
          
#endif /* CONFIG_CAN_ERRORS */
  
          /* Save contents of CANSTS and CANERR for other functions to use
           * and for comparison next time */
          canmod->status = cansts;
          canmod->errors = canerr;
          
          continue;
        } /* if canint == TIVA_CAN_INT_INTID_STATUS */
      
      /* TX interrupt */
      
      else if (canint == TIVA_CAN_TX_MBOX_NUM)
        {
          /* TODO: How should persistent remote responder messages be
           * configured? should such a thing be supported?
           */
          
          /* Note: the TM4C123GH6PM incorrectly states "A message interrupt is
           * cleared by clearing the message object's INTPND bit in the
           * CANIFnMCTL register or by reading the CAN Status (CANSTS)
           * register."
           * 
           * In fact, a message interrupt can only be cleared via the MCTL
           * register.
           */
          putreg32(TIVA_CANIF_CMSK_CLRINTPND,
                   canmod->isr_iface_base + TIVA_CANIF_OFFSET_CMSK);
          putreg32(TIVA_CAN_TX_MBOX_NUM,
                   canmod->isr_iface_base + TIVA_CANIF_OFFSET_CRQ);
          
          /* Wait for request to be processed */
          while (getreg32(canmod->isr_iface_base + TIVA_CANIF_OFFSET_CRQ)
                  & TIVA_CANIF_CRQ_BUSY);
          
          can_txdone(dev);
        }
      
      /* RX message interrupt */
        
      else if (canint < TIVA_CAN_TX_MBOX_NUM)
        {
          /* Process the received message(s). Since hardware RX FIFOS are used
           * and new messages are received into the mailbox with the lowest
           * number, we can't just keep reading CANINT since a new message
           * might arrive in a mailbox we'd already read from before the
           * higher-address mailboxes in the FIFO were emptied.
           * 
           * CANINT is off-by-one (mailboxes 1-32 instead of 0-31)
           */
          
          for (int i = canint - 1; i < TIVA_CAN_NUM_MSG_OBJS; ++i)
            {
              uint32_t reg;
              
              /* Check if there's an interrupt pending for this mailbox */
              if (! (tivacan_readsplitreg32(canmod->base,
                    TIVA_CAN_OFFSET_MSG1INT,
                    TIVA_CAN_OFFSET_MSG2INT) & 1 << i))
                {
                  continue;
                }
              
              /* Command Mask register
               * Pull arbitration bits, control bits, and data bits from the
               * message SRAM. Clear the interrupt and the new data bit.
               */
              reg = TIVA_CANIF_CMSK_ARB         | TIVA_CANIF_CMSK_CONTROL
                    | TIVA_CANIF_CMSK_CLRINTPND | TIVA_CANIF_CMSK_NEWDAT
                    | TIVA_CANIF_CMSK_DATAA     | TIVA_CANIF_CMSK_DATAB;
              putreg32(reg, canmod->isr_iface_base + TIVA_CANIF_OFFSET_CMSK);
              
              /* Submit request */
              putreg32(i + 1, canmod->isr_iface_base + TIVA_CANIF_OFFSET_CRQ);
              
              /* Wait for response */
              while (getreg32(canmod->isr_iface_base + TIVA_CANIF_OFFSET_CRQ)
                                                        & TIVA_CANIF_CRQ_BUSY);
              
              /* ARB2 - upper chunk of the message ID, direction, and extended
               * message indication.
               */
              reg = getreg32(canmod->isr_iface_base + TIVA_CANIF_OFFSET_ARB2);
              
              /* TODO: is this correct? On remote frames, is the DIR bit set
               * or the RMTEN bit?
               */
              msg.cm_hdr.ch_rtr = (0 != (reg & TIVA_CANIF_ARB2_DIR));
              
#ifdef CONFIG_CAN_EXTID
              msg.cm_hdr.ch_extid = (0 != (reg & TIVA_CANIF_ARB2_XTD));
              if (msg.cm_hdr.ch_extid)
                {
                  msg.cm_hdr.ch_id = (reg & TIVA_CANIF_ARB2_ID_EXT_MASK)
                                          << TIVA_CANIF_ARB2_ID_EXT_PRESHIFT;
                  
                  /* ARB1 - lower chunk of the message ID for extended IDs */
                  reg = getreg32(canmod->isr_iface_base
                                  + TIVA_CANIF_OFFSET_ARB1);
                  
                  msg.cm_hdr.ch_id |= reg & TIVA_CANIF_ARB1_ID_EXT_MASK;
                }
              else
                {
                  msg.cm_hdr.ch_id = (reg & TIVA_CANIF_ARB2_ID_STD_MASK)
                                            >> TIVA_CANIF_ARB2_ID_STD_SHIFT;
                }
#else
              msg.cm_hdr.ch_id = (reg & TIVA_CANIF_ARB2_ID_STD_MASK)
                                        >> TIVA_CANIF_ARB2_ID_STD_SHIFT;
#endif /* CONFIG_CAN_EXTID */
              
              /* Message Control - remote enable (RMTEN) & data length code */
              reg = getreg32(canmod->isr_iface_base + TIVA_CANIF_OFFSET_MCTL);
              msg.cm_hdr.ch_dlc = reg & TIVA_CANIF_MCTL_DLC_MASK;
              
              if (! (reg & TIVA_CANIF_MCTL_NEWDAT))
                {
                  canerr("ISR called on RX mailbox but no new data!\n");
                }
              
              /* Data registers - these are little endian but the uint8_t
               * array in can_msg_s is big endian.
               */
              for (int j = 0; j < (msg.cm_hdr.ch_dlc + 1) / 2; ++j)
                {
                  reg = getreg32(canmod->isr_iface_base
                                  + TIVA_CANIF_OFFSET_DATA(j));
                  
                  msg.cm_data[j * 2]      = (reg & TIVA_CANIF_DATA_HBYTE_MASK)
                                            >> TIVA_CANIF_DATA_HBYTE_SHIFT;
                  msg.cm_data[j * 2 + 1]  = (reg & TIVA_CANIF_DATA_LBYTE_MASK)
                                            >> TIVA_CANIF_DATA_LBYTE_SHIFT;
                }
              
              can_receive(dev, &msg.cm_hdr, msg.cm_data);
            }
        }
      else
        {
          /* The interrupt handler was called, but there is no interrupt...! */
          canerr("ISR called but no interrupt given!\n");
        }
    } /* while CANINT reg != 0*/
    
    return OK;
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
 *   dev - An instance of the "upper half" CAN driver structure
 *   timing - An instance of tiva_can_timing_s, with the prescaler, TSEG1,
 *            TSEG2, and SJW set to their real (not off-by-one) values.
 * 
 * Returned value: None
 ****************************************************************************/

static void tivacan_bittiming_set(FAR struct can_dev_s *dev,
                                  FAR struct tiva_can_timing_s *timing)
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
              timing->tseg2 < TIVA_CAN_TSEG2_MAX);
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
 * Return value: The index of a tiva_can_fifo_s in dev->cd_priv (tiva_canmod_s),
 *               which contains a bit-field of the message objects that may be
 *               used, -ENOSPC if there is not enough room in the CAN module
 *               SRAM (i.e. too many FIFOs already allocated), or -ENOANO if
 *               the limit CONFIG_TIVA_CAN_FILTERS_MAX has been reached.
 ****************************************************************************/

int tivacan_alloc_fifo(FAR struct can_dev_s *dev, int depth)
{
  /* Mailbox 1 (aka 0) is always claimed for TX */
  uint32_t claimed = TIVA_CAN_TX_MBOX_MASK;
  int i;
  int numclaimed = 0;
  int free_fifo_idx = -1;
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  
  nxmutex_lock(&canmod->fifo_mtx);
  
  /* Message objects allocated other RX FIFOs */
  
  for (i = 0; i < CONFIG_TIVA_CAN_FILTERS_MAX; ++i)
    {
      if (canmod->fifos[i].msg_nums == 0)
        {
          free_fifo_idx = i;
        }
      
      claimed |= canmod->fifos[i].msg_nums;
    }
  
  if (free_fifo_idx < 0)
  {
    canwarn("Max number of filters allocated.\n");
    return -ENOANO;
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
      return -ENOSPC;
    }
  else
    {
      /* Clear the bits for message objects we don't need to claim right now.
       * NOTE: If a new chip came along with more than 32 message objects,
       *       this would need to be reworked.
       */
      claimed &= 0xffffffff >> (32 - i);
      canmod->fifos[free_fifo_idx].msg_nums = claimed;
      
      nxmutex_unlock(&canmod->fifo_mtx);
      return free_fifo_idx;
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
  nxmutex_lock(&canmod->thd_iface_mtx);
  
  for (int i = 0; i < TIVA_CAN_NUM_MSG_OBJS; ++i)
    {
      if (fifo->msg_nums & 1 << i)
        {
          /* Wait for pending transmisions or reads to complete */
          
          while ((tivacan_readsplitreg32(canmod->base,
                                          TIVA_CAN_OFFSET_TXRQ1,
                                          TIVA_CAN_OFFSET_TXRQ2)
                | tivacan_readsplitreg32(canmod->base,
                                          TIVA_CAN_OFFSET_NWDA1,
                                          TIVA_CAN_OFFSET_NWDA2))
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
 *   checking whether there's a pending message in the object! It takes the
 *   raw base interface register base address, so it can be called from thread
 *   or ISR context.
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
  
  /* Signal a write (WRNRD) to the message SRAM touching the ARB registers */
  putreg32(TIVA_CANIF_CMSK_WRNRD | TIVA_CANIF_CMSK_ARB,
           iface_base + TIVA_CANIF_OFFSET_CMSK);
  
  /* Mark message object as invalid */
  putreg32(0, iface_base + TIVA_CANIF_OFFSET_ARB2);
  
  /* Make sure the interface is not busy */
  while (getreg32(iface_base + TIVA_CANIF_OFFSET_CRQ) & TIVA_CANIF_CRQ_BUSY);
  
  /* Submit request. Message objects are 1-indexed in hardware */
  putreg32(num + 1, iface_base + TIVA_CANIF_OFFSET_CRQ);
}

/****************************************************************************
 * Name: tivacan_initfilter
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
 *   dev    - An instance of the "upper-half" CAN driver structure
 *   fifo   - A FIFO in the message SRAM obtained by calling tivacan_alloc_fifo
 *   filter - A canioc_stdfilter_s or canioc_extfilter_s
 *            of type CAN_FILTER_MASK
 *   type   - One of CAN_TIVA_FILTER_TYPE_STD or CAN_TIVA_FILTER_TYPE_EXT
 * 
 * Return value:
 ****************************************************************************/

static int  tivacan_initfilter(FAR struct can_dev_s       *dev,
                               FAR struct tiva_can_fifo_s *fifo,
                               FAR void                   *filter,
                               int                         type)
{
  uint32_t  reg;
  uint32_t  msgval;  /* Message valid bitfield */
  uint32_t  newdat;  /* New data bitfield (for RX message objects) */
  
  /* Whether the End Of Buffer bit has been set - this is required by the Tiva
   * hardware for some reason */
  bool      eob_set     = false;
  
  FAR struct tiva_canmod_s *canmod = dev->cd_priv;
  uint32_t  canbase     = canmod->base;
  uint32_t  iface_base  = canmod->thd_iface_base;
  
  FAR struct canioc_stdfilter_s *sfilter = NULL;
#ifdef CONFIG_CAN_EXTID
  FAR struct canioc_extfilter_s *xfilter = NULL;
#endif
  
  if (type == CAN_TIVA_FILTER_TYPE_STD)
    {
      sfilter = filter;
    }
#ifdef CONFIG_CAN_EXTID
  else if (type == CAN_TIVA_FILTER_TYPE_EXT)
    {
      xfilter = filter;
    }
#endif
  else
    {
      canerr("Invalid filter type parameter.\n");
      return -EINVAL;
    }
  
  for (int i = TIVA_CAN_NUM_MSG_OBJS - 1; i >= 0; --i)
    {
      if (fifo->msg_nums & (1 << i))
        {
          int j = 0;
          
          do
            {
              msgval = tivacan_readsplitreg32(canbase, TIVA_CAN_OFFSET_MSG1VAL,
                                                       TIVA_CAN_OFFSET_MSG2VAL);
              newdat = tivacan_readsplitreg32(canbase, TIVA_CAN_OFFSET_NWDA1,
                                                       TIVA_CAN_OFFSET_NWDA2);
              ++j;
              
              if (msgval & newdat & 1 <<i)
                {
                  usleep(10);
                }
            }
          /* Wait in case the ISR hasn't had a chance to read the message object
           * we're trying to overwrite. 5 iterations is arbitrary. */
          while (msgval & newdat & 1 << i && j < 5);
          
          /* Command Mask Register
           * 
           * Write to message SRAM and access mask, control, and arbitration
           * bits. Clear any interrupts (which might be spurious due to
           * the hardware being uninitialized
           */
          reg =   TIVA_CANIF_CMSK_WRNRD     | TIVA_CANIF_CMSK_MASK
                | TIVA_CANIF_CMSK_CONTROL   | TIVA_CANIF_CMSK_ARB
                | TIVA_CANIF_CMSK_CLRINTPND;
          putreg32(reg, iface_base + TIVA_CANIF_OFFSET_CMSK);
          
          /* Mask 1 Register - lower 16 bits of extended ID mask */
#ifdef CONFIG_CAN_EXTID
          if (type == CAN_TIVA_FILTER_TYPE_EXT)
            {
              reg = xfilter->xf_id2 & TIVA_CANIF_MSK1_IDMSK_EXT_MASK;
              putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MSK1);
            }
#endif
          
          /* Mask 2 Register
           * Allow remote request frames through. This is done by disabling
           * filter-on-direction (MDIR = 0)
           */
#ifdef CONFIG_CAN_EXTID
          if (type == CAN_TIVA_FILTER_TYPE_EXT)
            {
              reg = (xfilter->xf_id2 >> TIVA_CANIF_MSK2_IDMSK_EXT_PRESHIFT)
                    & TIVA_CANIF_MSK2_IDMSK_EXT_MASK;
            }
          else
            {
#endif
              /* Filter out extended ID messages by default with standard
               * filters.
               */
              reg = TIVA_CANIF_MSK2_MXTD;
              reg |= sfilter->sf_id2 << TIVA_CANIF_MSK2_IDMSK_STD_SHIFT;
#ifdef CONFIG_CAN_EXTID
            }
#endif
          
          putreg32(reg, iface_base + TIVA_CANIF_OFFSET_MSK2);
          
          /* Arbitration 1 Register - lower 16 bits of extended ID */
#ifdef CONFIG_CAN_EXTID
          if (type == CAN_TIVA_FILTER_TYPE_EXT)
            {
              reg = xfilter->xf_id1 & TIVA_CANIF_ARB1_ID_EXT_MASK;
              putreg32(reg, iface_base + TIVA_CANIF_OFFSET_ARB1);
            }
#endif
          
          /* Arbitration 2 Register - upper extended & all standard ID bits
           * Also mark the message object as valid.
           */
          reg = TIVA_CANIF_ARB2_MSGVAL;
          
#ifdef CONFIG_CAN_EXTID
          if (type == CAN_TIVA_FILTER_TYPE_EXT)
            {
              reg |= xfilter->xf_id1 & TIVA_CANIF_ARB2_ID_EXT_MASK;
            }
          else
            {
#endif
              /* Leave the XTD bit clear to indicate that only standard messages
               * are allowed through the filter (MXTD is set).
               */
              reg |= sfilter->sf_id1 << TIVA_CANIF_ARB2_ID_STD_SHIFT;
#ifdef CONFIG_CAN_EXTID
            }
#endif
          
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
           * for the transfer to complete. Message objects are 1-indexed in
           * hardware
           */
          putreg32(i + 1, iface_base + TIVA_CANIF_OFFSET_CRQ);
          while (getreg32(iface_base + TIVA_CANIF_OFFSET_CRQ)
                                        & TIVA_CANIF_CRQ_BUSY);
        }
    }
  
  nxmutex_unlock(&canmod->thd_iface_mtx);
  
  return OK;
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
  FAR struct tiva_canmod_s *canmod;
  int ret;
  caninfo("tiva_can_initialize module %d\n", modnum);
  
#ifdef CONFIG_TIVA_CAN0
  if (modnum == 0)
    {
      dev = &g_tivacan0dev;
    }
#else
  if (modnum == 0)
    {
      return -ENODEV;
    }
#endif

#ifdef CONFIG_TIVA_CAN1
  if (modnum == 1)
    {
      dev = &g_tivacan1dev;
    }
#else
  if (modnum == 1)
    {
      return -ENODEV;
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
}

#endif /* CONFIG_TIVA_CAN && (CONFIG_TIVA_CAN0 || CONFIG_TIVA_CAN1) */
