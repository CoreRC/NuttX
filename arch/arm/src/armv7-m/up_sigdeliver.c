/****************************************************************************
 * arch/arm/src/armv7-m/up_sigdeliver.c
 *
 *   Copyright (C) 2009-2010, 2013-2016, 2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "sched/sched.h"
#include "up_internal.h"
#include "up_arch.h"

#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void up_sigdeliver(void)
{
  struct tcb_s  *rtcb = this_task();
  uint32_t regs[XCPTCONTEXT_REGS];
  sig_deliver_t sigdeliver;

  /* Save the errno.  This must be preserved throughout the signal handling
   * so that the user code final gets the correct errno value (probably
   * EINTR).
   */

  int saved_errno = rtcb->pterrno;

#ifdef CONFIG_SMP
  /* In the SMP case, we must terminate the critical section while the signal
   * handler executes, but we also need to restore the irqcount when the
   * we resume the main thread of the task.
   */

  int16_t saved_irqcount;
#endif

  board_autoled_on(LED_SIGNAL);

  sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
        rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
  ASSERT(rtcb->xcp.sigdeliver != NULL);

  /* Save the real return state on the stack. */

  up_copyfullstate(regs, rtcb->xcp.regs);
  regs[REG_PC]         = rtcb->xcp.saved_pc;
#ifdef CONFIG_ARMV7M_USEBASEPRI
  regs[REG_BASEPRI]    = rtcb->xcp.saved_basepri;
#else
  regs[REG_PRIMASK]    = rtcb->xcp.saved_primask;
#endif
  regs[REG_XPSR]       = rtcb->xcp.saved_xpsr;
#ifdef CONFIG_BUILD_PROTECTED
  regs[REG_LR]         = rtcb->xcp.saved_lr;
#endif

  /* Get a local copy of the sigdeliver function pointer. We do this so that
   * we can nullify the sigdeliver function pointer in the TCB and accept
   * more signal deliveries while processing the current pending signals.
   */

  sigdeliver           = (sig_deliver_t)rtcb->xcp.sigdeliver;
  rtcb->xcp.sigdeliver = NULL;

#ifdef CONFIG_SMP
  /* In the SMP case, up_schedule_sigaction(0) will have incremented
   * 'irqcount' in order to force us into a critical section.  Save the
   * pre-incremented irqcount.
   */

  saved_irqcount       = rtcb->irqcount - 1;
  DEBUGASSERT(saved_irqcount >= 0);

  /* Now we need call leave_critical_section() repeatedly to get the irqcount
   * to zero, freeing all global spinlocks that enforce the critical section.
   */

  do
    {
#ifdef CONFIG_ARMV7M_USEBASEPRI
      leave_critical_section((uint8_t)regs[REG_BASEPRI]);
#else
      leave_critical_section((uint16_t)regs[REG_PRIMASK]);
#endif
    }
  while (rtcb->irqcount > 0);
#endif /* CONFIG_SMP */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Then make sure that interrupts are enabled.  Signal handlers must always
   * run with interrupts enabled.
   */

  up_irq_enable();
#endif

  /* Deliver the signal */

  sigdeliver(rtcb);

  /* Output any debug messages BEFORE restoring errno (because they may
   * alter errno), then disable interrupts again and restore the original
   * errno that is needed by the user logic (it is probably EINTR).
   *
   * REVISIT: In SMP mode up_irq_save() probably only disables interrupts
   * on the local CPU.  We do not want to call enter_critical_section()
   * here, however, because we don't want this state to stick after the
   * call to up_fullcontextrestore().
   *
   * I would prefer that all interrupts are disabled when
   * up_fullcontextrestore() is called, but that may not be necessary.
   */

  sinfo("Resuming\n");

  (void)up_irq_save();

  /* Restore the saved errno value */

  rtcb->pterrno = saved_errno;

#ifdef CONFIG_SMP
  /* Restore the saved 'irqcount' and recover the critical section
   * spinlocks.
   */

  DEBUGASSERT(rtcb->irqcount == 0);
  while (rtcb->irqcount < saved_irqcount)
    {
      (void)enter_critical_section();
    }
#endif

  /* Then restore the correct state for this thread of
   * execution.
   */

  board_autoled_off(LED_SIGNAL);
  up_fullcontextrestore(regs);
}

#endif /* !CONFIG_DISABLE_SIGNALS */
