#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"

uint64
sys_exit(void)
{
  int n;
  argint(0, &n);
  exit(n);
  return 0;  // not reached
}

uint64
sys_getpid(void)
{
  return myproc()->pid;
}

uint64
sys_fork(void)
{
  return fork();
}

uint64
sys_wait(void)
{
  uint64 p;
  argaddr(0, &p);
  return wait(p);
}

uint64
sys_sbrk(void)
{
  uint64 addr;
  int n;

  argint(0, &n);
  addr = myproc()->sz;
  if(growproc(n) < 0)
    return -1;
  return addr;
}

uint64
sys_sleep(void)
{
  int n;
  uint ticks0;

  backtrace();

  argint(0, &n);
  if(n < 0)
    n = 0;
  acquire(&tickslock);
  ticks0 = ticks;
  while(ticks - ticks0 < n){
    if(killed(myproc())){
      release(&tickslock);
      return -1;
    }
    sleep(&ticks, &tickslock);
  }
  release(&tickslock);
  return 0;
}

uint64
sys_kill(void)
{
  int pid;

  argint(0, &pid);
  return kill(pid);
}

// return how many clock tick interrupts have occurred
// since start.
uint64
sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}

// sigalarm
uint64
sys_sigalarm(void)
{
  struct proc *proc = myproc();
  argint(0, &proc->alarm_interval);
  if (proc->alarm_interval < 0) {
    printf("sigalarm failed, alarm interval: %d\n",proc->alarm_interval);	  
    return -1; 
  }
  argaddr(1, &proc->alarm_handler);
  if (proc->alarm_handler < 0) {
    printf("sigalarm failed!\n");	  
    return -1;
  }
  return 0;
}

// sigreturn
uint64
sys_sigreturn(void)
{
  struct proc* proc = myproc();
  *proc->trapframe = proc->saved_trapframe;
  proc->alarm_cnt = 0;
  return proc->trapframe->a0;
}
