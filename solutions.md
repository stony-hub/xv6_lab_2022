# XV6 Lab 实验报告

[TOC]

# Lab Utilities

配置好环境后，运行结果如图所示。

![](0_0.png)

同时按照官方文档中的要求完成了几个xv6系统中运行的程序，并通过`make grade`进行测试，结果如图所示。

![](0_grade.png)

下面介绍各个lab的详细情况。以下所有lab都通过了`make grade`测试并满分。

# Lab System calls

本lab需要完成trace和sysinfo两个部分。首先修改Makefile，加入`$U/_trace`一项，同时在系统调用相关部分新增关于trace和sysinfo两个系统调用的声明
```c
// user/user.h
int trace(int);
int sysinfo(struct sysinfo*);

// user/usys.pl
entry("trace");
entry("sysinfo");

// kernel/syscall.h
#define SYS_trace  22
#define SYS_sysinfo 23

// kernel/syscall.c
// An array mapping syscall numbers from syscall.h
// to the function that handles the system call.
static uint64 (*syscalls[])(void) = {
...
[SYS_trace]   sys_trace,
[SYS_sysinfo] sys_sysinfo,
};

// syscall names
static char* syscall_name[] = {
...
[SYS_trace]   "trace",
[SYS_sysinfo] "sysinfo",
};
```

## System call tracing

对于trace系统调用，在结构体proc中增加变量来记录trace mask，并在`kernel/sysproc.c`中定义系统调用`uint64 sys\_trace(void)`。修改fork()保证子进程继承了父进程的mask，最后修改syscall来输出系统调用信息。
```c
// kernel/syscall.c

void
syscall(void)
{
  int num;
  struct proc *p = myproc();

  num = p->trapframe->a7;
  if(num > 0 && num < NELEM(syscalls) && syscalls[num]) {
    // Use num to lookup the system call function for num, call it,
    // and store its return value in p->trapframe->a0
    p->trapframe->a0 = syscalls[num]();
    // syscall trace
    if (p->trace_mask & (1<<num))
      printf("%d: syscall %s -> %d\n", p->pid, syscall_name[num], p->trapframe->a0);
  } else {
    printf("%d %s: unknown sys call %d\n",
            p->pid, p->name, num);
    p->trapframe->a0 = -1;
  }
}
```

## Sysinfo

对于sysinfo系统调用，需要定义两个函数分别统计当前可用的进程和内存数量。
```c
// kernel/proc.c
// get number of system process
int
get_proc_num(void)
{
  struct proc* p;
  int n = 0;
  for (p = proc; p < &proc[NPROC]; p++) {
    if (p->state !=UNUSED)
      n++;
  }
  return n;
}

// kernel/kalloc.c
// get free mem
uint64
get_free_mem(void)
{
  struct run* r = kmem.freelist;
  uint64 res = 0;
  while (r) {
    res++;
    r = r->next;
  }
  return res * PGSIZE;
}
```
并在`kernel/sysproc.c`中定义系统调用`uint64 sys\_sysinfo(void)`，之后在`sys\_sysinfo`调用这两个函数完成sysinfo结构体的填写。

## 测试

这样就完成了system call lab的内容，最后通过`make grade`进行测试，结果如图所示。

![](1_grade.png)

# Lab Page Tables

本lab需要处理xv6的页表机制，分为三个任务。

## Speed up system calls

首先需要实现一个用户态共享内存机制。由于`getpid`函数需要从用户态进入内核态访问内核态空间，效率较低，因此xv6提供了一个`ugetpid`函数，可以直接从用户态访问对应内存。用户态虚拟内存布局如下所示，其中`USYSCALL`部分专门用来存放pid信息。
```c
// kernel/memlayout.h

// User memory layout.
// Address zero first:
//   text
//   original data and bss
//   fixed-size stack
//   expandable heap
//   ...
//   USYSCALL (shared with kernel)
//   TRAPFRAME (p->trapframe, used by the trampoline)
//   TRAMPOLINE (the same page as in the kernel)
#define TRAPFRAME (TRAMPOLINE - PGSIZE)
#ifdef LAB_PGTBL
#define USYSCALL (TRAPFRAME - PGSIZE)

struct usyscall {
  int pid;  // Process ID
};
#endif
```
在初始化进程时，需要为这段虚拟内存分配一个物理空间，并在生成页表时建立对应关系。这里所在页的权限应设置为`PTE_R | PTE_U`，保证用户可以访问。
```c
// kernel/proc.c

static struct proc*
allocproc(void)
{
  ...

  // Allocate usyscall page
    if ((p->usyscallpage = (struct usyscall *)kalloc()) == 0) {
    freeproc(p);
    release(&p->lock);
    return 0;
  }
  p->usyscallpage->pid = p->pid;

  // An empty user page table.
  p->pagetable = proc_pagetable(p);

  ...
}

pagetable_t
proc_pagetable(struct proc *p)
{
  pagetable_t pagetable;

  ...

  // write usyscall page
  if(mappages(pagetable, USYSCALL, PGSIZE,
              (uint64)(p->usyscallpage), PTE_R | PTE_U) < 0){
    uvmunmap(pagetable, TRAMPOLINE, 1, 0);
    uvmunmap(pagetable, TRAPFRAME, 1, 0);
    uvmfree(pagetable, 0);
    return 0;
  }

  return pagetable;
}
```
同时在释放进程时要释放这一部分空间
```c
// kernel/proc.c

static void
freeproc(struct proc *p)
{
  ...

  if(p->pagetable)
    proc_freepagetable(p->pagetable, p->sz);
  p->pagetable = 0;
  // release usyscall page
  if(p->usyscallpage)
    kfree((void *)p->usyscallpage);
  p->usyscallpage = 0;
  
  ...
}

void
proc_freepagetable(pagetable_t pagetable, uint64 sz)
{
  uvmunmap(pagetable, TRAMPOLINE, 1, 0);
  uvmunmap(pagetable, TRAPFRAME, 1, 0);
  uvmunmap(pagetable, USYSCALL, 1, 0);
  uvmfree(pagetable, sz);
}
```

## Print a page table

这个任务比较简单，递归实现即可
```c
// kernel/vm.c
static int printdeep = 0;
void
vmprint(pagetable_t pagetable)
{
  if (printdeep == 0)
    printf("page table %p\n", (uint64)pagetable);
  for (int i = 0; i < 512; i++) {
    pte_t pte = pagetable[i];
    if (pte & PTE_V) {
      for (int j = 0; j <= printdeep; j++) {
        printf("..");
      }
      printf("%d: pte %p pa %p\n", i, (uint64)pte, (uint64)PTE2PA(pte));
    }
    // pintes to lower-level page table
    if((pte & PTE_V) && (pte & (PTE_R|PTE_W|PTE_X)) == 0){
      printdeep++;
      uint64 child_pa = PTE2PA(pte);
      vmprint((pagetable_t)child_pa);
      printdeep--;
    }
  }
}
```

## Detect which pages have been accessed

首先按照提示，添加`PTE_A`标志位。该标识是由硬件写入的。
```c
// kernel/riscv.h

#define PTE_A (1L << 6)
```
接下来实现`pgaccess`系统调用。首先初始化存储标识信息的bitmap，这里用一个uint64变量`maskbits`作为bitmap。遍历传入的页，使用walk函数找到对应的PTE，如果`PTE_A`存在，则存回bitmap中，再清除PTE表中的`PTE_A`位，最后使用`copyout`写回即可。
```c
// kernel/sysproc.c

int
sys_pgaccess(void)
{
  // lab pgtbl: your code here.
  uint64 va;
  int pagenum;
  uint64 abitsaddr;
  argaddr(0, &va);
  argint(1, &pagenum);
  argaddr(2, &abitsaddr);

  uint64 maskbits = 0;
  struct proc *proc = myproc();
  for (int i = 0; i < pagenum; i++) {
    pte_t *pte = walk(proc->pagetable, va+i*PGSIZE, 0);
    if (pte == 0)
      panic("page not exist.");
    if (PTE_FLAGS(*pte) & PTE_A) {
      maskbits = maskbits | (1L << i);
    }
    // clear PTE_A, set PTE_A bits zero
    *pte = ((*pte&PTE_A) ^ *pte) ^ 0 ;
  }
  if (copyout(proc->pagetable, abitsaddr, (char *)&maskbits, sizeof(maskbits)) < 0)
    panic("sys_pgacess copyout error");

  return 0;
}
```

## 测试

这样就完成了页表lab的内容，最后通过`make grade`进行测试，结果如图所示。

![](2_grade.png)

# Lab Traps

首先在本lab初始的部分有介绍RISC-V汇编的内容，可以通过阅读相关代码了解RISC-V汇编。在xv6系统中中断/异常机制和汇编代码有着重要的关系。xv6允许用户调用的系统调用定义在`user/user.h`中，而在`user/usys.pl`中通过汇编代码将相应的系统调用号填入`a7`寄存器中，并通过`ecall`指令陷入内核态。相应系统调用函数的实际定义在`kernel/syscall.c`中，可以从寄存器中读取系统调用的参数。

## Backtrace

接下来完成backtrace任务，该任务需要实现一个backtrace函数输出当前函数栈帧。用变量fp_address保存RISC-V中fp寄存器的内容，由于xv6地址为64位，当前栈帧的地址为`fp_address - 8`，上一层栈帧的地址为`fp_address - 16`，当fp_address遍历到当前内存页的页首时代表已经遍历了栈帧。代码实现如下，同时在`sys_sleep`中调用该函数即可。
```c
// kernel/printf.c
void
backtrace(void)
{
  uint64 fp_address = r_fp();
  printf("backtrace:\n");
  while(fp_address != PGROUNDDOWN(fp_address)) {
    printf("%p\n", *(uint64*)(fp_address-8));
    fp_address = *(uint64*)(fp_address - 16);
  }
}
```

## Alarm

接下来完成alarm任务。首先添加sigalarm和sigreturn两个系统调用，过程略。在`kernel/proc.h`的pcb中添加alarm tick的有关信息
```c
// kernel/proc.h
// Per-process state
struct proc {
  ...
  // alarm
  int alarm_interval;
  int alarm_cnt;
  uint64 alarm_handler;
  struct trapframe saved_trapframe;
};
```
在`kernel/sysproc.c`中实现sigalarm和sigreturn两个系统调用如下
```c
// kernel/sysproc.c
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
```
其中sigalarm将alarm的间隔和handler信息填入pcb中，sigreturn在返回时还原栈帧。而实际陷入alarm的情况在`trap.c`中处理
```c
// kernel/trap.c
void
usertrap(void)
{
  ...

  if(killed(p))
    exit(-1);

  // give up the CPU if this is a timer interrupt.
  if(which_dev == 2) {
    struct proc *proc = myproc();
    // alarm_interval == 0 indicates that alarm func is forbbiden
    if (proc->alarm_interval != 0) {
      if (++proc->alarm_cnt == proc->alarm_interval) {
        proc->saved_trapframe = *p->trapframe;
        proc->trapframe->epc = proc->alarm_handler;
      }
    }
    yield();
  }

  usertrapret();
}
```
即通过判断alarm tick是否达到了设置的间隔将进程切换到alarm handler中。

## 测试

这样就完成了traps lab的内容，最后通过`make grade`进行测试，结果如图所示。

![](3_grade.png)

# Lab Copy on-write

本lab需要在xv6上实现写时复制(Copy-on-write)机制。在复制用户态虚拟内存页后不需要立刻实际复制物理内存，而是在对内存进行写操作时再进行复制。
首先定义`PTE_C`标识位表示页面写时复制，并且定义数组`page_ref`存储页面引用计数。
```c
// kernel/riscv.h

// 8-9 is reserved PTE digits
#define PTE_C (1L << 8) // copy-on-write

// page ref index
#define PA_INDEX(pa) (((uint64)(pa) - KERNBASE) >> 12)

// kernel/kalloc.c

// page ref cnt
uint page_ref[(PHYSTOP - KERNBASE) / PGSIZE];
struct spinlock page_ref_lock;
```
在`kalloc`时，将其初始化为1
```c
// kernel/kalloc.c

void *
kalloc(void)
{
  struct run *r;

  acquire(&kmem.lock);
  r = kmem.freelist;
  if(r)
    kmem.freelist = r->next;
  release(&kmem.lock);

  if(r) {
    memset((char*)r, 5, PGSIZE); // fill with junk
    acquire(&page_ref_lock);
    page_ref[PA_INDEX(r)] = 1;
    release(&page_ref_lock);
  }
  return (void*)r;
}
```
在`kfree`时，只有引用计数降为0时才释放页面
```c
// kernel/kalloc.c

void
kfree(void *pa)
{
  ...
  
  // Check page ref
  acquire(&page_ref_lock);
  if(page_ref[PA_INDEX(pa)] > 1){
    page_ref[PA_INDEX(pa)]--;
    release(&page_ref_lock);
    return;
  }
  page_ref[PA_INDEX(pa)] = 0;
  release(&page_ref_lock);

  ...
}
```
在进程`fork`时，内核会调用`uvmcopy`函数复制用户态内存，因此需要修改`uvmcopy`函数，将新页表中的地址映射到原来的物理页面中，其引用计数加1，并将权限修改为不可写(`~PTE_W`)，写时复制(`PTE_C`)，这样在写这个页面时就会通过`trap`处理
```c
// kernel/vm.c

extern uint page_ref[];

int
uvmcopy(pagetable_t old, pagetable_t new, uint64 sz)
{
  pte_t *pte;
  uint64 pa, i;
  uint flags;

  for(i = 0; i < sz; i += PGSIZE){
    if((pte = walk(old, i, 0)) == 0)
      panic("uvmcopy: pte should exist");
    if((*pte & PTE_V) == 0)
      panic("uvmcopy: page not present");
    pa = PTE2PA(*pte);

    // set copy-on-write PTE
    *pte = (*pte & ~PTE_W) | PTE_C;
    flags = PTE_FLAGS(*pte);
    if(mappages(new, i, PGSIZE, pa, flags) != 0){
      goto err;
    }
    acquire(&page_ref_lock);
    page_ref[PA_INDEX(pa)]++;
    release(&page_ref_lock);
  }
  return 0;

 err:
  uvmunmap(new, 0, i / PGSIZE, 1);
  return -1;
}
```
`usertrap`对写时复制的处理如下
```c
// kernel/trap.c

void
usertrap(void)
{
  ...
  
  if(r_scause() == 8){
    ...
  // Store / AMO page fault
  } else if(r_scause() == 15){
    uint64 va = r_stval();
    if(va >= p->sz)
      p->killed = 1;
    else if(kalloc_cow(p->pagetable, va) != 0)
      p->killed = 1;
  } else if((which_dev = devintr()) != 0){
    // ok
  } else {
    ...
  }

  ...
}

// kernel/kalloc.c

// copy-on-write alloc
int
kalloc_cow(pagetable_t pagetable, uint64 va)
{
  va = PGROUNDDOWN(va);
  if(va >= MAXVA || va == 0) return -1;
  pte_t *pte = walk(pagetable, va, 0);
  if(pte == 0) return -1;
  uint64 pa = PTE2PA(*pte);
  if(pa == 0) return -1;
  uint64 flags = PTE_FLAGS(*pte);
  if(flags & PTE_C) {
    uint64 mem = (uint64)kalloc();
    if (mem == 0) return -1;
    memmove((char*)mem, (char*)pa, PGSIZE);
    uvmunmap(pagetable, va, 1, 1);
    //*pte = PA2PTE(mem) | flags;
    if (mappages(pagetable, va, PGSIZE, mem, (flags & ~PTE_C) | PTE_W) != 0) {
      kfree((void*)mem);
      return -1;
    }
  }
  return 0;
}
```
最后完成`copyout`函数，需要处理内核页面被共享的情况，这里可以直接调用上面写的`kalloc_cow`函数来分配用户态内存
```c
// kernel/vm.c

int
copyout(pagetable_t pagetable, uint64 dstva, char *src, uint64 len)
{
  uint64 n, va0, pa0;
  pte_t *pte;

  while(len > 0){
    va0 = PGROUNDDOWN(dstva);
    if (kalloc_cow(pagetable, va0) != 0)
      return -1;
    pa0 = walkaddr(pagetable, va0);
    if(pa0 == 0)
      return -1;
    n = PGSIZE - (dstva - va0);
    if(n > len)
      n = len;
    pte = walk(pagetable, va0, 0);
    if(pte == 0)
      return -1;
    memmove((void *)(pa0 + (dstva - va0)), src, n);

    len -= n;
    src += n;
    dstva = va0 + PGSIZE;
  }
  return 0;
}
```
这样就完成了`Copy-on-write`的功能。

## 测试

本lab通过`make grade`测试结果如下

![](4_grade.png)

# Lab Multithreading

本lab需要探究xv6中的多线程机制，并编写两个实际操作系统(Linux)中的多线程并发问题。

## Uthread: switching between threads 

首先需要在xv6中实现一个用户多线程机制。在`uthread.c`中仿照`proc.h`中的形式定义栈帧，并添加到用户thread数据结构中。同时按照提示补充thread_schedule和thread_create两个函数的内容。
```c
// user/uthread.c

struct ucontext {
  uint64 ra;
  uint64 sp;

  // callee-saved
  uint64 s0;
  uint64 s1;
  uint64 s2;
  uint64 s3;
  uint64 s4;
  uint64 s5;
  uint64 s6;
  uint64 s7;
  uint64 s8;
  uint64 s9;
  uint64 s10;
  uint64 s11;
};

struct thread {
  char       stack[STACK_SIZE]; /* the thread's stack */
  int        state;             /* FREE, RUNNING, RUNNABLE */
  struct ucontext context;      /* Uthread Context*/
};

void 
thread_schedule(void)
{
  ...

  if (current_thread != next_thread) {         /* switch threads?  */
    next_thread->state = RUNNING;
    t = current_thread;
    current_thread = next_thread;
    /* YOUR CODE HERE
     * Invoke thread_switch to switch from t to next_thread:
     * thread_switch(??, ??);
     */
    thread_switch((uint64)&t->context, (uint64)&next_thread->context);
  } else
    next_thread = 0;
}

void 
thread_create(void (*func)())
{
  struct thread *t;

  for (t = all_thread; t < all_thread + MAX_THREAD; t++) {
    if (t->state == FREE) break;
  }
  t->state = RUNNABLE;
  // YOUR CODE HERE
  t->context.ra=(uint64)func;
  t->context.sp=(uint64)(t->stack+STACK_SIZE);
}
```
这里thread_switch函数的定义在`uthread_switch.S`中，为汇编代码，需要完成上下文保存和切换的功能，代码如下
```c
/* user/uthread_switch.S */
	.text

	/*
         * save the old thread's registers,
         * restore the new thread's registers.
         */

	.globl thread_switch
thread_switch:
	/* YOUR CODE HERE */
	
    sd ra, 0(a0)
    sd sp, 8(a0)
    sd s0, 16(a0)
    sd s1, 24(a0)
    sd s2, 32(a0)
    sd s3, 40(a0)
    sd s4, 48(a0)
    sd s5, 56(a0)
    sd s6, 64(a0)
    sd s7, 72(a0)
    sd s8, 80(a0)
    sd s9, 88(a0)
    sd s10, 96(a0)
    sd s11, 104(a0)

    ld ra, 0(a1)
    ld sp, 8(a1)
    ld s0, 16(a1)
    ld s1, 24(a1)
    ld s2, 32(a1)
    ld s3, 40(a1)
    ld s4, 48(a1)
    ld s5, 56(a1)
    ld s6, 64(a1)
    ld s7, 72(a1)
    ld s8, 80(a1)
    ld s9, 88(a1)
    ld s10, 96(a1)
    ld s11, 104(a1)

	ret    /* return to ra */

```

## Using threads

后两个问题不用直接在linux(或其他满足POSIX协议标准的系统)中完成。任务ph需要实现细粒度的锁，由于不同hash块之间不会产生冲突，可以为每一个hash块设置一个单独的锁。代码如下
```c
// notxv6/ph.c

pthread_mutex_t lock[NBUCKET];

static 
void put(int key, int value)
{
  int i = key % NBUCKET;

  pthread_mutex_lock(&lock[i]);
  ...
  pthread_mutex_unlock(&lock[i]);

}

static struct entry*
get(int key)
{
  int i = key % NBUCKET;

  pthread_mutex_lock(&lock[i]);
  ...
  pthread_mutex_unlock(&lock[i]);

  return e;
}

int
main(int argc, char *argv[])
{
  for (int i=0; i<NBUCKET; i++) {
    pthread_mutex_init(&lock[i], NULL);
  }
  ...
}
```

## Barrier

任务barrier需要实现barrier机制，这里采用经典的信号量机制，代码如下
```c
// notxv6/barrier.c

static void 
barrier()
{
  pthread_mutex_lock(&bstate.barrier_mutex);
  if (++bstate.nthread == nthread) {
    bstate.round++;
    bstate.nthread = 0;
    pthread_cond_broadcast(&bstate.barrier_cond);
  } else {
    pthread_cond_wait(&bstate.barrier_cond, &bstate.barrier_mutex);
  }
  pthread_mutex_unlock(&bstate.barrier_mutex);
}
```

## 测试

本lab通过`make grade`测试结果如下

![](5_grade.png)

# Lab network driver

Networking lab需要为xv6系统编写网络驱动器，用于e1000模拟网络设备。驱动器代码的大部分已经完成，需要在其中填写`e1000_transmit`和`e1000_recv`两个部分，按照e1000手册填写相关内容即可。
```c
// kernel/e1000.c

int
e1000_transmit(struct mbuf *m)
{
  //
  // Your code here.
  //
  // the mbuf contains an ethernet frame; program it into
  // the TX descriptor ring so that the e1000 sends it. Stash
  // a pointer so that it can be freed after sending.
  //

  int i;

  acquire(&e1000_lock);

  i = regs[E1000_TDT];

  if (!(tx_ring[i].status & E1000_TXD_STAT_DD)) {
    release(&e1000_lock);
    return -1;
  }
  if (tx_mbufs[i])
    mbuffree(tx_mbufs[i]);
  tx_mbufs[i] = m;
  tx_ring[i].addr = (uint64)m->head;
  tx_ring[i].length = m->len;
  tx_ring[i].cmd = E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS;

  regs[E1000_TDT] = (i + 1) % TX_RING_SIZE;

  release(&e1000_lock);
  
  return 0;
}

static void
e1000_recv(void)
{
  //
  // Your code here.
  //
  // Check for packets that have arrived from the e1000
  // Create and deliver an mbuf for each packet (using net_rx()).
  //

  int i;

  //acquire(&e1000_lock);

  i = (regs[E1000_RDT] + 1) % RX_RING_SIZE;

  while (rx_ring[i].status & E1000_RXD_STAT_DD){
    rx_mbufs[i]->len = rx_ring[i].length;
    net_rx(rx_mbufs[i]);
    rx_mbufs[i] = mbufalloc(0);
    rx_ring[i].addr = (uint64)rx_mbufs[i]->head;
    rx_ring[i].status = 0;
    i = (i + 1) % RX_RING_SIZE;
  }

  regs[E1000_RDT] = (i + RX_RING_SIZE - 1) % RX_RING_SIZE;

  //release(&e1000_lock);
  
}
```

## 测试

本lab通过`make grade`测试结果如下

![](6_grade.png)

# Lab Lock

本lab需要使用锁机制处理内存分配问题。

## Memory allocator

原xv6只有一个单独的空闲内存链表，在有多个CPU同时运行的情况下可能会发生阻塞，效率低下。第一个任务需要为每一个CPU实现一个空闲内存链表。首先将原有的链表`kmem`改为数组
```c
// kernel/kalloc.c

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem[NCPU];

void
kinit()
{
  for (int i=0; i<NCPU; i++) {
    initlock(&kmem[i].lock, "kmem");
  }
  // initlock(&kmem.lock, "kmem");
  freerange(end, (void*)PHYSTOP);
}
```
在分配内存时从当前CPU的链表中取出一个block。当空闲内存链表为空时，可以从别的可用CPU空闲内存中搬运一个过来。同时在free内存的时候也做相应处理。
```c
// kernel/kalloc.c

void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  r = (struct run*)pa;

  push_off();

  int i = cpuid();

  acquire(&kmem[i].lock);
  r->next = kmem[i].freelist;
  kmem[i].freelist = r;
  release(&kmem[i].lock);

  pop_off();
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *
kalloc(void)
{
  struct run *r;

  push_off();

  int i = cpuid();

  acquire(&kmem[i].lock);
  r = kmem[i].freelist;
  if(r){
    kmem[i].freelist = r->next;
    release(&kmem[i].lock);
  } else {
    // try to stole from other cpu's freelist
    for (int j=0; j<NCPU; j++) {
      if (i == j) continue;

      acquire(&kmem[j].lock);
      if (kmem[j].freelist){
        r = kmem[j].freelist;
        kmem[j].freelist = r->next;
        
        release(&kmem[j].lock);
        break;
      }
      release(&kmem[j].lock);
    }
    release(&kmem[i].lock);
  }

  pop_off();

  if(r)
    memset((char*)r, 5, PGSIZE); // fill with junk
  return (void*)r;
}
```

## Buffer cache

对于第二个任务，需要通过细粒度的锁实现buffer cache。原有的设计只有一个大锁，很容易造成阻塞。这里将原有的bcache分为若干个hash块，块的个数按照提示设置为13。同时在`buf`中添加时间戳变量用于后面实现基于时间戳的LRU算法。初始化buffer的过程也要相应修改。
```c
// kernel/buf.h

#define NBUCKET 13

struct buf {
  uint tickstamp;
  ...
};

// kernel/bio.c

struct {
  struct spinlock lock;
  struct buf list;
} hash[NBUCKET];

void
binit(void)
{
  struct buf *b;

  for (int i=0; i<NBUCKET; i++) {
    initlock(&hash[i].lock, "bcache");
    hash[i].list.next = 0;
  }

  // Create linked list of buffers
  for(b = bcache.buf; b < bcache.buf+NBUF; b++){
    initsleeplock(&b->lock, "buffer");
    b->blockno = 0;
    b->next = hash[0].list.next;
    hash[0].list.next = b;
  }
}
```
buffer的获取通过时间戳LRU算法实现。buffer的释放改为释放到hash块。同时`bpin`和`bunpin`两个辅助函数也相应修改。
```c
// kernel/bio.c

static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;

  int bucket = blockno % NBUCKET;
  acquire(&hash[bucket].lock);

  // Is the block already cached?
  for (b = hash[bucket].list.next; b != 0; b = b->next){
    if (b->dev == dev && b->blockno == blockno) {
      b->refcnt++;
      release(&hash[bucket].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached.
  // Recycle the least recently used (LRU) unused buffer.
  int i = bucket;
    do {
        if (i != bucket)    acquire(&hash[i].lock);

        struct buf *minibuf = 0;
        uint mini = 0xffffffff;
        for (b = hash[i].list.next; b != 0; b = b->next) {
            if (b->refcnt == 0 && mini > b->tickstamp) {
                mini = b->tickstamp;
                minibuf = b;
            }
        }

        if (minibuf != 0)
            return subbget(minibuf, dev, blockno);
        
        if (i != bucket)    release(&hash[i].lock);
        i = (i + 1) % NBUCKET;
    } while (i != bucket);
  panic("bget: no buffers");
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  int bucket = b->blockno % NBUCKET;
  acquire(&hash[bucket].lock);

  b->refcnt--;
  if (b->refcnt == 0)
    b->tickstamp = ticks;
  
  release(&hash[bucket].lock);
}

void
bpin(struct buf *b) {
  int bucket = b->blockno % NBUCKET;
  acquire(&hash[bucket].lock);
  b->refcnt++;
  release(&hash[bucket].lock);
}

void
bunpin(struct buf *b) {
  int bucket = b->blockno % NBUCKET;
  acquire(&hash[bucket].lock);
  b->refcnt--;
  release(&hash[bucket].lock);
}
```
这样就完成了第二个任务。

## 测试

本lab通过`make grade`测试结果如下

![](7_grade.png)

# Lab File system

本lab扩展xv6的文件系统，需要完成扩大文件大小上限以及实现符号链接两个任务。

## Large files

首先实现大文件。原先xv6文件inode只有一级索引，将其扩展到二级索引就可以完成任务。首先将inode一级索引中的一项修改为存放二级索引
```c
// kernel/file.h

// in-memory copy of an inode
struct inode {
  ...
  uint addrs[NDIRECT+2];
};

// kernel/fs.h

#define NDIRECT 11 // remove 1 from NDIRECT for 2 ji suo yi
#define NINDIRECT (BSIZE / sizeof(uint))
#define NINDIRECT2 (NINDIRECT * NINDIRECT)
#define MAXFILE (NDIRECT + NINDIRECT + NINDIRECT2)

// On-disk inode structure
struct dinode {
  short type;           // File type
  short major;          // Major device number (T_DEVICE only)
  short minor;          // Minor device number (T_DEVICE only)
  short nlink;          // Number of links to inode in file system
  uint size;            // Size of file (bytes)
  uint addrs[NDIRECT+2];   // Data block addresses
};
```
在`bmap`函数中将原有的一级索引改为二级索引
```c
// kernel/fs.c

static uint
bmap(struct inode *ip, uint bn)
{
  uint addr, *a;
  struct buf *bp;

  if(bn < NDIRECT){
    if((addr = ip->addrs[bn]) == 0){
      addr = balloc(ip->dev);
      if(addr == 0)
        return 0;
      ip->addrs[bn] = addr;
    }
    return addr;
  }
  bn -= NDIRECT;

  if(bn < NINDIRECT){
    // Load indirect block, allocating if necessary.
    if((addr = ip->addrs[NDIRECT]) == 0){
      addr = balloc(ip->dev);
      if(addr == 0)
        return 0;
      ip->addrs[NDIRECT] = addr;
    }
    bp = bread(ip->dev, addr);
    a = (uint*)bp->data;
    if((addr = a[bn]) == 0){
      addr = balloc(ip->dev);
      if(addr){
        a[bn] = addr;
        log_write(bp);
      }
    }
    brelse(bp);
    return addr;
  }
  bn -= NINDIRECT;

  // level 2 index
  if(bn < NINDIRECT2){
    if((addr = ip->addrs[NDIRECT+1]) == 0){
      addr = balloc(ip->dev);
      if(addr == 0)
        return 0;
      ip->addrs[NDIRECT+1] = addr;
    }

    // level 2
    bp = bread(ip->dev, addr);
    a = (uint*)bp->data;
    if((addr = a[bn/NINDIRECT]) == 0){
      addr = balloc(ip->dev);
      if(addr){
        a[bn/NINDIRECT] = addr;
        log_write(bp);
      }
    }
    brelse(bp);

    // level 1
    bp = bread(ip->dev, addr);
    a = (uint*)bp->data;
    if((addr = a[bn%NINDIRECT]) == 0){
      addr = balloc(ip->dev);
      if(addr){
        a[bn%NINDIRECT] = addr;
        log_write(bp);
      }
    }
    brelse(bp);

    return addr;
  }

  panic("bmap: out of range");
}
```
原有的释放索引也加上释放二级索引的过程
```c
// kernel/fs.c

void
itrunc(struct inode *ip)
{
  int i, j, k;
  struct buf *bp, *bp2;
  uint *a, *a2;

  for(i = 0; i < NDIRECT; i++){
    if(ip->addrs[i]){
      bfree(ip->dev, ip->addrs[i]);
      ip->addrs[i] = 0;
    }
  }

  if(ip->addrs[NDIRECT]){
    bp = bread(ip->dev, ip->addrs[NDIRECT]);
    a = (uint*)bp->data;
    for(j = 0; j < NINDIRECT; j++){
      if(a[j])
        bfree(ip->dev, a[j]);
    }
    brelse(bp);
    bfree(ip->dev, ip->addrs[NDIRECT]);
    ip->addrs[NDIRECT] = 0;
  }

  // level 2
  if(ip->addrs[NDIRECT+1]){
    bp = bread(ip->dev, ip->addrs[NDIRECT+1]);
    a = (uint*)bp->data;
    for(j = 0; j < NINDIRECT; j++){
      if(a[j]){
        bp2 = bread(ip->dev, a[j]);
        a2 = (uint*)bp2->data;
        for(k = 0; k < NINDIRECT; k++){
          if(a2[k])
            bfree(ip->dev, a2[k]);
        }
        brelse(bp2);
        bfree(ip->dev, a[j]);
        a[j] = 0;
      }
    }
    brelse(bp);
    bfree(ip->dev, ip->addrs[NDIRECT+1]);
    ip->addrs[NDIRECT+1] = 0;
  }

  ip->size = 0;
  iupdate(ip);
}
```

## Symbolic links

接下来实现软链接。首先添加`symlink`系统调用，过程略。在`fcntl.h`中增加一项文件控制选项，注意需要使用保留位且不能和别的选项冲突。同时定义软链接搜索层数上限，避免出现链接环的时候陷入死循环。此外定义新的文件类型`T_SYMLINK`。
```c
// kernel/fcntl.h

#define O_NOFOLLOW 0x004

// kernel/param.h

#define MAXDEPTH     10    // maximum symbolic link search depth

// kernel/stat.h

#define T_SYMLINK 4   // Symbolic links
```
软链接的创建过程如下，具体方法为建立一个新的inode并填入目标路径的信息。
```c
// kernel/sysfile.c

uint64
sys_symlink(void)
{  
  char path[MAXPATH], target[MAXPATH];
  struct inode *ip;
  if(argstr(0, target, MAXPATH) < 0)
    return -1;
  if(argstr(1, path, MAXPATH) < 0)
    return -1;

  begin_op();
  if((ip = create(path, T_SYMLINK, 0, 0)) == 0){
    end_op();
    return -1;
  }
  if(writei(ip, 0, (uint64)target, 0, MAXPATH) < MAXPATH) {
    iunlockput(ip);
    end_op();
    return -1;
  }
  iunlockput(ip);
  end_op();

  return 0;
}
```
在`open`系统调用中，增加一项对软链接的处理。软链接inode以`T_SYNLINK`标识。如果当前inode为软链接就一直循环处理，直到找到非软链接或超过层数上限为止。
```c
// kernel/sysfile.c

uint64
sys_open(void)
{
  ...

  if(omode & O_CREATE){
    ...
  } else {
    if((ip = namei(path)) == 0){
      end_op();
      return -1;
    }
    ilock(ip);
    int depth = 0;
    while(ip->type == T_SYMLINK && !(omode & O_NOFOLLOW)){
       if(depth++ >= MAXDEPTH){
         iunlockput(ip);
         end_op();
         return -1;
       }
       if(readi(ip, 0, (uint64)path, 0, MAXPATH) < MAXPATH){
         iunlockput(ip);
         end_op();
         return -1;
       }
       iunlockput(ip);
       if((ip = namei(path)) == 0) {
         end_op();
         return -1;
       }
       ilock(ip);
     }

    if(ip->type == T_DIR && omode != O_RDONLY){
      iunlockput(ip);
      end_op();
      return -1;
    }
  }

  ...
}
```
这样就实现了软链接的功能。

## 测试

本lab通过`make grade`测试结果如下

![](8_grade.png)

# Lab mmap

本lab在xv6中实现类似Linux的mmap机制。通过man命令查看`mmap`和`munmap`两个函数的格式如下，这两个函数分别将文件的一部分映射到内存中，以及取消映射关系。
```c
void *mmap(void *addr, size_t length, int prot, int flags,
           int fd, off_t offset);
int munmap(void *addr, size_t length);
```
这里首先添加`mmap`和`munmap`两个系统调用，过程略。下面介绍实现过程。
内存映射机制需要从内存中抽取可用的一部分用于映射。这里应该通过VMA来进行内存管理。在本lab中可以简单使用数组结构的VMA实现内存管理，不需要考虑VMA访问效率等问题。在`proc.h`中定义VMA结构如下，并在pcb中添加VMA。
```c
// kernel/proc.h

#define MAXVMA 16
struct VMA {
  int used;           // if this vma is used
  uint64 addr;        // address
  uint64 len;         // length
  int prot;           // permissions 
  int flags;          // flags
  struct file *f;     // the file being mapped
  uint64 start_point; // starting piont in the file at which to map
};

// Per-process state
struct proc {
  ...
  struct VMA vma[MAXVMA];
};
```
这样`mmap`只需要找到可用的VMA并填写内容即可。对于`munmap`也是进行类似操作。
```c
// kernel/sysproc.c

uint64
sys_mmap(void)
{
   int len;
   int prot,flags,fd;
   struct file *f;
   if(argint(1, &len) < 0 ||  argint(2, &prot) < 0 || argint(3, &flags) < 0  || argfd(4, &fd,&f) < 0  )
    return -1;

   // if file is read-only,but map it as writable.return fail
   if(!f->writable && (prot & PROT_WRITE) && (flags & MAP_SHARED ) )
   {
      return -1;
   }

   struct proc* p=myproc();
   for(uint i=0;i<MAXVMA;i++)
   { 
      struct VMA *v=&p->vma[i]	;   
      if(!v->used) //find an unsed vma
      {
         // store relative auguments
	    v->used=1;
        v->addr=p->sz;//use p->sz to p->sz+len to map the file
	    len= PGROUNDUP(len);// map的最小单位是PGSIZE
	    p->sz+=len;
	    v->len=len;
	    v->prot=prot;
	    v->flags=flags;
	    v->f= filedup(f);//increase the file's ref cnt
	    v->start_point=0;//staring point in f to map is 0
	    return v->addr;
      }
   }
   
   return -1;
}
```
在分配新进程的时候，需要初始化VMA
```c
// kernel/proc.c

static struct proc*
allocproc(void)
{
  ...

  for(int i=0; i<MAXVMA; i++){
    struct VMA *v = &p->vma[i];
    memset(v, 0, sizeof(struct VMA)); 
  }
  return p;
}
```

在发生缺页异常的时候，需要在VMA中找到缺页位置对应的部分并实际分配内存。需要注意将内存越界的情况排除掉，这种情况与mmap无关。
```c
// kernel/trap.c

void
usertrap(void)
{
  ...
  
  if(r_scause() == 8){
    ...
  } else if((which_dev = devintr()) != 0){
    // ok
  } else if(r_scause() == 13 || r_scause() == 15) { // page fault
    uint64 va = r_stval();

    if (va >= p->sz || va < p->trapframe->sp) goto fail;

    for(int i=0; i<MAXVMA; i++)
    {
      struct VMA *v = &p->vma[i];
      if(v->used && va >= v->addr && va < v->addr + v->len)
      {
        char * mem;
        mem = kalloc();
        memset(mem, 0, PGSIZE);
        va = PGROUNDDOWN(va);
        uint64 offset = v->start_point + va - v->addr;
        // PROT_READ=1 PROT_WRITE=2 PROT_EXEC=4
        // PTE_R=2     PTE_W=4      PTE_X=8
        // vma[i]->prot left shift
        mappages(p->pagetable, va, PGSIZE, (uint64)mem, (v->prot<<1)|PTE_U);
        ilock(v->f->ip);
        readi(v->f->ip, 1, va, offset, PGSIZE);
        iunlock(v->f->ip);
        break;
      }
    }
  } else {
    fail:
    printf("usertrap(): unexpected scause %p pid=%d\n", r_scause(), p->pid);
    printf("            sepc=%p stval=%p\n", r_sepc(), r_stval());
    setkilled(p);
  }

  ...
}
```

接下来需要处理`fork`和`exit`时候的情况。当进行`fork`操作时，需要将内存映射的部分复制过去。当进行`exit`操作时，需要清除掉内存映射的部分。
```c
// kernel/proc.c

int
fork(void)
{
  ...

  for(int i=0; i<MAXVMA; i++){
    struct VMA *v = &p->vma[i];
    struct VMA *nv = &np->vma[i];
    //only unmap at start,end or the whole region
    if(v->used){
      memmove(nv, v, sizeof(struct VMA)); 
      filedup(nv->f);
    }
  }

  ...
}

void
exit(int status)
{
  ...

  for(int i=0; i<MAXVMA; i++){
    struct VMA *v = &p->vma[i];
    if(v->used){
      uvmunmap(p->pagetable, v->addr, v->len / PGSIZE, 0);
      memset(v, 0, sizeof(struct VMA)); 
    }
  }

  ...
}
```

最后，由于是lazy allocation，所以在`uvmunmap`和`uvmcopy`中`p->sz`范围之内的虚拟地址可能并没有全部映射到`p->pagetable`中，所以不再判断这一情况
```c
// kernel/vm.c

void
uvmunmap(pagetable_t pagetable, uint64 va, uint64 npages, int do_free)
{
  ...

  for(a = va; a < va + npages*PGSIZE; a += PGSIZE){
    ...
    if((*pte & PTE_V) == 0){
      return;
      // panic("uvmunmap: not mapped");
    }
    ...
  }
}

int
uvmcopy(pagetable_t old, pagetable_t new, uint64 sz)
{
  ...

  for(i = 0; i < sz; i += PGSIZE){
    ...
    if((*pte & PTE_V) == 0){
      continue;
      //panic("uvmcopy: page not present");
    }
    ...
  }
  return 0;

  ...
}
```

这样就完成了内存映射机制。

## 测试

本lab通过`make grade`测试结果如下

![](9_grade.png)

# 参考资料

[XV6官方手册](https://pdos.csail.mit.edu/6.S081/2022/tools.html)
https://zhuanlan.zhihu.com/p/429304672
https://zhuanlan.zhihu.com/p/442455606
https://horbyn.github.io/2022/04/15/xv6-8
https://zhuanlan.zhihu.com/p/451462274
https://zhuanlan.zhihu.com/p/342082201
