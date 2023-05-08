#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"
#include "file.h"
#include "fcntl.h"

#define BSIZE 1024

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

uint64
sys_mmap(void)
{
  uint64 len;
  int prot, flags, fd;
  // assume addr and offset = 0
  struct file *f;
  argaddr(1, &len);
  argint(2, &prot);
  argint(3, &flags);

  //argfd(4, &fd, &f);
  // ***
  argint(4, &fd);
  if(fd < 0 || fd >= NOFILE || (f=myproc()->ofile[fd]) == 0)
    return -1;
  // ***

  // if file is read-only, but map it as writable
  if(!f->writable && (prot & PROT_WRITE) && (flags & MAP_SHARED ))
  {
     return -1;
  }

  struct proc *p = myproc();
  for(uint i=0; i<MAXVMA; i++)
  { 
    struct VMA *v = &p->vma[i];   
    if(!v->used) //find an unsed vma
    {
      v->used = 1;
      v->addr = p->sz;
      len = PGROUNDUP(len);
      p->sz += len;
      v->len = len;
      v->prot = prot;
      v->flags = flags;
      v->f = filedup(f);
      v->start_point = 0;
      return v->addr;
    }
  }

  return -1;
}

int
file_write_new(struct file *f, uint64 addr, int n, uint off)
{
  int r=0;
  if (f->writable == 0) return -1;

  int max = ((MAXOPBLOCKS - 1 - 1 - 2) / 2) * BSIZE, i = 0;
  while (i < n)
  {
    int n1 = n-i;
    if (n1 > max) n1 = max;

    begin_op();
    ilock(f->ip);
    if((r = writei(f->ip , 1, addr + i, off, n1)) > 0)
        off += r;
    iunlock(f->ip);
    end_op();

    if(r != n1)  break;
    i += r;
  }

  return 0;
}

uint64
sys_munmap(void)
{
  uint64 addr;
  int len;
  int close=0;
  argaddr(0, &addr);
  argint(1, &len);

  struct proc* p=myproc();
  for(uint i=0; i<MAXVMA; i++)
  {
    struct VMA *v = &p->vma[i];
    //only unmap at start,end or the whole region
    if(v->used && addr >= v->addr && addr <= v->addr + v->len)
    {
      uint64 npages = 0;
      uint off = v->start_point;
      if(addr == v->addr) // unmap at start
      {
        if(len >= v->len) //unmap whole region
        {
          len = v->len;
          v->used = 0;
          close = 1;
        }
        else //unmap from start but not whole region
        {
          v->addr += len;
          v->start_point += len;//update start point at which to map
        }
      }

      len = PGROUNDUP(len);
      npages = len/PGSIZE; 
      v->len -= len;
      p->sz -= len;

      if(v->flags & MAP_SHARED) // need to write back pages
      {
        file_write_new(v->f, addr, len, off);
      }   
      
      uvmunmap(p->pagetable, PGROUNDDOWN(addr), npages, 0);
      if(close) fileclose(v->f);

      return 0;
    }
  }

  return -1;
}
