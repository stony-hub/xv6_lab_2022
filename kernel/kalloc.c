// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

// page ref cnt
uint page_ref[(PHYSTOP - KERNBASE) / PGSIZE];
struct spinlock page_ref_lock;

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem;

void
kinit()
{
  initlock(&kmem.lock, "kmem");
  freerange(end, (void*)PHYSTOP);
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE)
    kfree(p);
}

// Free the page of physical memory pointed at by pa,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");
  
  // Check page ref
  // wrong
  //if (--page_ref[PA_INDEX(pa)] > 0) return;
  acquire(&page_ref_lock);
  if(page_ref[PA_INDEX(pa)] > 1){
    page_ref[PA_INDEX(pa)]--;
    release(&page_ref_lock);
    return;
  }
  page_ref[PA_INDEX(pa)] = 0;
  release(&page_ref_lock);

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  r = (struct run*)pa;

  acquire(&kmem.lock);
  r->next = kmem.freelist;
  kmem.freelist = r;
  release(&kmem.lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
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

// copy-on-write alloc
int
kalloc_cow(pagetable_t pagetable, uint64 va)
{
  va = PGROUNDDOWN(va);
  if(va >= MAXVA) return -1;
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
