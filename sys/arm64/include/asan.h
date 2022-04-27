#ifndef _MACHINE_ASAN_H_
#define	_MACHINE_ASAN_H_

#ifdef KASAN

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>
#include <machine/vmparam.h>

static inline vm_offset_t
kasan_md_addr_to_shad(vm_offset_t addr)
{
	return (((addr - VM_MIN_KERNEL_ADDRESS) >> KASAN_SHADOW_SCALE_SHIFT) +
	    KASAN_MIN_ADDRESS);
}

static inline bool
kasan_md_unsupported(vm_offset_t addr)
{
	return(addr < VM_MIN_KERNEL_ADDRESS || addr >= virtual_end);
        //return (true);
}

static inline void
kasan_md_init(void)
{

}

static inline void
kasan_md_init_early(vm_offset_t bootstack, size_t size)
{
	/* TODO */
}

#endif /* KASAN */
#endif /* !_MACHINE_ASAN_H_ */
