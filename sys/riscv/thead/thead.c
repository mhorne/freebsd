#include <sys/param.h>
#include <sys/systm.h>

#include <machine/cpufunc.h>
#include <machine/machdep.h>

__attribute((target("arch=+xtheadcmo,+xtheadsync")))
void
thead_cpu_dcache_wbinv_range(vm_offset_t va, vm_size_t len)
{
	vm_offset_t end = va + len;

	va = rounddown(va, dcache_line_size);
	//printf("%s: va=%#lx, len=%lu\n", __func__, va, len);
	for (; va < end; va += dcache_line_size) {
		__asm __volatile("th.dcache.civa %0\n"
		                 "th.sync.s\n"
		                 :: "r" (va) : "memory");
	}
}

__attribute((target("arch=+xtheadcmo,+xtheadsync")))
void
thead_cpu_dcache_inv_range(vm_offset_t va, vm_size_t len)
{
	vm_offset_t end = va + len;

	va = rounddown(va, dcache_line_size);
	//printf("%s: va=%#lx, len=%lu\n", __func__, va, len);
	for (; va < end; va += dcache_line_size) {

		__asm __volatile("th.dcache.iva %0\n"
		                 "th.sync.s\n"
		                 :: "r" (va) : "memory");
	}
}

__attribute((target("arch=+xtheadcmo,+xtheadsync")))
void
thead_cpu_dcache_wb_range(vm_offset_t va, vm_size_t len)
{
	vm_offset_t end = va + len;

	va = rounddown(va, dcache_line_size);
	//printf("%s: va=%#lx, len=%lu\n", __func__, va, len);
	for (; va < end; va += dcache_line_size) {

		__asm __volatile("th.dcache.cva %0\n"
		                 "th.sync.s\n"
		                 :: "r" (va) : "memory");
	}
}
