/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1998-2003 Hewlett-Packard Co
 *	David Mosberger-Tang <davidm@hpl.hp.com>
 *	Stephane Eranian <eranian@hpl.hp.com>
 * Copyright (C) 2000, Rohit Seth <rohit.seth@intel.com>
 * Copyright (C) 1999 VA Linux Systems
 * Copyright (C) 1999 Walt Drummond <drummond@valinux.com>
 * Copyright (C) 2003 Silicon Graphics, Inc. All rights reserved.
 *
 * Routines used by ia64 machines with contiguous (or virtually contiguous)
 * memory.
 */
#include <linux/bootmem.h>
#include <linux/efi.h>
#include <linux/mm.h>
#include <linux/swap.h>

#include <asm/meminit.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/sections.h>
#include <asm/mca.h>

#ifdef CONFIG_VIRTUAL_MEM_MAP
static unsigned long max_gap;
#endif

/**
 * show_mem - display a memory statistics summary
 *
 * Just walks the pages in the system and describes where they're allocated.
 */
void
show_mem (void)
{
	int i, total = 0, reserved = 0;
	int shared = 0, cached = 0;

	printk(KERN_INFO "Mem-info:\n");
	show_free_areas();

	printk(KERN_INFO "Free swap:       %6ldkB\n",
	       nr_swap_pages<<(PAGE_SHIFT-10));
	i = max_mapnr;
	for (i = 0; i < max_mapnr; i++) {
		if (!pfn_valid(i)) {
#ifdef CONFIG_VIRTUAL_MEM_MAP
			if (max_gap < LARGE_GAP)
				continue;
			i = vmemmap_find_next_valid_pfn(0, i) - 1;
#endif
			continue;
		}
		total++;
		if (PageReserved(mem_map+i))
			reserved++;
		else if (PageSwapCache(mem_map+i))
			cached++;
		else if (page_count(mem_map + i))
			shared += page_count(mem_map + i) - 1;
	}
	printk(KERN_INFO "%d pages of RAM\n", total);
	printk(KERN_INFO "%d reserved pages\n", reserved);
	printk(KERN_INFO "%d pages shared\n", shared);
	printk(KERN_INFO "%d pages swap cached\n", cached);
	printk(KERN_INFO "%ld pages in page table cache\n",
	       pgtable_quicklist_total_size());
}

/* physical address where the bootmem map is located */
unsigned long bootmap_start;

/**
 * find_max_pfn - adjust the maximum page number callback
 * @start: start of range
 * @end: end of range
 * @arg: address of pointer to global max_pfn variable
 *
 * Passed as a callback function to efi_memmap_walk() to determine the highest
 * available page frame number in the system.
 */
int
find_max_pfn (unsigned long start, unsigned long end, void *arg)
{
	unsigned long *max_pfnp = arg, pfn;

	pfn = (PAGE_ALIGN(end - 1) - PAGE_OFFSET) >> PAGE_SHIFT;
	if (pfn > *max_pfnp)
		*max_pfnp = pfn;
	return 0;
}

/**
 * find_bootmap_location - callback to find a memory area for the bootmap
 * @start: start of region
 * @end: end of region
 * @arg: unused callback data
 *
 * Find a place to put the bootmap and return its starting address in
 * bootmap_start.  This address must be page-aligned.
 */
static int __init
find_bootmap_location (unsigned long start, unsigned long end, void *arg)
{
	unsigned long needed = *(unsigned long *)arg;
	unsigned long range_start, range_end, free_start;
	int i;

#if IGNORE_PFN0
	if (start == PAGE_OFFSET) {
		start += PAGE_SIZE;
		if (start >= end)
			return 0;
	}
#endif

	free_start = PAGE_OFFSET;

	for (i = 0; i < num_rsvd_regions; i++) {
		range_start = max(start, free_start);
		range_end   = min(end, rsvd_region[i].start & PAGE_MASK);

		free_start = PAGE_ALIGN(rsvd_region[i].end);

		if (range_end <= range_start)
			continue; /* skip over empty range */

		if (range_end - range_start >= needed) {
			bootmap_start = __pa(range_start);
			return -1;	/* done */
		}

		/* nothing more available in this segment */
		if (range_end == end)
			return 0;
	}
	return 0;
}

/**
 * find_memory - setup memory map
 *
 * Walk the EFI memory map and find usable memory for the system, taking
 * into account reserved areas.
 */
void __init
find_memory (void)
{
	unsigned long bootmap_size;

	reserve_memory();

	/* first find highest page frame number */
	max_pfn = 0;
	efi_memmap_walk(find_max_pfn, &max_pfn);

	/* how many bytes to cover all the pages */
	bootmap_size = bootmem_bootmap_pages(max_pfn) << PAGE_SHIFT;

	/* look for a location to hold the bootmap */
	bootmap_start = ~0UL;
	efi_memmap_walk(find_bootmap_location, &bootmap_size);
	if (bootmap_start == ~0UL)
		panic("Cannot find %ld bytes for bootmap\n", bootmap_size);

	bootmap_size = init_bootmem(bootmap_start >> PAGE_SHIFT, max_pfn);

	/* Free all available memory, then mark bootmem-map as being in use. */
	efi_memmap_walk(filter_rsvd_memory, free_bootmem);
	reserve_bootmem(bootmap_start, bootmap_size);

	find_initrd();

#ifdef CONFIG_CRASH_DUMP
	/* If we are doing a crash dump, we still need to know the real mem
	 * size before original memory map is * reset. */
	saved_max_pfn = max_pfn;
#endif
}

#ifdef CONFIG_SMP
/**
 * per_cpu_init - setup per-cpu variables
 *
 * Allocate and setup per-cpu data areas.
 */
void * __cpuinit
per_cpu_init (void)
{
	void *cpu_data;
	int cpu;
	static int first_time=1;

	/*
	 * get_free_pages() cannot be used before cpu_init() done.  BSP
	 * allocates "NR_CPUS" pages for all CPUs to avoid that AP calls
	 * get_zeroed_page().
	 */
	if (first_time) {
		first_time=0;
		cpu_data = __alloc_bootmem(PERCPU_PAGE_SIZE * NR_CPUS,
					   PERCPU_PAGE_SIZE, __pa(MAX_DMA_ADDRESS));
		for (cpu = 0; cpu < NR_CPUS; cpu++) {
			memcpy(cpu_data, __phys_per_cpu_start, __per_cpu_end - __per_cpu_start);
			__per_cpu_offset[cpu] = (char *) cpu_data - __per_cpu_start;
			cpu_data += PERCPU_PAGE_SIZE;
			per_cpu(local_per_cpu_offset, cpu) = __per_cpu_offset[cpu];
		}
	}
	return __per_cpu_start + __per_cpu_offset[smp_processor_id()];
}
#endif /* CONFIG_SMP */

static int
count_pages (u64 start, u64 end, void *arg)
{
	unsigned long *count = arg;

	*count += (end - start) >> PAGE_SHIFT;
	return 0;
}

/*
 * Set up the page tables.
 */

void __init
paging_init (void)
{
	unsigned long max_dma;
	unsigned long max_zone_pfns[MAX_NR_ZONES];

	num_physpages = 0;
	efi_memmap_walk(count_pages, &num_physpages);

	max_dma = virt_to_phys((void *) MAX_DMA_ADDRESS) >> PAGE_SHIFT;
	memset(max_zone_pfns, 0, sizeof(max_zone_pfns));
	max_zone_pfns[ZONE_DMA] = max_dma;
	max_zone_pfns[ZONE_NORMAL] = max_low_pfn;

#ifdef CONFIG_VIRTUAL_MEM_MAP
	efi_memmap_walk(register_active_ranges, NULL);
	efi_memmap_walk(find_largest_hole, (u64 *)&max_gap);
	if (max_gap < LARGE_GAP) {
		vmem_map = (struct page *) 0;
		free_area_init_nodes(max_zone_pfns);
	} else {
		unsigned long map_size;

		/* allocate virtual_mem_map */

		map_size = PAGE_ALIGN(ALIGN(max_low_pfn, MAX_ORDER_NR_PAGES) *
			sizeof(struct page));
		vmalloc_end -= map_size;
		vmem_map = (struct page *) vmalloc_end;
		efi_memmap_walk(create_mem_map_page_table, NULL);

		/*
		 * alloc_node_mem_map makes an adjustment for mem_map
		 * which isn't compatible with vmem_map.
		 */
		NODE_DATA(0)->node_mem_map = vmem_map +
			find_min_pfn_with_active_regions();
		free_area_init_nodes(max_zone_pfns);

		printk("Virtual mem_map starts at 0x%p\n", mem_map);
	}
#else /* !CONFIG_VIRTUAL_MEM_MAP */
	add_active_range(0, 0, max_low_pfn);
	free_area_init_nodes(max_zone_pfns);
#endif /* !CONFIG_VIRTUAL_MEM_MAP */
	zero_page_memmap_ptr = virt_to_page(ia64_imva(empty_zero_page));
}