/*
 *  BSD LICENSE
 *
 *  Copyright (c) 2011-2018 Broadcom.  All Rights Reserved.
 *  The term "Broadcom" refers to Broadcom Limited and/or its subsidiaries.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *    * Neither the name of Intel Corporation nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *
 * OCS linked list API
 *
 */

#if !defined(__OCS_LIST_H__)
#define __OCS_LIST_H__

#define OCS_LIST_DEBUG

#if defined(OCS_LIST_DEBUG)

extern void _ocs_list_assertmsg(const char *label, const char *filename, int linenum);

#define ocs_list_magic_decl		uint32_t magic;
#define OCS_LIST_LIST_MAGIC		0xcafe0000
#define OCS_LIST_LINK_MAGIC		0xcafe0001
#define ocs_list_set_list_magic		list->magic = OCS_LIST_LIST_MAGIC
#define ocs_list_set_link_magic		list->magic = OCS_LIST_LINK_MAGIC

#define ocs_list_assert(cond, ...) \
	if (!(cond)) { \
		_ocs_list_assertmsg(#cond, __FILE__, __LINE__); \
		return __VA_ARGS__; \
	}
#else
#define ocs_list_magic_decl
#define ocs_list_assert(cond, ...)
#define ocs_list_set_list_magic
#define ocs_list_set_link_magic
#endif

/**
 * @brief list/link structure
 *
 * used for both the list object, and the link object(s).  offset
 * is specified when the list is initialized; this implies that a list
 * will always point to objects of the same type.  offset is not used
 * when ocs_list_t is used as a link (ocs_list_link_t).
 *
 */

typedef struct ocs_list_s ocs_list_t;
struct ocs_list_s {
	ocs_list_magic_decl			/*<< used if debugging is enabled */
	ocs_list_t *next;			/*<< pointer to head of list (or next if link) */
	ocs_list_t *prev;			/*<< pointer to tail of list (or previous if link) */
	uint32_t offset;			/*<< offset in bytes to the link element of the objects in list */
};
typedef ocs_list_t ocs_list_link_t;

/* item2link - return pointer to link given pointer to an item */
#define item2link(list, item)	((ocs_list_t*) (((uint8_t*)(item)) + (list)->offset))

/* link2item - return pointer to item given pointer to a link */
#define link2item(list, link)	((void*) (((uint8_t*)(link)) - (list)->offset))

/**
 * @brief Initialize a list
 *
 * A list object is initialized.  Helper define is used to call _ocs_list_init() with
 * offsetof(type, link)
 *
 * @param list Pointer to list
 * @param offset Offset in bytes in item to the link element
 *
 * @return none
 */
static inline void
_ocs_list_init(ocs_list_t *list, uint32_t offset)
{
	ocs_list_assert(list);
	ocs_list_set_list_magic;

	list->next = list;
	list->prev = list;
	list->offset = offset;
}
#define ocs_list_init(head, type, link)		_ocs_list_init(head, offsetof(type, link))


/**
 * @ingroup os
 * @brief Test if a list is empty
 *
 * @param list Pointer to list head
 *
 * @return 1 if empty, 0 otherwise
 */
static inline int32_t
ocs_list_empty(ocs_list_t *list)
{
	ocs_list_assert(list, 1);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC, 1);
	return list->next == list;
}

/**
 * @ingroup os
 * @brief Test if a list is valid (ready for use)
 *
 * @param list Pointer to list head
 *
 * @return true if list is usable, false otherwise
 */
static inline int
ocs_list_valid(ocs_list_t *list)
{
	return (list->magic == OCS_LIST_LIST_MAGIC);
}

/**
 * @brief Insert link between two other links
 *
 * Inserts a link in between two other links
 *
 * @param a Pointer to first link
 * @param b Pointer to next link
 * @param c Pointer to link to insert between a and b
 *
 * @return none
 */
static inline void
_ocs_list_insert_link(ocs_list_t *a, ocs_list_t *b, ocs_list_t *c)
{
	ocs_list_assert(a);
	ocs_list_assert((a->magic == OCS_LIST_LIST_MAGIC) || (a->magic == OCS_LIST_LINK_MAGIC));
	ocs_list_assert(a->next);
	ocs_list_assert(a->prev);
	ocs_list_assert(b);
	ocs_list_assert((b->magic == OCS_LIST_LIST_MAGIC) || (b->magic == OCS_LIST_LINK_MAGIC));
	ocs_list_assert(b->next);
	ocs_list_assert(b->prev);
	ocs_list_assert(c);
	ocs_list_assert((c->magic == OCS_LIST_LIST_MAGIC) || (c->magic == OCS_LIST_LINK_MAGIC));
	ocs_list_assert(!c->next);
	ocs_list_assert(!c->prev);

	ocs_list_assert(a->offset == b->offset);
	ocs_list_assert(b->offset == c->offset);

	c->next = a->next;
	c->prev = b->prev;
	a->next = c;
	b->prev = c;
}

#if defined(OCS_LIST_DEBUG)
/**
 * @brief Initialize a list link for debug purposes
 *
 * For debugging a linked list link element has a magic number that is initialized,
 * and the offset value initialzied and used for subsequent assertions.
 *
 *
 * @param list Pointer to list head
 * @param link Pointer to link to be initialized
 *
 * @return none
 */
static inline void
ocs_list_init_link(ocs_list_t *list, ocs_list_t *link)
{
	ocs_list_assert(list);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC);
	ocs_list_assert(link);

	if (link->magic == 0) {
		link->magic = OCS_LIST_LINK_MAGIC;
		link->offset = list->offset;
		link->next = NULL;
		link->prev = NULL;
	}
}
#else
#define ocs_list_init_link(...)
#endif

/**
 * @ingroup os
 * @brief Add an item to the head of the list
 *
 * @param list Pointer to list head
 * @param item Item to add
 */
static inline void
ocs_list_add_head(ocs_list_t *list, void *item)
{
	ocs_list_t *link;

	ocs_list_assert(list);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC);
	ocs_list_assert(item);

	link = item2link(list, item);
	ocs_list_init_link(list, link);

	ocs_list_assert(link->magic == OCS_LIST_LINK_MAGIC);
	ocs_list_assert(link->offset == list->offset);
	ocs_list_assert(link->next == NULL);
	ocs_list_assert(link->prev == NULL);

	_ocs_list_insert_link(list, list->next, item2link(list, item));
}


/**
 * @ingroup os
 * @brief Add an item to the tail of the list
 *
 * @param list Head of the list
 * @param item Item to add
 */
static inline void
ocs_list_add_tail(ocs_list_t *list, void *item)
{
	ocs_list_t *link;

	ocs_list_assert(list);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC);
	ocs_list_assert(item);

	link = item2link(list, item);
	ocs_list_init_link(list, link);

	ocs_list_assert(link->magic == OCS_LIST_LINK_MAGIC);
	ocs_list_assert(link->offset == list->offset);
	ocs_list_assert(link->next == NULL);
	ocs_list_assert(link->prev == NULL);

	_ocs_list_insert_link(list->prev, list, link);
}


/**
 * @ingroup os
 * @brief Return the first item in the list
 *
 * @param list Head of the list
 *
 * @return pointer to the first item, NULL otherwise
 */
static inline void *
ocs_list_get_head(ocs_list_t *list)
{
	ocs_list_assert(list, NULL);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC, NULL);
	return ocs_list_empty(list) ? NULL : link2item(list, list->next);
}

/**
 * @ingroup os
 * @brief Return the first item in the list
 *
 * @param list head of the list
 *
 * @return pointer to the last item, NULL otherwise
 */
static inline void *
ocs_list_get_tail(ocs_list_t *list)
{
	ocs_list_assert(list, NULL);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC, NULL);
	return ocs_list_empty(list) ? NULL : link2item(list, list->prev);
}

/**
 * @ingroup os
 * @brief Return the last item in the list
 *
 * @param list Pointer to list head
 *
 * @return pointer to the last item, NULL otherwise
 */
static inline void *ocs_list_tail(ocs_list_t *list)
{
	ocs_list_assert(list, NULL);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC, NULL);
	return ocs_list_empty(list) ? NULL : link2item(list, list->prev);
}

/**
 * @ingroup os
 * @brief Get the next item on the list
 *
 * @param list head of the list
 * @param item current item
 *
 * @return pointer to the next item, NULL otherwise
 */
static inline void *ocs_list_next(ocs_list_t *list, void *item)
{
	ocs_list_t *link;

	//TODO: this is needed, not sure why
	if (item == NULL) {
		return NULL;
	}

	ocs_list_assert(list, NULL);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC, NULL);
	ocs_list_assert(item, NULL);

	link = item2link(list, item);

	ocs_list_assert(link->magic == OCS_LIST_LINK_MAGIC, NULL);
	ocs_list_assert(link->offset == list->offset, NULL);
	ocs_list_assert(link->next, NULL);
	ocs_list_assert(link->prev, NULL);

	if ((link->next) == list) {
		return NULL;
	}

	return link2item(list, link->next);
}

/**
 * @ingroup os
 * @brief Remove and return an item from the head of the list
 *
 * @param list head of the list
 *
 * @return pointer to returned item, or NULL if list is empty
 */
#define ocs_list_remove_head(list)		ocs_list_remove(list, ocs_list_get_head(list))

/**
 * @ingroup os
 * @brief Remove an item from the list
 *
 * @param list Head of the list
 * @param item Item to remove
 *
 * @return pointer to item, or NULL if item is not found.
 */
static inline void *ocs_list_remove(ocs_list_t *list, void *item)
{
	ocs_list_t *link;
	ocs_list_t *prev;
	ocs_list_t *next;

	if (item == NULL) {
		return NULL;
	}
	ocs_list_assert(list, NULL);
	ocs_list_assert(list->magic == OCS_LIST_LIST_MAGIC, NULL);

	link = item2link(list, item);

	ocs_list_assert(link->magic == OCS_LIST_LINK_MAGIC, NULL);
	ocs_list_assert(link->offset == list->offset, NULL);
	ocs_list_assert(link->next, NULL);
	ocs_list_assert(link->prev, NULL);

	prev = link->prev;
	next = link->next;

	prev->next = next;
	next->prev = prev;

	link->next = link->prev = NULL;

	return item;
}

/**
 * @brief Iterate a linked list
 *
 * Iterate a linked list.
 *
 * @param list Pointer to list
 * @param item Pointer to iterated item
 *
 * note, item is NULL after full list is traversed.

 * @return none
 */

#define ocs_list_foreach(list, item) \
	for (item = ocs_list_get_head((list)); item; item = ocs_list_next((list), item) )

/**
 * @brief Iterate a linked list safely
 *
 * Iterate a linked list safely, meaning that the iterated item
 * may be safely removed from the list.
 *
 * @param list Pointer to list
 * @param item Pointer to iterated item
 * @param nxt Pointer to saveed iterated item
 *
 * note, item is NULL after full list is traversed.
 *
 * @return none
 */

#define ocs_list_foreach_safe(list, item, nxt) \
	for (item = ocs_list_get_head(list), nxt = item ? ocs_list_next(list, item) : NULL; item; \
		item = nxt, nxt = ocs_list_next(list, item))

/**
 * @brief Test if object is on a list
 *
 * Returns True if object is on a list
 *
 * @param link Pointer to list link
 *
 * @return returns True if object is on a list
 */
static inline int32_t
ocs_list_on_list(ocs_list_link_t *link)
{
	return (link->next != NULL);
}

#endif // __OCS_LIST_H__

/**
 * @page linux_os_overview OS APIs
 * - @ref os
 *
 * <div class="overview">
 * <img src="elx_linux_os.jpg" alt="OS Component" title="OS Component" align="right"/>
 *
 * <h2>Linux OS</h2>
 *
 * The Linux OS component consists of the OS abstraction and PCI subcomponents,
 * as described in the following sections.
 *
 * <h3>OS Abstraction</h3>
 *
 * The transport, HAL, and SLI layers achieve operating system independence through
 * the use of an OS abstraction. The required functionality includes:
 * <ul><li>Defining all common objects (that is, DMA, SLI port, domain, and remote node).
 * <li>Declaring endianess and providing functions for converting between host order
 * and big-endian byte order.
 * <li>Busy-wait or delay function.
 * <li>Memory copy and set routines.
 * <li>Memory allocation, free, and cache coherency functions for both CPU and DMA
 * memory types.
 * <li>Locking for concurrency protection.
 * <li>Linked list and associated operations.
 * <li>Bitmap and associated operations.
 * <li>PCI register access.
 * </ul>
 *
 * In most cases, the OS abstractions have been implemented with macros or in-line functions.
 *
 * <h3>PCI</h3>
 *
 * The ocs_lnx.c file contains code for performing PCI-related operations and interrupt
 * handling. The ocs_pci_probe() and ocs_pci_remove() functions provide two main PCI operations:
 * <ul><li>ocs_pci_probe() – allocates required structures, sets up interrupt handling, and
 * sets up the hardware through the HAL. It also registers callback functions with
 * the HAL, and requests the HAL to bring the port online.
 * <li>ocs_pci_remove() – performs the reverse of ocs_pci_probe(). It requests the HAL to
 * shutdown the port, releases resources, and unregisters the interrupt handler.
 * </ul>
 *
 * Interrupt handling begins with the interrupt service routine, ocs_intr_msix(). This
 * function launches a tasklet to perform interrupt services. This tasklet (ocs_tasklet) calls
 * ocs_hal_process() in the HAL to process the interrupt.
 *
 * The ocs_lnx.c file also contains module parameters that can be set when the module is
 * loaded. See the
 * <i><a href="../../../../../doc/ocs_sdk_quick_start_guide.pdf" target="_blank">OneCore Storage Quick Start Guide</a></i>
 * for a list of available module parameters.
 * <br><br>
 * </div><!-- overview -->
 */



/**
 * @page uspace_linux_os_overview OS APIs
 * - @ref os
 *
 * <div class="overview">
 * <img src="elx_uspace_linux_os.jpg" alt="OS Component" title="OS Component" align="right"/>
 *
 * <h2>Linux OS</h2>
 *
 * The Linux OS component consists of the OS abstraction subcomponent,
 * as described in the following section.
 * @n @n
 * @b Note: For the kernel space drivers, the OS component includes the PCI subcomponent.
 * However, for the user space drivers, the PCI subcomponent is managed in the
 * <a href="kernel_mod_overview.html">Kernel Module</a>.
 *
 * <h3>OS Abstraction</h3>
 *
 * The transport, HAL, and SLI layers achieve operating system independence through
 * the use of an OS abstraction. The required functionality includes:
 * <ul><li>Defining all common objects (that is, DMA, SLI port, domain, and remote node).
 * <li>Declaring endianess and providing functions for converting between host order
 * and big-endian byte order.
 * <li>Busy-wait or delay function.
 * <li>Memory copy and set routines.
 * <li>Memory allocation, free, and cache coherency functions for both CPU and DMA
 * memory types.
 * <li>Locking for concurrency protection.
 * <li>Linked list and associated operations.
 * <li>Bitmap and associated operations.
 * <li>PCI register access.
 * </ul>
 *
 * In most cases, the OS abstractions have been implemented with macros or in-line functions.
 * <br><br>
 * </div><!-- overview -->
 */

/**
 * @page bsd_os_overview OS APIs
 * - @ref os
 *
 * <div class="overview">
 * <img src="elx_bsd_os.jpg" alt="OS Component" title="OS Component" align="right"/>
 *
 * <h2>FreeBSD OS</h2>
 *
 * The FreeBSD OS component consists of the OS abstraction and PCI subcomponents,
 * as described in the following sections.
 *
 * <h3>OS Abstraction</h3>
 *
 * The transport, HAL, and SLI layers achieve operating system independence through
 * the use of an OS abstraction. The required functionality includes:
 * <ul><li>Defining all common objects (that is, DMA, SLI port, domain, and remote node).
 * <li>Declaring endianess and providing functions for converting between host order
 * and big-endian byte order.
 * <li>Busy-wait or delay function.
 * <li>Memory copy and set routines.
 * <li>Memory allocation, free, and cache coherency functions for both CPU and DMA
 * memory types.
 * <li>Locking for concurrency protection.
 * <li>Linked list and associated operations.
 * <li>Bitmap and associated operations.
 * <li>PCI register access.
 * </ul>
 *
 * In most cases, the OS abstractions have been implemented with macros or in-line functions.
 * <br><br>
 * <h3>PCI</h3>
 *
 * The PCI component, contained in ocs_pci.c file, is the main entry point to the driver and
 * implements the standard FreeBSD methods:
 * <ul><li>ocs_pci_probe() – determines if the driver supports the provided PCI vendor and device.
 * ID.</li>
 * <li>ocs_pci_attach() – performs device initialization and register with CAM.</li>
 * <li>ocs_pci_detach() – stops the device.</li>
 * <li>ocs_pci_shutdown() – frees allocated resources.</li></ul>
 *
 * The driver also allocates interrupt resources through this component,
 * either using MSI-X or INTx (legacy) interrupts. @n @n
 *
 * The ocs_pci.c file also contains module parameters that can be set when the module is
 * loaded. See the
 * <i><a href="../../../../../doc/ocs_sdk_quick_start_guide.pdf" target="_blank">OneCore Storage Quick Start Guide</a></i>
 * for a list of available module parameters.
 * <br><br>
 * </div><!-- overview -->
 */
