/*
 *	The PCI Library -- Example of use (simplistic lister of PCI devices)
 *
 *	Written by Martin Mares and put to public domain. You can do
 *	with it anything you want, but I don't give you any warranty.
 */

#include <stdio.h>

#include <pci/pci.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "5613_ret.h"
  
/* defines for vendorID and deviceID of CP5613 */
#define VENDOR_ID         0x110a
#define DEVICE_ID         0x3142

/* base[0]: plx      0x54 bytes
   base[2]: download 0x100000 bytes
   base[3]: data     0x100000 bytes
*/


long cp5613_available(unsigned long *plxp,
		      unsigned long *dloadp,
		      unsigned long *dpramp)
{
  long ret = CI_RET_START_CP_NOT_FOUND;
  struct pci_access *pacc;
  struct pci_dev *dev;
  unsigned int c;
  int mem;
  void *plx, *dload, *dpram;

  pacc = pci_alloc();		/* Get the pci_access structure */
  /* Set all options you want -- here we stick with the defaults */
  pci_init(pacc);		/* Initialize the PCI library */
  pci_scan_bus(pacc);		/* We want to get the list of devices */
  for(dev=pacc->devices; dev; dev=dev->next)	/* Iterate over all devices */
    {
      if ((dev->vendor_id == VENDOR_ID) && (dev->device_id == DEVICE_ID))
	{
	  /* Fill in header info we need */
	  pci_fill_info(dev, PCI_FILL_IDENT | PCI_FILL_BASES);

	  /* Read config register directly */
	  c = pci_read_word(dev, PCI_CLASS_DEVICE);

	  printf("%02x:%02x.%d vendor=%04x device=%04x class=%04x irq=%d\n",
		 dev->bus, dev->dev, dev->func, dev->vendor_id, dev->device_id,
		 c, dev->irq);
	  printf(" base0=%lx base2=%lx base3=%lx\n",
		 (long unsigned int)dev->base_addr[0], (long unsigned int)dev->base_addr[2],
		 (long unsigned int)dev->base_addr[3]);

	  mem = open("/dev/mem", O_RDWR);
	  if (mem < 0)
	    {
	      fprintf(stderr, "Can't open /dev/mem.\n");
	      ret = CI_RET_START_CP_RESOURCES_INT;
	      break;
	    }

	  dpram = mmap(0, 0x100000, PROT_READ|PROT_WRITE, MAP_SHARED, mem,
		       dev->base_addr[3]);
	  if (dpram == MAP_FAILED)
	    {
	      perror("Can't map dpram");
	      ret = CI_RET_START_CP_RESOURCES_INT;
	      break;
	    }

	  dload = mmap(0, 0x100000, PROT_READ|PROT_WRITE, MAP_SHARED, mem,
		       dev->base_addr[2]);
	  if (dpram == MAP_FAILED)
	    {
	      perror("Can't map download");
	      ret = CI_RET_START_CP_RESOURCES_INT;
	      break;
	    }

	  {
	    int size = getpagesize();
	    unsigned long addr = dev->base_addr[0] & ~(size-1);

	    plx = mmap(0, size, PROT_READ|PROT_WRITE, MAP_SHARED, mem, addr);
	    if (plx == MAP_FAILED)
	      {
		perror("Can't map plx");
		ret = CI_RET_START_CP_RESOURCES_INT;
		break;
	      }
	    plx += dev->base_addr[0] - addr;
	  }

	  *plxp = (unsigned long) plx;
	  *dloadp = (unsigned long) dload;
	  *dpramp = (unsigned long) dpram;

	  ret = CI_RET_OK;
	  break;
	}
    }

  if (!dev)
    {
      fprintf(stderr, "PCI Card not found.\n");
    }

  pci_cleanup(pacc);		/* Close everything */
  return ret;
}

#if 0
int main()
{
  int e;
  unsigned long plx, dload, dpram;

  e = cp5613_available(&plx, &dload, &dpram);

  if (e != CI_RET_OK)
    {
      printf("cp5613 not available: error %d.\n", e);
    }
  else
    {
      printf("cp5613 available.\nMapped at addresses %lx, %lx and %lx.\n",
	     plx, dload, dpram);
    }

  return 0;
}
#endif
