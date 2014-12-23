#include "oskit_misc.h"
#include <oskit/x86/pio.h>

void delay(unsigned int x)
{
  x *= 1000;
  while (x) { iodelay(); x--; }
}

#define pci_init              FDEV_LINUX_pci_init
#define pci_devices           FDEV_LINUX_pci_devices
#define pci_find_device       FDEV_LINUX_pci_find_device
#define pci_set_master        FDEV_LINUX_pci_set_master
#define pci_read_config_byte  FDEV_LINUX_pci_read_config_byte
#define pci_read_config_word  FDEV_LINUX_pci_read_config_word
#define pci_read_config_dword FDEV_LINUX_pci_read_config_dword
#define pci_write_config_byte FDEV_LINUX_pci_write_config_byte

#define __KERNEL__
#include <linux/pci.h>

static struct pci_dev *dev = 0;

int pci_find(unsigned vendor_id, unsigned device_id)
{
  if (!dev)
    {
      pci_init();
      dev = pci_devices;
    }

  while (dev)
    {
      if ((dev->vendor == vendor_id) &&
          (dev->device == device_id))
	return 0;

      dev = dev->next;
    }

  return -1;
}

int pci_read_config(unsigned reg, unsigned *x)
{
  if (dev)
    {
      pci_read_config_dword(dev, reg, x);
      return 0;
    }
  else
    return -1;
}

#if 0
#include <oskit/startup.h>
#include <oskit/clientos.h>

#include "dp_5613.h"
#include "ci_5613.h"
int main()
{
  oskit_clientos_init();
  start_devices();
  start_fs_bmod();
  /* delay(5000); */
  return CI_start_cp(0, 0, 0, 0);
}
#endif
