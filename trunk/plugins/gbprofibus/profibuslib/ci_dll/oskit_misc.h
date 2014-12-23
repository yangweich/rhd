/* delay for x milliseconds */
extern void delay(unsigned int x);

extern int pci_find(unsigned vendor_id, unsigned device_id);
extern int pci_read_config(unsigned reg, unsigned *x);
