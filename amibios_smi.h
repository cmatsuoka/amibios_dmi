#ifndef AMIBIOS_SMI_H
#define AMIBIOS_SMI_H

struct amibios_dmi_structure_info {
	int handle;
	int header_size;
	int size;
};

int amibios_smi_init(int *);
void amibios_smi_deinit(void);
int amibios_smi_write_byte(int, int, u8);
int amibios_smi_write_string(int, int, char *);
int amibios_dmi_get_handle(int);
int amibios_dmi_get_byte(struct amibios_dmi_structure_info *, int, u8 *);
int amibios_dmi_get_string(struct amibios_dmi_structure_info *, int, char *, int);

#endif
