/* AMI BIOS DMI update driver
 * Copyright (C) 2013 Claudio Matsuoka
 * Copyright (C) 2013 CITS - Centro Internacional de Tecnologia de Software
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License v2.0 as published by
 * the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/ctype.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include "amibios_smi.h"

struct dmi_type {
	struct amibios_dmi_structure_info info;
	struct kobject *kobj;
};

static struct kobject *amibios_kobj;
static struct dmi_type type1;
static struct dmi_type type2;
static struct dmi_type type3;
static int smbios_version;


/*
 * Sysfs interface
 */

#define DMI_UPDATE(x,y,z,w)						\
static ssize_t show_type##x##_##w(struct kobject *kobj,			\
		struct kobj_attribute *attr, char *buf)	{		\
	return show_dmi_##z(&type##x, (y), buf);			\
}									\
static ssize_t store_type##x##_##w(struct kobject *kobj, struct 	\
		kobj_attribute *attr, const char *buf, size_t count) {	\
	return store_dmi_##z(&type##x, (y), buf, count);		\
}									\
static struct kobj_attribute type##x##_##w##_attribute =		\
	__ATTR(w, 0600, show_type##x##_##w, store_type##x##_##w);

#define FIELD_SIZE 32

static ssize_t show_dmi_string(struct dmi_type *dt, int offset, char *buf)
{
	char s[FIELD_SIZE];

	if (amibios_dmi_get_string(&dt->info, offset, s, FIELD_SIZE) < 0) {
		pr_err("amibios_dmi: error reading string (handle 0x%04x "
				"offset 0x%02x)\n", dt->info.handle, offset);
		return sprintf(buf, "%s\n", "<error>");
	}

	return sprintf(buf, "%s\n", s);
}

static ssize_t store_dmi_string(struct dmi_type *dt, int offset,
					const char *buf, size_t count)
{
	char s[FIELD_SIZE];
	int i, len;

	strncpy(s, buf, FIELD_SIZE);
	s[FIELD_SIZE - 1] = 0;
	len = strlen(s);
	if (len > 0 && s[len - 1] == '\n')
		s[--len] = 0;

	/* Filter out invalid bytes */
	for (i = 0; s[i]; i++) {
		if (!isprint(s[i]))
			s[i] = ' ';
	}

	pr_info("amibios_dmi: set handle 0x%04x offset 0x%02x: "
				"\"%s\"\n", dt->info.handle, offset, s);
			
	if (amibios_smi_write_string(dt->info.handle, offset, s) < 0) {
		pr_err("amibios_dmi: error setting string (handle 0x%04x "
				"offset 0x%02x)\n", dt->info.handle, offset);
	}

	return count;
}

static ssize_t show_dmi_byte(struct dmi_type *dt, int offset, char *buf)
{
	u8 c;

	if (amibios_dmi_get_byte(&dt->info, offset, &c) < 0) {
		pr_err("amibios_dmi: error reading byte (handle 0x%04x "
				"offset 0x%02x)\n", dt->info.handle, offset);
		return sprintf(buf, "%s\n", "<error>");
	}

	return sprintf(buf, "0x%02x\n", c);
}

static ssize_t store_dmi_byte(struct dmi_type *dt, int offset,
					const char *buf, size_t count)
{
	int ret;
	long val;

	ret = kstrtol(buf, 0, &val);

	pr_info("amibios_dmi: set handle 0x%04x offset 0x%02x: "
				"0x%02x\n", dt->info.handle, offset, (int)val);
			
	if (ret < 0 || val < 0 || val > 0xff || amibios_smi_write_byte(
					dt->info.handle, offset, val) < 0) {
		pr_err("amibios_dmi: error setting byte (handle 0x%04x "
				"offset 0x%02x)\n", dt->info.handle, offset);
	}

	return count;
}

static ssize_t show_smbios_version(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%08x\n", smbios_version);
}

DMI_UPDATE(1, 0x04, string, manufacturer)
DMI_UPDATE(1, 0x05, string, product_name)
DMI_UPDATE(1, 0x06, string, version)
DMI_UPDATE(1, 0x07, string, serial_number)

DMI_UPDATE(2, 0x04, string, manufacturer)
DMI_UPDATE(2, 0x05, string, product_name)
DMI_UPDATE(2, 0x06, string, version)
DMI_UPDATE(2, 0x07, string, serial_number)
DMI_UPDATE(2, 0x08, string, asset_tag)

DMI_UPDATE(3, 0x04, string, manufacturer)
DMI_UPDATE(3, 0x05, byte,   type)
DMI_UPDATE(3, 0x06, string, version)
DMI_UPDATE(3, 0x07, string, serial_number)
DMI_UPDATE(3, 0x08, string, asset_tag)

static struct kobj_attribute smbios_version_attribute =
	__ATTR(smbios_version, 0444, show_smbios_version, NULL);

static struct attribute *amibios_attrs[] = {
	&smbios_version_attribute.attr,
	NULL,
};

static struct attribute_group amibios_attr_group = {
	.attrs = amibios_attrs,
};

static struct attribute *type1_attrs[] = {
	&type1_manufacturer_attribute.attr,
	&type1_product_name_attribute.attr,
	&type1_version_attribute.attr,
	&type1_serial_number_attribute.attr,
	NULL
};

static struct attribute_group type1_attr_group = {
	.attrs = type1_attrs,
};

static struct attribute *type2_attrs[] = {
	&type2_manufacturer_attribute.attr,
	&type2_product_name_attribute.attr,
	&type2_version_attribute.attr,
	&type2_serial_number_attribute.attr,
	&type2_asset_tag_attribute.attr,
	NULL
};

static struct attribute_group type2_attr_group = {
	.attrs = type2_attrs,
};

static struct attribute *type3_attrs[] = {
	&type3_manufacturer_attribute.attr,
	&type3_type_attribute.attr,
	&type3_version_attribute.attr,
	&type3_serial_number_attribute.attr,
	&type3_asset_tag_attribute.attr,
	NULL
};

static struct attribute_group type3_attr_group = {
	.attrs = type3_attrs,
};


/*
 * Module initialization
 */

static int __init structure_init(int type, struct dmi_type *dt,
			char *name, struct attribute_group *attr_group)
{
	int retval = 0, handle;

	handle = amibios_dmi_get_handle(type);
	if (handle < 0)
		goto err;

	dt->info.handle = handle;

	dt->kobj = kobject_create_and_add(name, amibios_kobj);
	if (!dt->kobj) {
		retval = -ENOMEM;
		goto err;
	}

	retval = sysfs_create_group(dt->kobj, attr_group);
	if (retval)
		goto err2;

	return 0;

    err2:
	kobject_put(dt->kobj);
    err:
	return retval;
}

static void structure_deinit(struct dmi_type *dt,
				struct attribute_group *attr_group)
{
	sysfs_remove_group(dt->kobj, attr_group);
	kobject_put(dt->kobj);
}

static int __init amibios_init(void)
{
	int retval;

	retval = amibios_smi_init(&smbios_version);
	if (retval)
		goto err;

	amibios_kobj = kobject_create_and_add("amibios", firmware_kobj);
	if (!amibios_kobj) {
		retval = -ENOMEM;
		goto err2;
	}

	retval = sysfs_create_group(amibios_kobj, &amibios_attr_group);
	if (retval)
		goto err3;

	retval = structure_init(0x01, &type1, "system", &type1_attr_group);
	if (retval)
		goto err4;

	retval = structure_init(0x02, &type2, "baseboard", &type2_attr_group);
	if (retval)
		goto err5;

	retval = structure_init(0x03, &type3, "chassis", &type3_attr_group);
	if (retval)
		goto err6;

	return 0;
    
    err6:
	structure_deinit(&type2, &type2_attr_group);
    err5:
	structure_deinit(&type1, &type1_attr_group);
    err4:
	sysfs_remove_group(amibios_kobj, &amibios_attr_group);
    err3:
	kobject_put(amibios_kobj);
    err2:
	amibios_smi_deinit();
    err:
	return retval;
}

static void __exit amibios_exit(void)
{
	structure_deinit(&type3, &type3_attr_group);
	structure_deinit(&type2, &type2_attr_group);
	structure_deinit(&type1, &type1_attr_group);
	sysfs_remove_group(amibios_kobj, &amibios_attr_group);
	kobject_put(amibios_kobj);
	amibios_smi_deinit();
}

module_init(amibios_init);
module_exit(amibios_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Claudio Matsuoka <claudio.matsuoka@cits.br>");
