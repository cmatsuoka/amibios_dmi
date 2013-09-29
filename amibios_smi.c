/* AMI BIOS DMI update driver
 * Copyright (C) 2013 Claudio Matsuoka
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name(s) of the above-listed copyright holder(s) nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Alternatively, this software may be distributed, used, and modified
 * under the terms of the GPL v2 license.
 */

#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include "amibios_smi.h"

#define SMI_PORT		0xb2
#define SMI_CMD_INFO		0x50
#define SMI_CMD_READ		0x51
#define SMI_CMD_WRITE		0x52
#define SMI_BUFFER_SIZE		(2 * PAGE_SIZE)
#define SMI_DATA_SIZE		(PAGE_SIZE + SMI_BUFFER_SIZE)
#define SMI_DATA_ORDER		(get_order(SMI_DATA_SIZE))

/* AMI SMI error codes */
#define SMI_ERR_INVALID_CMD	0x81
#define SMI_ERR_UNSUP_CMD	0x82
#define SMI_ERR_INVALID_STRUCT	0x83
#define SMI_ERR_INVALID_PARM	0x84
#define SMI_ERR_UNSUP_SUBCMD	0x85
#define SMI_ERR_NO_CHANGES	0x86
#define SMI_ERR_STORAGE		0x87
#define SMI_ERR_READ_ONLY	0x8d

struct amibios_info {
	u16 command;		/* 0x00 */
	u32 version_ptr;	/* 0x02 */
	u32 st_num_ptr;		/* 0x06 */
	u32 st_size_ptr;	/* 0x0A */
	u32 addr_ptr;		/* 0x0E */
	u32 size_ptr;		/* 0x12 */
	u16 unknown;		/* 0x16 */
	u32 version;		/* 0x18 */
	u32 st_num;		/* 0x1C */
	u32 st_size;		/* 0x20 */
	u32 addr;		/* 0x24 */
	u32 size;		/* 0x28 */
} __attribute__((packed));

/*
 * EXAMPLE: BIOS Information with strings: 
 *
 * BIOS_Info	LABEL BYTE 
 *	db 0		; Indicates BIOS Structure Type 
 *	db 13h		; Length of information in bytes 
 *	dw ?		; Reserved for handle 
 *	db 01h		; String 1 is the Vendor Name 
 *	db 02h		; String 2 is the BIOS version 
 *	dw 0E800h	; BIOS Starting Address 
 *	db 03h		; String 3 is the BIOS Build Date
 *	db 1		; Size of BIOS ROM is 128K (64K * (1 + 1)) 
 *	dq BIOS_Char	; BIOS Characteristics 
 *	db 0		; BIOS Characteristics Extension Byte 1 
 *	db ‘System BIOS Vendor Name’,0 ; 
 *	db ‘4.04’,0	; 
 *	db ‘00/00/0000’,0 ; 
 *	db 0		; End of strings
 */
struct amibios_dmi_structure {
	u8 type;		/* 0x00 */
	u8 length;		/* 0x01 */
	u16 handle;		/* 0x02 */
	u8 data[1];		/* 0x04 */
} __attribute__((packed));

union amibios_dmi_data {
	struct amibios_dmi_structure dmi_structure;
	u8 raw[1];
};

struct amibios_read {
	u16 command;		/* 0x00 */
	u32 handle_ptr;		/* 0x02 */
	u32 structure_ptr;	/* 0x06 */
	u16 unknown1;		/* 0x0A */
	u16 unknown2;		/* 0x0C */
	u16 handle;		/* 0x0E */
	union amibios_dmi_data data;	/* 0x10 */
} __attribute__((packed));

struct amibios_write_data {
#define AMIBIOS_CMD_WRITE_BYTE	0
#define AMIBIOS_CMD_ADD		3
#define AMIBIOS_CMD_DELETE	4
#define AMIBIOS_CMD_WRITE_STRING 5
	u8 command;		/* 0x00 */
	u8 offset;		/* 0x01 */
	u32 unknown1;		/* 0x02 */
	u32 value;		/* 0x06 */
	u16 length;		/* 0x0A */
	union amibios_dmi_data data;	/* 0x10 */
} __attribute__((packed));

struct amibios_write {
	u16 command;		/* 0x00 */
	u32 data_ptr;		/* 0x02 */
	u32 buffer_ptr;		/* 0x06 */
	u8 write_enable;	/* 0x0A */
	u16 unknown1;		/* 0x0B */
	u16 unknown2;		/* 0x0D */
	struct amibios_write_data data;	/* 0x0F */
} __attribute__((packed));

union amibios_data {
	struct amibios_info info;
	struct amibios_read read;
	struct amibios_write write;
};

static union amibios_data *amibios_data;
static unsigned long amibios_data_ptr;
static unsigned long amibios_buffer_ptr;
static int amibios_st_size;
static int amibios_size;
static DEFINE_MUTEX(amibios_mutex);

static int dmi_structure_length(union amibios_dmi_data *dd)
{
	int i;

	for (i = dd->dmi_structure.length; i < (amibios_st_size - 1); i++) {
		if (dd->raw[i] == 0 && dd->raw[i + 1] == 0)
			return i + 2;
	}

	return -1;
}

static void smi_error_message(int code)
{
	char *msg;

	switch (code) {
	case SMI_ERR_INVALID_CMD:
		msg = "Invalid command";
		break;
	case SMI_ERR_UNSUP_CMD:
		msg = "Unsupported command";
		break;
	case SMI_ERR_INVALID_STRUCT:
		msg = "Invalid structure";
		break;
	case SMI_ERR_INVALID_PARM:
		msg = "Invalid parameter";
		break;
	case SMI_ERR_UNSUP_SUBCMD:
		msg = "Unsupported subcommand";
		break;
	case SMI_ERR_NO_CHANGES:
		msg = "No pending changes";
		break;
	case SMI_ERR_STORAGE:
		msg = "Out of storage";
		break;
	case SMI_ERR_READ_ONLY:
		msg = "Read only data";
		break;
	default:
		msg = "Unknown error";
		break;
	}

	pr_err("amibios_dmi: SMI error %02x: %s.\n", code, msg);
}

static int smi_command(int port, int cmd, u32 ptr)
{
	int retval;

#if defined(__i386__) || defined(__x86_64__)
	/*
	 * From the Phoenix Developer Network article on SMI handling:
	 * "In fact, while it appears that the SMI services are invoked
	 *  immediately after writing to the I/O port and before the next
	 *  CPU instruction, in some systems, the delay in propagation of
	 *  the interrupt to the CPU cores may allow the cores to execute
	 *  further instructions before the SMI is actually detected."
	 */
	asm volatile(
		"movl %[ptr], %%ebx\n\t"
		"movl %[cmd], %%eax\n\t"
		"movl %[port], %%edx\n\t"
		"outb %%al, %%dx\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"movl %%eax, %[retval]"
		: [retval] "=r"(retval)
		: [ptr] "r"(ptr), [cmd] "r"(cmd), [port] "r"(port)
		: "ebx", "eax", "edx");

#else
#error Unsupported architecture!
#endif

	return retval;
}

static int smi_info(void)
{
	int ret;

	/* Set up info structure */
	memset(amibios_data, 0, SMI_DATA_SIZE);
	amibios_data->info.command     = SMI_CMD_INFO;
	amibios_data->info.version_ptr = amibios_data_ptr + 0x18;
	amibios_data->info.st_num_ptr  = amibios_data_ptr + 0x1c;
	amibios_data->info.st_size_ptr = amibios_data_ptr + 0x20;
	amibios_data->info.addr_ptr    = amibios_data_ptr + 0x24;
	amibios_data->info.size_ptr    = amibios_data_ptr + 0x28;

	ret = smi_command(SMI_PORT, SMI_CMD_INFO, (u32)amibios_data_ptr);
	if (ret) {
		if (ret == SMI_CMD_INFO) {
			pr_err("amibios_dmi: Compatible AMI BIOS not found\n");
		} else {
			smi_error_message(ret);
		}

		return -1;
	}

	amibios_st_size = amibios_data->info.st_size;
	amibios_size    = amibios_data->info.size;

	return 0;
}

static int smi_read(int handle)
{
	int ret, len;

	if (smi_info() < 0)
		return -1;

	if (amibios_st_size > PAGE_SIZE) {
		pr_err("amibios_dmi: structure too large (%d bytes)\n",
							amibios_st_size);
		return -1;
	}


	/* Set up read structure */
	memset(amibios_data, 0, SMI_DATA_SIZE);
	amibios_data->read.command       = SMI_CMD_READ;
	amibios_data->read.handle_ptr    = amibios_data_ptr + 0x0e;
	amibios_data->read.structure_ptr = amibios_data_ptr + 0x10;
	amibios_data->read.handle        = handle;

	ret = smi_command(SMI_PORT, SMI_CMD_READ, (u32)amibios_data_ptr);
	if (ret) {
		smi_error_message(ret);
		return -1;
	}

	/* Sanity check */
	if (amibios_data->read.data.dmi_structure.handle != handle) {
		pr_err("amibios_dmi: SMI read error (handle 0x%04x)\n", handle);
		return -1;
	}

	len = dmi_structure_length(&amibios_data->read.data);
	if (len < 0) {
		pr_err("amibios_dmi: DMI structure too long (handle 0x%04x)\n",
								handle);
		return -1;
	}

	return len;
}

static int check_buffer_size(int n)
{
	if (amibios_size > SMI_BUFFER_SIZE + n) {
		pr_err("amibios_dmi: out of work buffer space\n");
		return -1;
	}

	return 0;
}

/**
 * amibios_smi_write_byte() - write a byte to a DMI structure field
 * @handle: DMI structure handle
 * @offset: DMI structure field to read
 * @b:      the byte to write
 *
 * Write the given byte to the DMI structure.
 *
 * Return:  0 if successful, or -1 in case of error.
 */
int amibios_smi_write_byte(int handle, int offset, u8 b)
{
	int ret, len;

	mutex_lock(&amibios_mutex);

	/* Read existing data */
	len = smi_read(handle);
	if (len < 0)
		goto err;

	if (check_buffer_size(1) < 0)
		goto err;

	memmove(&amibios_data->write.data.data, &amibios_data->read.data, len);
	
	/* Set up write structure */
	amibios_data->write.command       = SMI_CMD_WRITE;
	amibios_data->write.data_ptr      = amibios_data_ptr + 0x0f;
	amibios_data->write.buffer_ptr    = amibios_buffer_ptr;
	amibios_data->write.write_enable  = 1;
	amibios_data->write.unknown1      = 0;
	amibios_data->write.unknown2      = 0;

	amibios_data->write.data.command  = AMIBIOS_CMD_WRITE_BYTE;
	amibios_data->write.data.offset   = offset;
	amibios_data->write.data.unknown1 = 0;
	amibios_data->write.data.value    = b;
	amibios_data->write.data.length   = 1;

	ret = smi_command(SMI_PORT, SMI_CMD_WRITE, (u32)amibios_data_ptr);
	if (ret) {
		smi_error_message(ret);
		goto err;
	}

	mutex_unlock(&amibios_mutex);
	return 0;

    err:
	mutex_unlock(&amibios_mutex);
	return -1;
}

/**
 * amibios_smi_write_string() - write a string to a DMI structure field
 * @handle: DMI structure handle
 * @offset: DMI structure field to read
 * @s     : the string to write
 *
 * Write the given string to the DMI structure. If the string is longer
 * or shorter than the existing string, the SMI handler will take care of
 * moving data to accomodate the new string. amibios_buffer_ptr is used
 * as a temporary buffer to rewrite DMI data.
 *
 * Return:  0 if successful, or -1 in case of error.
 */
int amibios_smi_write_string(int handle, int offset, char *s)
{
	int ret, len;

	mutex_lock(&amibios_mutex);

	/* Read existing data */
	len = smi_read(handle);
	if (len < 0)
		goto err;

	if (check_buffer_size(0x80) < 0)	/* max. string size */
		goto err;

	memmove(&amibios_data->write.data.data, &amibios_data->read.data, len);
	
	/* Set up write structure */
	amibios_data->write.command      = SMI_CMD_WRITE;
	amibios_data->write.data_ptr     = amibios_data_ptr + 0x0f;
	amibios_data->write.buffer_ptr   = amibios_buffer_ptr;
	amibios_data->write.write_enable = 1;
	amibios_data->write.unknown1     = 0;
	amibios_data->write.unknown2     = 0;

	amibios_data->write.data.command = AMIBIOS_CMD_WRITE_STRING;
	amibios_data->write.data.offset  = offset;
	amibios_data->write.data.length  = strlen(s) + 1;
	strcpy(amibios_data->write.data.data.raw + 4, s);

	ret = smi_command(SMI_PORT, SMI_CMD_WRITE, (u32)amibios_data_ptr);
	if (ret) {
		smi_error_message(ret);
		goto err;
	}

	mutex_unlock(&amibios_mutex);
	return 0;

    err:
	mutex_unlock(&amibios_mutex);
	return -1;
}

/**
 * amibios_smi_init() - initialize SMI data.
 * @version: return buffer for SMBIOS version
 *
 * Initialize buffers and pointers to use the BIOS SMI handlers.
 *
 * Return:   0 if successful, or the error code in case of error.
 */
int __init amibios_smi_init(int *version)
{
	int retval = -ENODEV;

	mutex_lock(&amibios_mutex);

	amibios_data = (void*)__get_free_pages(GFP_KERNEL | GFP_DMA,
							SMI_DATA_ORDER);
	if (amibios_data == NULL)
		goto err;

	amibios_data_ptr = virt_to_phys(amibios_data);
	amibios_buffer_ptr = amibios_data_ptr + PAGE_SIZE;

#if defined(__x86_64__)
	if (amibios_data_ptr >> 32 || amibios_buffer_ptr >> 32) {
		retval = -ENOMEM;
		goto err2;
	}
#endif

	if (smi_info() < 0)
		goto err2;

	/* Being extra cautious */
	if (amibios_data->info.version < 0x24 ||
	    amibios_data->info.version > 0x28) {
		pr_err("amibios_dmi: unsupported SMBIOS version\n");
		goto err2;
	}

	if (amibios_data->info.st_num == 0) {
		pr_err("amibios_dmi: no DMI structures\n");
		goto err2;
	}

	if (amibios_data->info.st_size > PAGE_SIZE) {
		retval = -ENOMEM;
		goto err2;
	}

	*version = amibios_data->info.version;

	pr_info("amibios_dmi: SMBIOS %d.%d found (%d structures).\n",
					amibios_data->info.version >> 4,
					amibios_data->info.version & 0x0f,
					amibios_data->info.st_num);
	

	mutex_unlock(&amibios_mutex);
	return 0;

    err2:
	free_pages((unsigned long)amibios_data, SMI_DATA_ORDER);
    err:
	mutex_unlock(&amibios_mutex);
	return retval;
}

/**
 * amibios_smi_deinit() - deinitialize SMI data
 *
 * Clean up after amibios_smi_init().
 */
void amibios_smi_deinit()
{
	free_pages((unsigned long)amibios_data, SMI_DATA_ORDER);
}


/*
 * DMI structure handling
 */

static int dmi_read(struct amibios_dmi_structure_info *ds)
{
	int len;

	len = smi_read(ds->handle);
	if (len < 0)
		return -1;

	ds->header_size = amibios_data->read.data.dmi_structure.length;
	ds->size = len;

	if (ds->size < 0) {
		pr_err("ambios: DMI structure length error (handle 0x%04x)\n",
								ds->handle);
		return -1;
	}

	return 0;
}

/**
 * amibios_dmi_get_handle() - find the handle for a DMI structure type
 * @type:  the DMI structure type
 *
 * Find a handle to a structure matching the given type.
 *
 * Return: the handle if successful, or -1 in case of error
 */
int amibios_dmi_get_handle(int type)
{
	struct amibios_dmi_structure *ds;
	int next_handle;

	mutex_lock(&amibios_mutex);

	next_handle = type;
	ds = &amibios_data->read.data.dmi_structure;

	do {
		if (smi_read(next_handle) < 0)
			goto err;
		next_handle = amibios_data->read.handle;
	} while (ds->type != type && ds->type < 127);

	if (ds->type >= 127)
		goto err;

	pr_err("amibios_dmi: handle for type %d: 0x%04x\n", type, ds->handle);

	mutex_unlock(&amibios_mutex);
	return ds->handle;

    err:
	pr_err("amibios_dmi: can't find handle for type %d\n", type);
	mutex_unlock(&amibios_mutex);
	return -1;
}

/**
 * amibios_dmi_get_byte() - read a byte from a DMI structure field
 * @ds:     DMI structure metadata
 * @offset: DMI structure field to read
 * @b:      buffer to store the byte to be read
 *
 * Read a byte value from the specified field of a DMI structure. If read
 * successfully, the new byte will be stored at the location given by b.
 *
 * Return:  0 if successful, or -1 in case of error.
 */
int amibios_dmi_get_byte(struct amibios_dmi_structure_info *ds,
						int offset, u8 *b)
{
	mutex_lock(&amibios_mutex);

	if (dmi_read(ds) < 0)
		goto err;

	if (offset > ds->header_size) {
		pr_err("amibios_dmi: invalid DMI data request (handle 0x%04x "
				"offset 0x%02x)\n", ds->handle, offset);
		goto err;
	}

	*b = amibios_data->read.data.raw[offset];

	mutex_unlock(&amibios_mutex);
	return 0;

    err:
	mutex_unlock(&amibios_mutex);
	return -1;
}

/**
 * amibios_dmi_get_string() - read a string from a DMI structure field
 * @ds:     DMI structure metadata
 * @offset: DMI structure field to read
 * @s:      buffer to store the string to be read
 * @len:    size of the buffer
 *
 * Read a string value from the specified field of a DMI structure. Ifread
 * successfully, the string will be stored at the location given by s.
 * Strings larger than the buffer size it will be trucated.
 *
 * Return:  0 if successful, or -1 in case of error.
 */
int amibios_dmi_get_string(struct amibios_dmi_structure_info *ds,
					int offset, char *s, int len)
{
	char *data = amibios_data->read.data.raw;
	int i, index;

	mutex_lock(&amibios_mutex);

	if (dmi_read(ds) < 0)
		goto err;

	if (offset > ds->header_size) {
		pr_err("amibios_dmi: invalid DMI data request (handle 0x%04x "
				"offset 0x%02x)\n", ds->handle, offset);
		goto err;
	}

	index = data[offset] - 1;
	if (index < 0) {
		pr_err("amibios_dmi: invalid string number (handle 0x%04x "
				"offset 0x%02x)\n", ds->handle, offset);
		goto err;
	}

	/* Locate indexed string */
	for (i = ds->header_size; i < ds->size; i++) {
		if (index == 0)
			break;
		if (data[i] == 0) {
			if (data[i + 1] == 0)
				goto err;
			index--;
		}
	}

	strncpy(s, data + i, len); 

	mutex_unlock(&amibios_mutex);
	return 0;

    err:
	mutex_unlock(&amibios_mutex);
	return -1;
}
