/*
 * Copyright (C) 2015 Maxime Coquelin <mcoquelin.stm32@gmail.com>
 * Use of this source code is governed by a BSD-style
 * license that can be found in the LICENSE file.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <bfd.h>
#include <errno.h>

#include "stlink-common.h"
#include "uglylogging.h"

#define BUF_ALIGN	0x1000
#define STLINK_MAX_CHUNK_SIZE 0x800

/*
 * External loaders ELFs structs and enums.
 */
#define SECTOR_NUM 10

struct DeviceSectors {
  unsigned int SectorNum;
  unsigned int SectorSize;
};

struct StorageInfo {
   char DeviceName[100];
   unsigned short DeviceType;
   unsigned int DeviceStartAddress;
   unsigned int DeviceSize;
   unsigned int PageSize;
   unsigned char EraseValue;
   struct DeviceSectors	sectors[SECTOR_NUM];
};

static char *storage_types[] = {
	"N/A",
	"MCU_FLASH",
	"NAND_FLASH",
	"NOR_FLASH",
	"SRAM",
	"PSRAM",
	"PC_CARD",
	"SPI_FLASH",
	"I2C_FLASH",
	"SDRAM",
	"I2C_EEPROM",
};

struct function {
	unsigned int addr;
	unsigned int r0;
	unsigned int r1;
	unsigned int r2;
};

/*****************************************/

enum commands {
	CMD_NONE,
	CMD_WRITE,
	CMD_READ,
	CMD_ERASE,
};

static unsigned int command;
static unsigned int address;
static unsigned int read_size;
static char *loader_path;
static char *file_path;
static int debug_level = 50;
static bool noverify;
static unsigned int buf_addr;
static unsigned int buf_size;


static const struct option options[] = {
	{"help",		no_argument,		0, 'h'},
	{"write",		no_argument,		0, 'w'},
	{"read",		no_argument,		0, 'r'},
	{"erase",		no_argument,		0, 'e'},
	{"verbose",		no_argument,		0, 'v'},
	{"noverify",	no_argument,		0, 'n'},
	{"loader",		required_argument,	0, 'l'},
	{"file",		required_argument,	0, 'f'},
	{"address",		required_argument,	0, 'a'},
	{"size",		required_argument,	0, 's'},
};

static void usage(char *progname)
{
	printf("Usage:\n");
	printf("\t%s [--verbose | -v] [--noverify | -n] (--loader | -l) (--write | -w) (--file | -f) <file> (--address | -a) <address>\n", progname);
	printf("\t%s [--verbose | -v] (--loader | -l) (--read | -w) (--file | -f) <file> (--address | -a) <address> (--size | -s)\n", progname);
	printf("\t%s [--verbose | -v] (--loader | -l) (--erase| -e)\n", progname);
}

static void get_args(int argc, char **argv)
{
	int option, idx = 0;

	while ((option = getopt_long(argc, argv, "hwrevl:f:a:s:", options, &idx)) != -1) {
		switch (option) {
		case 'w':
			if (command != CMD_NONE)
				goto error;

			command = CMD_WRITE;
			break;
		case 'r':
			if (command != CMD_NONE)
				goto error;

			command = CMD_READ;
			break;
		case 'e':
			if (command != CMD_NONE)
				goto error;

			command = CMD_ERASE;
			break;
		case 'v':
			debug_level = 100;
			break;
		case 'n':
			noverify = true;
			break;
		case 'l':
			loader_path = strdup(optarg);
			break;
		case 'f':
			file_path = strdup(optarg);
			break;
		case 'a':
			address = strtol(optarg, NULL, 0);
			break;
		case 's':
			read_size = strtol(optarg, NULL, 0);
			break;
		case 'h':
		default:
			usage(argv[0]);
			exit(EXIT_FAILURE);
		}
	}

	if (!loader_path || command == CMD_NONE)
		goto error;

	if (command == CMD_WRITE && !file_path && !address)
		goto error;

	if (command == CMD_READ && !file_path && !address && !read_size)
		goto error;

	return;

error:
	usage(argv[0]);
	exit(EXIT_FAILURE);
}

static stlink_t *stlink_init_debug_mode(void)
{
	stlink_t *sl = stlink_open_usb(debug_level, 1, NULL);
	if (!sl)
		return NULL;

	if (stlink_current_mode(sl) == STLINK_DEV_DFU_MODE)
		stlink_exit_dfu_mode(sl);

	if (stlink_current_mode(sl) != STLINK_DEV_DEBUG_MODE)
		stlink_enter_swd_mode(sl);

	stlink_jtag_reset(sl,2);
	stlink_reset(sl);
    stlink_force_debug(sl);
    stlink_status(sl);

	return sl;
}

static bfd *open_loader_elf(char *path)
{
	bfd *bfd;

	bfd = bfd_openr(path, NULL);
	if (!bfd) {
		ELOG("Failed to open external loader (%s)\n", path);
		return NULL;
	}

	if (!bfd_check_format(bfd, bfd_object)) {
		if (bfd_get_error() != bfd_error_file_ambiguously_recognized) {
			ELOG("%s not an ELF file\n", path);
			return NULL;
		}
	}

	return bfd;
}

static int stlink_load_elf_section(stlink_t *sl, bfd *bfd, asection *s)
{
	size_t size, offset, remain;
	unsigned int addr;

	addr = bfd_section_lma(bfd, s);
	size = bfd_section_size(bfd, s);

	ILOG("Loading section %s: addr = 0x%08x size = %zu Bytes\n",
			bfd_section_name(bfd, s), addr, size);

	remain = size & 0x3;
	size &= ~0x3;
	for (offset = 0; offset < size; offset += STLINK_MAX_CHUNK_SIZE) {
		size_t chunk_size = STLINK_MAX_CHUNK_SIZE;
		if (offset + chunk_size > size)
			chunk_size = size - offset;

		if (!bfd_get_section_contents(bfd, s, sl->q_buf, offset, chunk_size)) {
			ELOG("Failed to get section content\n");
			return -1;
		}

		stlink_write_mem32(sl, addr + offset, chunk_size);
	}

	if (remain) {
		if (!bfd_get_section_contents(bfd, s, sl->q_buf, size, remain)) {
			ELOG("Failed to get section content\n");
			return -1;
		}

		stlink_write_mem8(sl, addr + size, remain);
	}

	return 0;

}

static int stlink_load_elf(stlink_t *sl, bfd *bfd)
{
	asection *s;
	int ret;

	for (s = bfd->sections; s; s = s->next) {
		if (!(bfd_get_section_flags(bfd, s) & SEC_LOAD))
			continue;

		ret = stlink_load_elf_section(sl, bfd, s);
		if (ret) {
			ELOG("Failed to load section %s\n", bfd_get_section_name(bfd, s));
			return -1;
		}
	}

	return 0;
}

static int stlink_get_sram_buf(stlink_t *sl, bfd *bfd)
{
	asection *s;

	buf_addr = sl->sram_base;
	buf_size = sl->sram_size;

	for (s = bfd->sections; s; s = s->next) {
		size_t size;
		unsigned int addr;

		if (!(bfd_get_section_flags(bfd, s) & SEC_LOAD))
			continue;

		addr = bfd_section_lma(bfd, s);
		size = bfd_section_size(bfd, s);

		if (addr + size <= buf_addr)
			continue;

		buf_addr = (addr + size + BUF_ALIGN) & ~(BUF_ALIGN - 1);
		buf_size = sl->sram_size - buf_addr;
	}

	if (buf_size > STLINK_MAX_CHUNK_SIZE)
		buf_size = STLINK_MAX_CHUNK_SIZE;

	ILOG("Allocated SRAM buffer: %uB @ 0x%08x\n", buf_size, buf_addr);

	return 0;
}

static unsigned int get_symbol_address(bfd *bfd, const char *name)
{
	asymbol **symtab;
	unsigned int addr;
	int i, size, nsyms;

	size = bfd_get_symtab_upper_bound(bfd);
	if (size <= 0) {
		ELOG("Failed to get ELF's symbol table size\n");
		return 0;
	}

	symtab = malloc(size);
	if (!symtab)
		return 0;

	nsyms = bfd_canonicalize_symtab(bfd, symtab);

	for (i = 0; i < nsyms; i++) {
		if (!strcmp(name, bfd_asymbol_name(symtab[i]))) {
			addr = bfd_asymbol_value(symtab[i]);
			free(symtab);
			return addr;
		}
	}

	free(symtab);
	return 0;
}

static int get_storage_info(bfd *bfd, struct StorageInfo *si)
{
	asection *s;
	unsigned int addr;

	addr = get_symbol_address(bfd, "StorageInfo");
	if (!addr) {
		ELOG("StorageInfo symbol not found in ELF\n");
		return -1;
	}

	for (s = bfd->sections; s; s = s->next) {
		if (bfd_section_lma(bfd, s) != addr)
			continue;

		if (bfd_section_size(bfd, s) < sizeof(*si))
			continue;

		if (!bfd_get_section_contents(bfd, s, si, 0, sizeof(*si))) {
			ELOG("Failed to get storage_info content\n");
			return -1;
		}

		return 0;
	}

	ELOG("StorageInfo section not found\n");

	return -1;
}

static bool in_storage_device(struct StorageInfo *si, unsigned int addr)
{
	if (addr < si->DeviceStartAddress)
		return false;
	else if (addr >= si->DeviceStartAddress + si->DeviceSize)
		return false;

	return true;
}

static bool in_first_sector(struct StorageInfo *si, unsigned int addr)
{
	if (!in_storage_device(si, addr))
		return false;

	if (addr < si->DeviceStartAddress + si->sectors[0].SectorSize)
		return true;

	return false;
}

static bool in_last_sector(struct StorageInfo *si, unsigned int addr)
{
	int i;
	unsigned int sector_size;

	if (!in_storage_device(si, addr))
		return false;

	for (i = 0; si->sectors[i + 1].SectorNum; i++);

	sector_size = si->sectors[i].SectorSize;

	if (addr >= si->DeviceStartAddress + si->DeviceSize - sector_size)
		return true;

	return false;
}

static unsigned int loader_wait_function_return(stlink_t *sl)
{
	reg regs;

	do {
		usleep(10);
	} while (!is_core_halted(sl));

	stlink_read_reg(sl, 0, &regs);

	return regs.r[0];
}

static unsigned int loader_function_call(stlink_t *sl, struct function *f)
{
	stlink_write_reg(sl, f->r0, 0);
	stlink_write_reg(sl, f->r1, 1);
	stlink_write_reg(sl, f->r2, 2);
	stlink_write_reg(sl, sl->sram_base | 1, 14);
	stlink_write_reg(sl, f->addr, 15);

	stlink_run(sl);

	return loader_wait_function_return(sl);
}

static int do_Verify(stlink_t *sl, bfd *bfd, unsigned int to, unsigned int from, unsigned int len)
{
	struct function func;
	unsigned int ret;

	func.addr = get_symbol_address(bfd, "Verify");
	if (!func.addr) {
		ELOG("Failed to get Verify() address\n");
		return -1;
	}

	func.r0 = to;
	func.r1 = from;
	func.r2 = len / 4;

	ret = loader_function_call(sl, &func);
	if (ret) {
		ELOG("Verification failed at address 0x%08x\n", ret);
		return -1;
	}

	return 0;
}

static int do_Read(stlink_t *sl, bfd *bfd, unsigned int from, unsigned int to, unsigned int len)
{
	struct function func;

	func.addr = get_symbol_address(bfd, "Read");
	if (!func.addr) {
		ELOG("Failed to get Read() address\n");
		return -1;
	}

	func.r0 = from;
	func.r1 = len;
	func.r2 = to;

	if (loader_function_call(sl, &func) != 1) {
		ELOG("Loader failed to perform Read\n");
		return -1;
	}

	return 0;
}

static int do_Write(stlink_t *sl, bfd *bfd, unsigned int to, unsigned int from, unsigned int len)
{
	struct function func;

	func.addr = get_symbol_address(bfd, "Write");
	if (!func.addr) {
		ELOG("Failed to get Write() address\n");
		return -1;
	}

	func.r0 = to;
	func.r1 = len;
	func.r2 = from;

	if (loader_function_call(sl, &func) != 1) {
		ELOG("Loader failed to perform Write\n");
		return -1;
	}

	return 0;
}

static int do_SectorErase(stlink_t *sl, bfd *bfd, unsigned int start, unsigned int end)
{
	struct function func;

	func.addr = get_symbol_address(bfd, "SectorErase");
	if (!func.addr) {
		ELOG("Failed to get SectorErase() address\n");
		return -1;
	}

	func.r0 = start;
	func.r1 = end;

	if (loader_function_call(sl, &func) != 1) {
		ELOG("Loader failed to perform Sector Erase\n");
		return -1;
	}

	return 0;
}

static int do_MassErase(stlink_t *sl, bfd *bfd)
{
	struct function func;

	func.addr = get_symbol_address(bfd, "MassErase");
	if (!func.addr) {
		ELOG("Failed to get MassErase() address\n");
		return -1;
	}

	if (loader_function_call(sl, &func) != 1) {
		ELOG("Loader failed to Mass Erase\n");
		return -1;
	}

	return 0;
}

static int do_Init(stlink_t *sl, bfd *bfd)
{
	struct function func;

	func.addr = get_symbol_address(bfd, "Init");
	if (!func.addr) {
		ELOG("Failed to get Init() address\n");
		return -1;
	}

	/* Write break opcode */
	sl->q_buf[0] = 0x00;
	sl->q_buf[1] = 0xbe;
	stlink_write_mem8(sl, sl->sram_base, 2);

	if (loader_function_call(sl, &func) != 1) {
		ELOG("External loader initialization failed\n");
		return -1;
	}

	return 0;
}

static int cmd_MassErase(stlink_t *sl, bfd *bfd)
{
	int ret;

	ILOG("Starting mass-erase (may take several minutes)...\n");

	ret = do_Init(sl, bfd);
	if (ret)
		goto err;

	ret = do_MassErase(sl, bfd);
	if (ret)
		goto err;

	ILOG("Mass Erase done\n");
	return 0;

err:
	ELOG("Mass Erase failed\n");
	return ret;
}

static int cmd_WriteSector(stlink_t *sl, bfd *bfd, unsigned int to,
						unsigned char *from, unsigned int size)
{
	int ret;

	while (size > 0x3) {
		unsigned int len = size > buf_size ? buf_size : size;

		ret = do_Init(sl, bfd);
		if (ret)
			return ret;

		len &= ~0x3;
		memcpy(sl->q_buf, from, len);

		stlink_write_mem32(sl, buf_addr, len);

		ret = do_Write(sl, bfd, to, buf_addr, len);
		if (ret)
			return ret;

		if (!noverify) {
			ret = do_Verify(sl, bfd, to, buf_addr, len);
			if (ret)
				return ret;
		}

		size -= len;
		to += len;
		from += len;
	}

	if (size) {
		ret = do_Init(sl, bfd);
		if (ret)
			return ret;

		memcpy(sl->q_buf, from, size);
		stlink_write_mem8(sl, buf_addr, size);

		ret = do_Write(sl, bfd, to, buf_addr, size);
		if (ret)
			return ret;

		if (!noverify) {
			ret = do_Verify(sl, bfd, to, buf_addr, size);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int cmd_Write(stlink_t *sl, bfd *bfd, struct StorageInfo *si,
						char *path, unsigned int addr)
{
	int i, k, ret;
	unsigned int j, len, sect_addr, left, offset;
	mapped_file_t mf = MAPPED_FILE_INITIALIZER;

	ret = map_file(&mf, path);
	if (ret) {
		ELOG("Failed to mmap %s\n", path);
		goto unmap;
	}

	len = (unsigned int)mf.len;

	ILOG("Writing file %s (%u Bytes) from 0x%08x to 0x%08x...\n", path, len, addr, addr + len);

	if (!in_storage_device(si, addr)) {
		ELOG("Start address (0x%08x) out of device range\n", addr);
		ret = -1;
		goto unmap;
	} else if (!in_storage_device(si, addr + len - 1)) {
		ELOG("End address (0x%08x) out of device range\n", addr + len - 1);
		ret = -1;
		goto unmap;
	}

	ret = do_Init(sl, bfd);
	if (ret)
		goto unmap;

	ILOG("Erasing from 0x%08x to 0x%08x (may take several minutes)...\n", addr, addr + len);

	/*
	 * If all sectors have to be erased, perform a mass erase which
	 * is almost instantaneous with some memories/loaders.
	 */
	if (in_first_sector(si, addr) && in_last_sector(si, addr + len - 1))
		ret = do_MassErase(sl, bfd);
	else
		ret = do_SectorErase(sl, bfd, addr, addr + len);

	if (ret)
		goto unmap;

	sect_addr = (unsigned int)si->DeviceStartAddress;
	left = len;
	offset = 0;

	for (i = 0, k = 0; si->sectors[i].SectorNum && left; i++) {
		for (j = 0; j < si->sectors[i].SectorNum && left; j++, k++) {
			unsigned int size = si->sectors[i].SectorSize;

			if (addr > sect_addr + size - 1) {
				sect_addr += size;
				continue;
			}

			if (addr > sect_addr) {
				size -= addr - sect_addr;
				sect_addr = addr;
			}

			if (size > left)
				size = left;

			ILOG("Writing sector %d (from 0x%08x to 0x%08x)\n", k, sect_addr, sect_addr + size - 1);

			ret = cmd_WriteSector(sl, bfd, sect_addr, mf.base + offset, size);
			if (ret)
				goto unmap;

			sect_addr += size;
			offset += size;
			left -= size;
		}
	}

	ILOG("Write operation succeeded (With%s verification done).\n", noverify ? "out":"");

unmap:
	unmap_file(&mf);
	return ret;
}

static int cmd_ReadSector(stlink_t *sl, bfd *bfd, unsigned int from,
						int fd, unsigned int size)
{
	int ret;
	unsigned int read_addr;
	bool has_read = get_symbol_address(bfd, "Read")? true : false;

	while (size > 0x3) {
		unsigned int len = size > buf_size ? buf_size : size;

		len &= ~0x3;

		if (has_read) {
			ret = do_Read(sl, bfd, from, buf_addr, len);
			if (ret)
				return ret;
			read_addr = buf_addr;
		} else {
			read_addr = from;
		}

		stlink_read_mem32(sl, read_addr, len);
		ret = write(fd, sl->q_buf, len);
		if (ret == -1) {
			ELOG("Failed to write to %s (%s)\n", file_path, strerror(errno));
			return ret;
		}

		size -= len;
		from += len;
	}

	if (size) {
		if (has_read) {
			ret = do_Read(sl, bfd, from, buf_addr, size);
			if (ret)
				return ret;
			read_addr = buf_addr;
		} else {
			read_addr = from;
		}

		stlink_read_mem32(sl, read_addr, size);
		ret = write(fd, sl->q_buf, size);
		if (ret == -1) {
			ELOG("Failed to write to %s\n", file_path);
			return ret;
		}
	}

	return 0;
}

static int cmd_Read(stlink_t *sl, bfd *bfd, struct StorageInfo *si,
						char *path, unsigned int addr, unsigned int size)
{
	int ret, fd, i, k;
	unsigned int sect_addr, left, offset, j;

	fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, S_IWUSR | S_IRUSR);
	if (fd < 0) {
		ELOG("Failed to create file: %s\n", path);
		return -1;
	}

	ILOG("Reading %u Bytes from 0x%08x in %s...\n", size, addr, path);

	if (!in_storage_device(si, addr)) {
		ELOG("Start address (0x%08x) out of device range\n", addr);
		ret = -1;
		goto close;
	} else if (!in_storage_device(si, addr + size - 1)) {
		ELOG("End address (0x%08x) out of device range\n", addr + size - 1);
		ret = -1;
		goto close;
	}

	ret = do_Init(sl, bfd);
	if (ret)
		goto close;

	sect_addr = (unsigned int)si->DeviceStartAddress;
	left = size;
	offset = 0;

	for (i = 0, k = 0; si->sectors[i].SectorNum && left; i++) {
		for (j = 0; j < si->sectors[i].SectorNum && left; j++, k++) {
			unsigned int sect_size = si->sectors[i].SectorSize;

			if (addr > sect_addr + sect_size - 1) {
				sect_addr += sect_size;
				continue;
			}

			if (addr > sect_addr) {
				sect_size -= addr - sect_addr;
				sect_addr = addr;
			}

			if (sect_size > left)
				sect_size = left;

			ILOG("Reading sector %d (from 0x%08x to 0x%08x)\n",
					k, sect_addr, sect_addr + sect_size - 1);

			ret = cmd_ReadSector(sl, bfd, sect_addr, fd, sect_size);
			if (ret)
				goto close;

			sect_addr += sect_size;
			offset += sect_size;
			left -= sect_size;
		}
	}

	ILOG("Read operation succeeded.\n");

close:
	close(fd);
	return 0;
}

int main(int argc, char **argv)
{
	stlink_t *sl;
	bfd *bfd;
	struct StorageInfo si;
	int i, ret = EXIT_SUCCESS;

	get_args(argc, argv);

	sl = stlink_init_debug_mode();
	if (!sl)
		exit(EXIT_FAILURE);

	bfd = open_loader_elf(loader_path);
	if (!bfd) {
		ret = EXIT_FAILURE;
		goto out_stlink;
	}

	ret = get_storage_info(bfd, &si);
	if (ret) {
		ret = EXIT_FAILURE;
		goto out_bfd;
	}

	ILOG("External storage device information:\n");
	ILOG("  Model: %s\n", si.DeviceName);
	ILOG("  Type: %s\n", storage_types[si.DeviceType]);
	ILOG("  Address: 0x%08x\n", si.DeviceStartAddress);
	ILOG("  Size: %uMB\n", si.DeviceSize / 1024 / 1024);
	ILOG("  Page size: %uKB\n", si.PageSize / 1024);
	ILOG("  Sectors:\n");
	for (i = 0; si.sectors[i].SectorNum; i++)
		ILOG("    %d sectors of %uKB\n",
				si.sectors[i].SectorNum,
				si.sectors[i].SectorSize / 1024);

	ret = stlink_load_elf(sl, bfd);
	if (ret) {
		ret = EXIT_FAILURE;
		goto out_bfd;
	}

	switch (command) {
	case CMD_ERASE:
		ret = cmd_MassErase(sl, bfd);
		if (ret)
			ret = EXIT_FAILURE;
		break;
	case CMD_WRITE:
		ret = stlink_get_sram_buf(sl, bfd);
		if (ret) {
			ret = EXIT_FAILURE;
			break;
		}

		ret = cmd_Write(sl, bfd, &si, file_path, address);
		if (ret)
			ret = EXIT_FAILURE;
		break;
	case CMD_READ:
		ret = stlink_get_sram_buf(sl, bfd);
		if (ret) {
			ret = EXIT_FAILURE;
			break;
		}

		ret = cmd_Read(sl, bfd, &si, file_path, address, read_size);
		if (ret)
			ret = EXIT_FAILURE;
		break;
	default:
		ELOG("Command not yet implemented\n");
	}

out_bfd:
	bfd_close(bfd);
out_stlink:
	stlink_close(sl);

	return ret;
}
