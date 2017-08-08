/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright 2010 Geoffrey McRae <geoff@spacevs.com>
  Copyright 2011 Steve Markgraf <steve@steve-m.de>
  Copyright 2012-2016 Tormod Volden <debian.tormod@gmail.com>
  Copyright 2013-2016 Antonio Borneo <borneo.antonio@gmail.com>
  Copyright (C) 2017 Tinder Smith <coflery@gmail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "init.h"
#include "utils.h"
#include "serial.h"
#include "stm32.h"
#include "parsers/parser.h"
#include "port.h"

#include "parsers/binary.h"
#include "parsers/hex.h"

#define VERSION "1.1"

/* device globals */
stm32_t		*stm		= NULL;

void		*p_st		= NULL;
parser_t	*parser		= NULL;

/* settings */
struct port_options port_opts = {
	.device			= NULL,
	.baudRate		= SERIAL_BAUD_57600,
	.serial_mode		= "8e1",
	.bus_addr		= 0,
	.rx_frame_max		= STM32_MAX_RX_FRAME,
	.tx_frame_max		= STM32_MAX_TX_FRAME,
};

enum actions {
	ACT_NONE,
	ACT_READ,
	ACT_WRITE,
	ACT_WRITE_UNPROTECT,
	ACT_READ_PROTECT,
	ACT_READ_UNPROTECT,
	ACT_ERASE_ONLY,
	ACT_CRC
};

enum actions	action		= ACT_NONE;
int		npages		= 0;
int             spage           = 0;
int             no_erase        = 0;
char		verify		= 0;
int		retry		= 10;
char		exec_flag	= 0;
uint32_t	execute		= 0;
char		init_flag	= 1;
char		force_binary	= 0;
char		reset_flag	= 0;
char		*filename;
char		*gpio_seq	= NULL;
uint32_t	start_addr	= 0;
uint32_t	readwrite_len	= 0;

/* functions */
int  parse_options(int argc, char *argv[]);
void show_help(char *name);

static const char *action2str(enum actions act)
{
	switch (act) {
		case ACT_READ:
			return "读出内存";
		case ACT_WRITE:
			return "写入内存";
		case ACT_WRITE_UNPROTECT:
			return "写入保护";
		case ACT_READ_PROTECT:
			return "读 保护";
		case ACT_READ_UNPROTECT:
			return "读 未保护";
		case ACT_ERASE_ONLY:
			return "擦除flash";
		case ACT_CRC:
			return "内存 crc";
		default:
			return "";
	};
}

static void err_multi_action(enum actions new)
{
	fprintf(stderr,
		"错误: 无效选项 !\n"
		"\t不能执行将 \"%s\" 和 \"%s\" 同时执行.\n",
		action2str(action), action2str(new));
}

static int is_addr_in_ram(uint32_t addr)
{
	return addr >= stm->dev->ram_start && addr < stm->dev->ram_end;
}

static int is_addr_in_flash(uint32_t addr)
{
	return addr >= stm->dev->fl_start && addr < stm->dev->fl_end;
}

/* returns the page that contains address "addr" */
static int flash_addr_to_page_floor(uint32_t addr)
{
	int page;
	uint32_t *psize;

	if (!is_addr_in_flash(addr))
		return 0;

	page = 0;
	addr -= stm->dev->fl_start;
	psize = stm->dev->fl_ps;

	while (addr >= psize[0]) {
		addr -= psize[0];
		page++;
		if (psize[1])
			psize++;
	}

	return page;
}

/* returns the first page whose start addr is >= "addr" */
int flash_addr_to_page_ceil(uint32_t addr)
{
	int page;
	uint32_t *psize;

	if (!(addr >= stm->dev->fl_start && addr <= stm->dev->fl_end))
		return 0;

	page = 0;
	addr -= stm->dev->fl_start;
	psize = stm->dev->fl_ps;

	while (addr >= psize[0]) {
		addr -= psize[0];
		page++;
		if (psize[1])
			psize++;
	}

	return addr ? page + 1 : page;
}

/* returns the lower address of flash page "page" */
static uint32_t flash_page_to_addr(int page)
{
	int i;
	uint32_t addr, *psize;

	addr = stm->dev->fl_start;
	psize = stm->dev->fl_ps;

	for (i = 0; i < page; i++) {
		addr += psize[0];
		if (psize[1])
			psize++;
	}

	return addr;
}

int main(int argc, char* argv[]) {
	struct port_interface *port = NULL;
	int ret = 1;
	stm32_err_t s_err;
	parser_err_t perr;
	FILE *diag = stdout;

	fprintf(diag, "stm32flash " VERSION "\n\n");
	fprintf(diag, "任复衡20170807 http://github.com/coflery\n\n");
	if (parse_options(argc, argv) != 0)
		goto close;

	if ((action == ACT_READ) && filename[0] == '-') {
		diag = stderr;
	}

	if (action == ACT_WRITE) {
		/* first try hex */
		if (!force_binary) {
			parser = &PARSER_HEX;
			p_st = parser->init();
			if (!p_st) {
				fprintf(stderr, "%s 初始化解析失败\n", parser->name);
				goto close;
			}
		}

		if (force_binary || (perr = parser->open(p_st, filename, 0)) != PARSER_ERR_OK) {
			if (force_binary || perr == PARSER_ERR_INVALID_FILE) {
				if (!force_binary) {
					parser->close(p_st);
					p_st = NULL;
				}

				/* now try binary */
				parser = &PARSER_BINARY;
				p_st = parser->init();
				if (!p_st) {
					fprintf(stderr, "%s 初始化解析失败\n", parser->name);
					goto close;
				}
				perr = parser->open(p_st, filename, 0);
			}

			/* if still have an error, fail */
			if (perr != PARSER_ERR_OK) {
				fprintf(stderr, "%s 错误: %s\n", parser->name, parser_errstr(perr));
				if (perr == PARSER_ERR_SYSTEM) perror(filename);
				goto close;
			}
		}

		fprintf(diag, "使用解析 : %s\n", parser->name);
	} else {
		parser = &PARSER_BINARY;
		p_st = parser->init();
		if (!p_st) {
			fprintf(stderr, "%s 初始化解析失败\n", parser->name);
			goto close;
		}
	}

	if (port_open(&port_opts, &port) != PORT_ERR_OK) {
		fprintf(stderr, "打开端口失败: %s\n", port_opts.device);
		goto close;
	}

	fprintf(diag, "通讯口 %s: %s\n", port->name, port->get_cfg_str(port));
	if (init_flag && init_bl_entry(port, gpio_seq) == 0)
		goto close;
	stm = stm32_init(port, init_flag);
	if (!stm)
		goto close;

	fprintf(diag, "版本         : 0x%02x\n", stm->bl_version);
	if (port->flags & PORT_GVR_ETX) {
		fprintf(diag, "Option 1     : 0x%02x\n", stm->option1);
		fprintf(diag, "Option 2     : 0x%02x\n", stm->option2);
	}
	fprintf(diag, "设备 ID      : 0x%04x (%s)\n", stm->pid, stm->dev->name);
	fprintf(diag, "- RAM        : %dKiB  (bootloader 保留 %d bit )\n", (stm->dev->ram_end - 0x20000000) / 1024, stm->dev->ram_start - 0x20000000);
	fprintf(diag, "- Flash      : %dKiB (首扇区大小: %dx%d)\n", (stm->dev->fl_end - stm->dev->fl_start ) / 1024, stm->dev->fl_pps, stm->dev->fl_ps[0]);
	fprintf(diag, "- Option RAM : %db\n", stm->dev->opt_end - stm->dev->opt_start + 1);
	fprintf(diag, "- System RAM : %dKiB\n", (stm->dev->mem_end - stm->dev->mem_start) / 1024);

	uint8_t		buffer[256];
	uint32_t	addr, start, end;
	unsigned int	len;
	int		failed = 0;
	int		first_page, num_pages;

	/*
	 * Cleanup addresses:
	 *
	 * Starting from options
	 *	start_addr, readwrite_len, spage, npages
	 * and using device memory size, compute
	 *	start, end, first_page, num_pages
	 */
	if (start_addr || readwrite_len) {
		start = start_addr;

		if (is_addr_in_flash(start))
			end = stm->dev->fl_end;
		else {
			no_erase = 1;
			if (is_addr_in_ram(start))
				end = stm->dev->ram_end;
			else
				end = start + sizeof(uint32_t);
		}

		if (readwrite_len && (end > start + readwrite_len))
			end = start + readwrite_len;

		first_page = flash_addr_to_page_floor(start);
		if (!first_page && end == stm->dev->fl_end)
			num_pages = STM32_MASS_ERASE;
		else
			num_pages = flash_addr_to_page_ceil(end) - first_page;
	} else if (!spage && !npages) {
		start = stm->dev->fl_start;
		end = stm->dev->fl_end;
		first_page = 0;
		num_pages = STM32_MASS_ERASE;
	} else {
		first_page = spage;
		start = flash_page_to_addr(first_page);
		if (start > stm->dev->fl_end) {
			fprintf(stderr, "地址范围超出flash容量.\n");
			goto close;
		}

		if (npages) {
			num_pages = npages;
			end = flash_page_to_addr(first_page + num_pages);
			if (end > stm->dev->fl_end)
				end = stm->dev->fl_end;
		} else {
			end = stm->dev->fl_end;
			num_pages = flash_addr_to_page_ceil(end) - first_page;
		}

		if (!first_page && end == stm->dev->fl_end)
			num_pages = STM32_MASS_ERASE;
	}

	if (action == ACT_READ) {
		unsigned int max_len = port_opts.rx_frame_max;

		fprintf(diag, "读取内存\n");

		perr = parser->open(p_st, filename, 1);
		if (perr != PARSER_ERR_OK) {
			fprintf(stderr, "%s 错误: %s\n", parser->name, parser_errstr(perr));
			if (perr == PARSER_ERR_SYSTEM)
				perror(filename);
			goto close;
		}

		fflush(diag);
		addr = start;
		while(addr < end) {
			uint32_t left	= end - addr;
			len		= max_len > left ? left : max_len;
			s_err = stm32_read_memory(stm, addr, buffer, len);
			if (s_err != STM32_ERR_OK) {
				fprintf(stderr, "读取地址 0x%08x 处的内存失败, 目标设备写保护吗?\n", addr);
				goto close;
			}
			if (parser->write(p_st, buffer, len) != PARSER_ERR_OK)
			{
				fprintf(stderr, "将数据写入文件失败\n");
				goto close;
			}
			addr += len;

			fprintf(diag,
				"\r读取地址 0x%08x (%.2f%%) ",
				addr,
				(100.0f / (float)(end - start)) * (float)(addr - start)
			);
			fflush(diag);
		}
		fprintf(diag,	"完成.\n");
		ret = 0;
		goto close;
	} else if (action == ACT_READ_PROTECT) {
		fprintf(stdout, "Flash 有读保护\n");
		/* the device automatically performs a reset after the sending the ACK */
		reset_flag = 0;
		stm32_readprot_memory(stm);
		fprintf(stdout,	"完成.\n");
	} else if (action == ACT_READ_UNPROTECT) {
		fprintf(stdout, "Flash 无读保护\n");
		/* the device automatically performs a reset after the sending the ACK */
		reset_flag = 0;
		stm32_runprot_memory(stm);
		fprintf(stdout,	"完成.\n");
	} else if (action == ACT_ERASE_ONLY) {
		ret = 0;
		fprintf(stdout, "擦除 flash\n");

		if (num_pages != STM32_MASS_ERASE &&
		    (start != flash_page_to_addr(first_page)
		     || end != flash_page_to_addr(first_page + num_pages))) {
			fprintf(stderr, "设置 开始 & 长度无效 (必须页对其)\n");
			ret = 1;
			goto close;
		}

		s_err = stm32_erase_memory(stm, first_page, num_pages);
		if (s_err != STM32_ERR_OK) {
			fprintf(stderr, "擦除内存失败\n");
			ret = 1;
			goto close;
		}
	} else if (action == ACT_WRITE_UNPROTECT) {
		fprintf(diag, "Flash 无写保护\n");
		/* the device automatically performs a reset after the sending the ACK */
		reset_flag = 0;
		stm32_wunprot_memory(stm);
		fprintf(diag,	"完成.\n");

	} else if (action == ACT_WRITE) {
		fprintf(diag, "写内存\n");

		off_t 	offset = 0;
		ssize_t r;
		unsigned int size;
		unsigned int max_wlen, max_rlen;

		max_wlen = port_opts.tx_frame_max - 2;	/* skip len and crc */
		max_wlen &= ~3;	/* 32 bit aligned */

		max_rlen = port_opts.rx_frame_max;
		max_rlen = max_rlen < max_wlen ? max_rlen : max_wlen;

		/* Assume data from stdin is whole device */
		if (filename[0] == '-' && filename[1] == '\0')
			size = end - start;
		else
			size = parser->size(p_st);

		// TODO: It is possible to write to non-page boundaries, by reading out flash
		//       from partial pages and combining with the input data
		// if ((start % stm->dev->fl_ps[i]) != 0 || (end % stm->dev->fl_ps[i]) != 0) {
		//	fprintf(stderr, "设定的起始 & 长度值无效 (必须页对其)\n");
		//	goto close;
		// }

		// TODO: If writes are not page aligned, we should probably read out existing flash
		//       contents first, so it can be preserved and combined with new data
		if (!no_erase && num_pages) {
			fprintf(diag, "擦除内存\n");
			s_err = stm32_erase_memory(stm, first_page, num_pages);
			if (s_err != STM32_ERR_OK) {
				fprintf(stderr, "内存擦除失败\n");
				goto close;
			}
		}

		fflush(diag);
		addr = start;
		while(addr < end && offset < size) {
			uint32_t left	= end - addr;
			len		= max_wlen > left ? left : max_wlen;
			len		= len > size - offset ? size - offset : len;

			if (parser->read(p_st, buffer, &len) != PARSER_ERR_OK)
				goto close;

			if (len == 0) {
				if (filename[0] == '-') {
					break;
				} else {
					fprintf(stderr, "读取输入的文件失败\n");
					goto close;
				}
			}

			again:
			s_err = stm32_write_memory(stm, addr, buffer, len);
			if (s_err != STM32_ERR_OK) {
				fprintf(stderr, "写入地址0x%08x的内存失败\n", addr);
				goto close;
			}

			if (verify) {
				uint8_t compare[len];
				unsigned int offset, rlen;

				offset = 0;
				while (offset < len) {
					rlen = len - offset;
					rlen = rlen < max_rlen ? rlen : max_rlen;
					s_err = stm32_read_memory(stm, addr + offset, compare + offset, rlen);
					if (s_err != STM32_ERR_OK) {
						fprintf(stderr, "读取地址0x%08x的内存失败\n", addr + offset);
						goto close;
					}
					offset += rlen;
				}

				for(r = 0; r < len; ++r)
					if (buffer[r] != compare[r]) {
						if (failed == retry) {
							fprintf(stderr, "0x%08x地址校验失败, 应该为 0x%02x 但是实际为 0x%02x\n",
								(uint32_t)(addr + r),
								buffer [r],
								compare[r]
							);
							goto close;
						}
						++failed;
						goto again;
					}

				failed = 0;
			}

			addr	+= len;
			offset	+= len;

			fprintf(diag,
				"\r写%s地址 0x%08x (%.2f%%) \r\n",
				verify ? "/校验 " : "",
				addr,
				(100.0f / size) * offset
			);
			fflush(diag);

		}

		fprintf(diag,	"完成.\n");
		ret = 0;
		goto close;
	} else if (action == ACT_CRC) {
		uint32_t crc_val = 0;

		fprintf(diag, "CRC 计算中\n");

		s_err = stm32_crc_wrapper(stm, start, end - start, &crc_val);
		if (s_err != STM32_ERR_OK) {
			fprintf(stderr, "读取CRC失败\n");
			goto close;
		}
		fprintf(diag, "CRC(0x%08x-0x%08x) = 0x%08x\n", start, end,
			crc_val);
		ret = 0;
		goto close;
	} else
		ret = 0;

close:
	if (stm && exec_flag && ret == 0) {
		if (execute == 0)
			execute = stm->dev->fl_start;

		fprintf(diag, "\n开始从 0x%08x 地址处执行... ", execute);
		fflush(diag);
		if (stm32_go(stm, execute) == STM32_ERR_OK) {
			reset_flag = 0;
			fprintf(diag, "完成.\n");
		} else
			fprintf(diag, "失败.\n");
	}

	if (stm && reset_flag) {
		fprintf(diag, "\n复位设备... ");
		fflush(diag);
		if (init_bl_exit(stm, port, gpio_seq))
			fprintf(diag, "完成.\n");
		else	fprintf(diag, "失败.\n");
	}

	if (p_st  ) parser->close(p_st);
	if (stm   ) stm32_close  (stm);
	if (port)
		port->close(port);

	fprintf(diag, "\n");
	return ret;
}

int parse_options(int argc, char *argv[])
{
	int c;
	char *pLen;

	while ((c = getopt(argc, argv, "a:b:m:r:w:e:vn:g:jkfcChuos:S:F:i:R")) != -1) {
		switch(c) {
			case 'a':
				port_opts.bus_addr = strtoul(optarg, NULL, 0);
				break;

			case 'b':
				port_opts.baudRate = serial_get_baud(strtoul(optarg, NULL, 0));
				if (port_opts.baudRate == SERIAL_BAUD_INVALID) {
					serial_baud_t baudrate;
					fprintf(stderr,	"无效波特率，有效值取值为:\n");
					for (baudrate = SERIAL_BAUD_1200; baudrate != SERIAL_BAUD_INVALID; ++baudrate)
						fprintf(stderr, " %d\n", serial_get_baud_int(baudrate));
					return 1;
				}
				break;

			case 'm':
				if (strlen(optarg) != 3
					|| serial_get_bits(optarg) == SERIAL_BITS_INVALID
					|| serial_get_parity(optarg) == SERIAL_PARITY_INVALID
					|| serial_get_stopbit(optarg) == SERIAL_STOPBIT_INVALID) {
					fprintf(stderr, "串口模式无效\n");
					return 1;
				}
				port_opts.serial_mode = optarg;
				break;

			case 'r':
			case 'w':
				if (action != ACT_NONE) {
					err_multi_action((c == 'r') ? ACT_READ : ACT_WRITE);
					return 1;
				}
				action = (c == 'r') ? ACT_READ : ACT_WRITE;
				filename = optarg;
				if (filename[0] == '-') {
					force_binary = 1;
				}
				break;
			case 'e':
				if (readwrite_len || start_addr) {
					fprintf(stderr, "错误: 选项无效, 不能设定开始页 / 页数 以及开始地址/长度\n");
					return 1;
				}
				npages = strtoul(optarg, NULL, 0);
				if (npages > 0xFF || npages < 0) {
					fprintf(stderr, "\n错误: 你需要指定一个介于0到255间的页数");
					return 1;
				}
				if (!npages)
					no_erase = 1;
				break;
			case 'u':
				if (action != ACT_NONE) {
					err_multi_action(ACT_WRITE_UNPROTECT);
					return 1;
				}
				action = ACT_WRITE_UNPROTECT;
				break;

			case 'j':
				if (action != ACT_NONE) {
					err_multi_action(ACT_READ_PROTECT);
					return 1;
				}
				action = ACT_READ_PROTECT;
				break;

			case 'k':
				if (action != ACT_NONE) {
					err_multi_action(ACT_READ_UNPROTECT);
					return 1;
				}
				action = ACT_READ_UNPROTECT;
				break;

			case 'o':
				if (action != ACT_NONE) {
					err_multi_action(ACT_ERASE_ONLY);
					return 1;
				}
				action = ACT_ERASE_ONLY;
				break;

			case 'v':
				verify = 1;
				break;

			case 'n':
				retry = strtoul(optarg, NULL, 0);
				break;

			case 'g':
				exec_flag = 1;
				execute   = strtoul(optarg, NULL, 0);
				if (execute % 4 != 0) {
					fprintf(stderr, "错误: 执行地址一定要字对齐\n");
					return 1;
				}
				break;
			case 's':
				if (readwrite_len || start_addr) {
					fprintf(stderr, "错误: 参数无效, 没有指定起始页 / 数字页 和 起始地址/长度\n");
					return 1;
				}
				spage    = strtoul(optarg, NULL, 0);
				break;
			case 'S':
				if (spage || npages) {
					fprintf(stderr, "错误: 参数无效, 没有指定起始页  /数字页 和 起始地址/长度\n");
					return 1;
				} else {
					start_addr = strtoul(optarg, &pLen, 0);
					if (*pLen == ':') {
						pLen++;
						readwrite_len = strtoul(pLen, NULL, 0);
						if (readwrite_len == 0) {
							fprintf(stderr, "错误: 参数无效, 长度不能为0\n");
							return 1;
						}
					}
				}
				break;
			case 'F':
				port_opts.rx_frame_max = strtoul(optarg, &pLen, 0);
				if (*pLen == ':') {
					pLen++;
					port_opts.tx_frame_max = strtoul(pLen, NULL, 0);
				}
				if (port_opts.rx_frame_max < 0
				    || port_opts.tx_frame_max < 0) {
					fprintf(stderr, "错误: 该参数不能为负值 -F\n");
					return 1;
				}
				if (port_opts.rx_frame_max == 0)
					port_opts.rx_frame_max = STM32_MAX_RX_FRAME;
				if (port_opts.tx_frame_max == 0)
					port_opts.tx_frame_max = STM32_MAX_TX_FRAME;
				if (port_opts.rx_frame_max < 20
				    || port_opts.tx_frame_max < 6) {
					fprintf(stderr, "错误: 当前代码不能工作于小帧模式.\n");
					fprintf(stderr, "min(RX) = 20, min(TX) = 6\n");
					return 1;
				}
				if (port_opts.rx_frame_max > STM32_MAX_RX_FRAME) {
					fprintf(stderr, "警告: RX 帧长度参数被忽略 -F\n");
					port_opts.rx_frame_max = STM32_MAX_RX_FRAME;
				}
				if (port_opts.tx_frame_max > STM32_MAX_TX_FRAME) {
					fprintf(stderr, "警告: TX 帧长度参数被忽略 -F\n");
					port_opts.tx_frame_max = STM32_MAX_TX_FRAME;
				}
				break;
			case 'f':
				force_binary = 1;
				break;

			case 'c':
				init_flag = 0;
				break;

			case 'h':
				show_help(argv[0]);
				exit(0);

			case 'i':
				gpio_seq = optarg;
				break;

			case 'R':
				reset_flag = 1;
				break;

			case 'C':
				if (action != ACT_NONE) {
					err_multi_action(ACT_CRC);
					return 1;
				}
				action = ACT_CRC;
				break;
		}
	}

	for (c = optind; c < argc; ++c) {
		if (port_opts.device) {
			fprintf(stderr, "错误: 设定参数无效\n");
			show_help(argv[0]);
			return 1;
		}
		port_opts.device = argv[c];
	}

	if (port_opts.device == NULL) {
		fprintf(stderr, "错误: 没有指定设备\n");
		show_help(argv[0]);
		return 1;
	}

	if ((action != ACT_WRITE) && verify) {
		fprintf(stderr, "错误: 无效用法, -v 参数只有在写入时有效\n");
		show_help(argv[0]);
		return 1;
	}

	return 0;
}

void show_help(char *name) {
	fprintf(stderr,
		"用法: %s [-bvngfhc] [-[rw] 文件名] [tty_设备 | i2c_设备]\n\n"
		"	-a 总线地址	I2C总线地址 (例如用于 I2C 口)\n"
		"	-b 波特率	波特率 (默认 57600)\n"
		"	-m 模式		串口模式 (默认 8e1)\n"
		"	-r 文件名	从flash读到文件 (或 - stdout)\n"
		"	-w 文件名	写flash到文件 (或 - stdout)\n"
		"	-C		计算flash content的CRC\n"
		"	-u		关闭flash写保护\n"
		"	-j		打开flash写保护\n"
		"	-k		关闭flash读保护\n"
		"	-o		仅擦除\n"
		"	-e n		在写到flash前仅擦除 n 页\n"
		"	-v		校验写入的数据\n"
		"	-n 次数	写入失败的时候的重试次数(默认值 10)\n"
		"	-g 地址	从指定地址处开始执行程序(0 = flash开始地址)\n"
		"	-S 地址[:长度]	指定开始 读/写/擦除 操作的地址\n"
		"	                   	    以及相应的长度\n"
		"	-F 接收长度[:发送长度]  指定 RX 和 TX 的帧最大长度\n"
		"	-s 起始页	设定flash开始的页地址 (0 = flash 开始)\n"
		"	-f		强制使用二进制解析\n"
		"	-h		显示本帮助\n"
		"	-c		恢复连接 (不发送初始化 INIT)\n"
		"			*波特率必须和第一次初始化的时的值保持一致*\n"
		"			这对于复位失败时很重要\n"
		"	-i GPIO_string	用于 进入/退出 bootloader 模式的GPIO号\n"
		"			GPIO_string=[进入IO口][:[退出IO口]]\n"
		"			序号=[-]n[,序号]\n"
		"	-R		退出时复位目标设备.\n"
		"\n"
		"例如:\n"
		"	要得到设备信息:\n"
		"		%s /dev/ttyS0\n"
		"	  这样也行:\n"
		"		%s /dev/i2c-0\n"
		"\n"
		"	写入然后校验最后执行:\n"
		"		%s -w 文件名 -v -g 0x0 /dev/ttyS0\n"
		"\n"
		"	读取flash到文件:\n"
		"		%s -r 文件名 /dev/ttyS0\n"
		"\n"
		"	从flash 的 0x1000 读取 100 个字节到标准输出(屏幕):\n"
		"		%s -r - -S 0x1000:100 /dev/ttyS0\n"
		"\n"
		"	开始执行:\n"
		"		%s -g 0x0 /dev/ttyS0\n"
		"\n"
		"	GPIO 电平:\n"
		"	- 进入BootLoader: GPIO_3=低, GPIO_2=低, GPIO_2=高\n"
		"	- 退出BootLoader: GPIO_3=高, GPIO_2=低, GPIO_2=高\n"
		"		%s -R -i -3,-2,2:3,-2,2 /dev/ttyS0\n",
		name,
		name,
		name,
		name,
		name,
		name,
		name,
		name
	);
}

