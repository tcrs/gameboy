/* Copyright (C) 2014-2018 Thomas Spurden <thomas@spurden.name>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/* Enable use of minizip library to load ROMs directly from zip files */
#ifndef WANT_MINIZIP
#define WANT_MINIZIP 1
#endif

#include <string.h>
#include <stdio.h>
#include "gameboy.h"

#if WANT_MINIZIP
#include <minizip/unzip.h>
#endif

static int SaveRAM(struct Gameboy* gb, char const* filename)
{
	FILE* file = fopen(filename, "wb");
	if(!file) {
		return 1;
	}
	if(fwrite(gb->mem.CartRAM, 1, gb->mem.CartRAMSize, file) != gb->mem.CartRAMSize) {
		fclose(file);
		return 1;
	}
	fclose(file);
	return 0;
}

static int LoadRAM(struct Gameboy* gb, char const* filename)
{
	FILE* file = fopen(filename, "rb");
	if(!file) {
		return 1;
	}
	if(fread(gb->mem.CartRAM, 1, gb->mem.CartRAMSize, file) != gb->mem.CartRAMSize) {
		fclose(file);
		return 1;
	}
	fclose(file);
	return 0;
}

static int LoadROM(struct Gameboy* gb, char const* filename)
{
#if WANT_MINIZIP
	unsigned fl = strlen(filename);
	if(fl > 4 && strcmp(&filename[fl - 4], ".zip") == 0) {
		unzFile rom = unzOpen(filename);
		if(rom == NULL) {
			fprintf(stderr, "Could not open %s\n", filename);
			return 1;
		}
		unz_global_info gi;
		if(unzGetGlobalInfo(rom, &gi) != UNZ_OK) {
			fprintf(stderr, "Could get global zip info: %s\n", filename);
			return 1;
		}
		if(gi.number_entry != 1) {
			fprintf(stderr, "Zip file contains multiple files: %s\n", filename);
			return 1;
		}
		if(unzGoToFirstFile(rom) != UNZ_OK) {
			fprintf(stderr, "Could not move to first file in zip: %s\n", filename);
			return 1;
		}
		unz_file_info fileinfo;
		if(unzGetCurrentFileInfo(rom, &fileinfo, NULL, 0, NULL, 0, NULL, 0) != UNZ_OK) {
			fprintf(stderr, "Could not get current file info: %s\n", filename);
			return 1;
		}
		if(fileinfo.uncompressed_size > Cart_MaxROMSize) {
			fprintf(stderr, "Uncompressed file size too large: %s\n", filename);
			return 1;
		}
		gb->mem.CartROMSize = fileinfo.uncompressed_size;
		if(unzOpenCurrentFile(rom) != UNZ_OK) {
			fprintf(stderr, "Could not open current file: %s\n", filename);
			return 1;
		}
		if(unzReadCurrentFile(rom, gb->mem.CartROM, fileinfo.uncompressed_size) != fileinfo.uncompressed_size) {
			fprintf(stderr, "Could not read ROM from file: %s\n", filename);
			return 1;
		}
		if(unzCloseCurrentFile(rom) != UNZ_OK) {
			fprintf(stderr, "Could not close current file: %s\n", filename);
			return 1;
		}
		if(unzClose(rom) != UNZ_OK) {
			fprintf(stderr, "Could not close zip file: %s\n", filename);
			return 1;
		}
	}
	else
#endif
	{
		FILE* rom = fopen(filename, "rb");
		if(rom == NULL) {
			fprintf(stderr, "Could not open %s\n", filename);
			return 1;
		}

		fseek(rom, 0, SEEK_END);
		gb->mem.CartROMSize = ftell(rom);
		if(gb->mem.CartROMSize > Cart_MaxROMSize) {
			fclose(rom);
			fprintf(stderr, "ROM is too large: %s\n", filename);
			return 1;
		}
		fseek(rom, 0, SEEK_SET);
		if(fread(gb->mem.CartROM, 1, gb->mem.CartROMSize, rom) != gb->mem.CartROMSize) {
			fclose(rom);
			fprintf(stderr, "Failed to read ROM from %s\n", filename);
			return 1;
		}
		fclose(rom);
	}
	return 0;
}
