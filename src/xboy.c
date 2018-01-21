/*
 * sboy: simple multi-gameboy emulator
 * Copyright (C) 2014-2018 Thomas Spurden <thomas@spurden.name>
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

#include <SDL2/SDL.h>

#include "gameboy.h"
#include "utils.h"

struct GBWindow {
	struct Gameboy gb;
	SDL_Window* window;
	SDL_Renderer* renderer;
	SDL_Texture* frameTexture;
};

int PollEvents(struct GBWindow* gbs, unsigned int numGBs)
{
	static unsigned int activeWindow = 0;
	static bool windowActive = false;

	SDL_Event event;
	while(SDL_PollEvent(&event)) {
		int key = 0;
		bool up = false;
		switch(event.type) {
			case SDL_WINDOWEVENT:
				switch(event.window.event) {
					case SDL_WINDOWEVENT_FOCUS_LOST:
						windowActive = false;
						break;
					case SDL_WINDOWEVENT_FOCUS_GAINED:
						windowActive = true;
						activeWindow = event.window.windowID;
						break;
					default:
						break;
				}
				break;
			case SDL_KEYUP:
				up = true;
				// fallthrough
			case SDL_KEYDOWN:
				switch(event.key.keysym.sym) {
					case SDLK_UP:
						key = Button_Up;
						break;
					case SDLK_DOWN:
						key = Button_Down;
						break;
					case SDLK_LEFT:
						key = Button_Left;
						break;
					case SDLK_RIGHT:
						key = Button_Right;
						break;
					case SDLK_RETURN:
						key = Button_Start;
						break;
					case SDLK_TAB:
						key = Button_Select;
						break;
					case SDLK_LALT:
						key = Button_A;
						break;
					case SDLK_LCTRL:
						key = Button_B;
						break;
					case SDLK_ESCAPE:
						return 1;
						break;
					default:
						break;
				}
				break;
			case SDL_QUIT:
				return 1;
				break;
			default:
				break;
		}

		if(key && windowActive) {
			for(unsigned int i = 0; i < numGBs; i += 1) {
				if(SDL_GetWindowID(gbs[i].window) == activeWindow) {
					gameboy_setButtonState(&gbs[i].gb, key, !up);
				}
			}
		}
	}

	return 0;
}

int gbwindow_init(struct GBWindow* gbw, char const* romFilename, char const* saveFilename)
{
	LoadROM(&gbw->gb, romFilename);
	char const* loadError = gameboy_load(&gbw->gb);

	if(loadError) {
		fprintf(stderr, "Could not load ROM: %s\n", loadError);
		return 1;
	}

	if(saveFilename && gbw->gb.info.HasBattery && gbw->gb.mem.CartRAMSize) {
		if(LoadRAM(&gbw->gb, saveFilename)) {
			fprintf(stderr, "Could not load RAM from \"%s\"\n", saveFilename);
		}
	}

	gbw->window = SDL_CreateWindow("SBoy", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600, SDL_WINDOW_RESIZABLE);
	if(!gbw->window) {
		fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
		return 1;
	}

	gbw->renderer = SDL_CreateRenderer(gbw->window, -1, SDL_RENDERER_PRESENTVSYNC);
	if(!gbw->renderer) {
		fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
		return 1;
	}

	gbw->frameTexture = SDL_CreateTexture(gbw->renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, 160, 144);
	if(!gbw->frameTexture) {
		fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
		return EXIT_FAILURE;
	}

	SDL_SetRenderDrawColor(gbw->renderer, 255, 255, 255, 255);
	SDL_RenderClear(gbw->renderer);
	SDL_RenderPresent(gbw->renderer);
	SDL_RenderSetLogicalSize(gbw->renderer, 160, 144);

	gameboy_reset(&gbw->gb, true);

	return 0;
}

uint32_t palette[4] = { 0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000 };
uint32_t frame[144][160];

int gbwindow_step(struct GBWindow* gbw)
{
	if(gameboy_step(&gbw->gb)) {
		return 1;
	}

	if(gbw->gb.lcd.NewFrame) {
		gbw->gb.lcd.NewFrame = false;
		for(unsigned int y = 0; y < 144; y += 1) {
			for(unsigned int x = 0; x < 160; x += 1) {
				frame[y][x] = palette[gbw->gb.lcd.Buffer[x][y] & 0x03];
			}
		}
		SDL_UpdateTexture(gbw->frameTexture, NULL, frame, 160 * sizeof(uint32_t));
		SDL_RenderClear(gbw->renderer);
		SDL_RenderCopy(gbw->renderer, gbw->frameTexture, NULL, NULL);
		SDL_RenderPresent(gbw->renderer);
	}

	return 0;
}

int main(int argc, char** argv)
{
	if(argc < 2) {
		fprintf(stderr, "Usage: sboy <romfile> [<savefile>]\n");
		return EXIT_FAILURE;
	}

	char const* saveFilename = NULL;
	if(argc > 2) {
		saveFilename = argv[2];
	}

	if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER)) {
		fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
		return EXIT_FAILURE;
	}

	atexit(SDL_Quit);

	static struct GBWindow gb[2];
	for(unsigned int i = 0; i < 2; i += 1) {
		if(gbwindow_init(&gb[i], argv[1], saveFilename)) {
			exit(EXIT_FAILURE);
		}
	}

	SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "best");

	bool transferActive = false;
	uint64_t lastBitCycle = 0;

	uint64_t lastKeyCheck = 0;
	while(1) {
		for(unsigned int i = 0; i < 2; i += 1) {
			if(gbwindow_step(&gb[i])) {
				exit(EXIT_FAILURE);
			}
		}

		uint8_t sc[2] = { gb[0].gb.mem.IO[0x02], gb[1].gb.mem.IO[0x02] };
		if(transferActive) {
			uint64_t delta;
			if(sc[0] & 1) {
				delta = gb[0].gb.TotalCycles - lastBitCycle;
			}
			else {
				delta = gb[1].gb.TotalCycles - lastBitCycle;
			}
			uint64_t bitsTransferred = delta / 512;
			if(bitsTransferred > 7) {
				transferActive = false;
				gb[0].gb.mem.IO[0x02] &= ~0x80;
				gb[1].gb.mem.IO[0x02] &= ~0x80;
				gb[0].gb.mem.IO[0x0F] |= Interrupt_Serial;
				gb[1].gb.mem.IO[0x0F] |= Interrupt_Serial;
				transferActive = false;
			}
		}
		else {
			if((sc[0] & 0x01) != (sc[1] & 0x01) && (sc[0] & 0x80) && (sc[1] & 0x80)) {
				transferActive = true;

				uint8_t sb[2] = { gb[0].gb.mem.IO[0x01], gb[1].gb.mem.IO[0x01] };

				printf("Transfer 0x%02X <-> 0x%02X\n", sb[0], sb[1]);
				gb[0].gb.mem.IO[0x01] = sb[1];
				gb[1].gb.mem.IO[0x01] = sb[0];

				if(sc[0] & 0x01) {
					lastBitCycle = gb[0].gb.TotalCycles;
				}
				else {
					lastBitCycle = gb[1].gb.TotalCycles;
				}
			}
		}

		if((gb[0].gb.TotalCycles - lastKeyCheck) > 4096) {
			lastKeyCheck = gb[0].gb.TotalCycles;
			if(PollEvents(gb, 2)) {
				if(saveFilename && gb[0].gb.info.HasBattery && gb[0].gb.mem.CartRAMSize) {
					if(SaveRAM(&gb[0].gb, saveFilename)) {
						fprintf(stderr, "Could not save RAM to \"%s\"\n", saveFilename);
						exit(EXIT_FAILURE);
					}
				}
				return (EXIT_SUCCESS);
			}
		}
	}

	return EXIT_FAILURE;
}
