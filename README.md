This project is a work-in-progress description of the Gameboy hardware, written
as an emulator in literate C. There is still a lot to do, and I'm working
slowly! The goal is not to produce a particularly fast emulator, but rather to
try and explain how the Gameboy works in a clear and concise manner.

You can read the [current documentation online now][ghpages]

Note: Currently doesn't support sound at all!

[ghpages]: https://tcrs.github.io/gameboy/

# Literate programming

The file `gameboy.lit` is a mix of extended-markdown and C code. The python
script `tools/literate.py` can convert this to either a HTML document for
reading or C source code for compiling. The format itself is quite simple: if a
markdown code blocks starts with a line of the form `<< name >>=` then the code
block is considered to be a chunk of code to be included in the output source
code. The name given to the chunk can be used to reference it, and multiple
chunks with the same name are concatenated together in document order. Within
such a chunk another chunk can be inserted at a particular line using a
reference of the form `<< name >>`. In this way the code can be built up from
many small chunks in any order.

The `literate,py` tool is implemented as a plugin to the python markdown module.
I originally used the excellent [`lit` tool](https://github.com/cdosborn/lit),
but moved to a custom python tool so I could use the extended markdown
formatting available in the python markdown module.

# Examples

There are two example emulators in `src/sboy.h` and `src/xboy.c`. Both use
functions in `src/utils.h` for loading ROM files and loading/saving RAMs (save
data). By default the `minizip` library is used to support loading ROMs directly
from zip files. Defining `WANT_MINIZIP` to 0 when building will disable this.
Both emulators use [SDL][sdl].

`sboy` is a simple emulator, currently set up to support a PS3 controller for
input. It runs at the same framerate as your monitor so YMMV!

`xboy` emulates two gameboys connected via a link cable. Currently they both run
the same ROM and RAM. This is a bit of a prototype but seems to basically work.

[sdl]: https://www.libsdl.org/

# Building

You can build the library and examples using [`ninja`][ninja]. This will The
build produces both source code of the emulation library (`src/gameboy.h`,
`src/gameboy.c`), and the readable document (`doc/gameboy.html`).

I have included pre-generated versions of `gameboy.c` and `gameboy.h`. If you
want to include this library into your own application you can either use the
static library `build/gameboy.a` built by the ninja build, or include the
gameboy source files directly into your application's build.

[ninja]: https://ninja-build.org/
