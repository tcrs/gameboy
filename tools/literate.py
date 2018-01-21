#!/usr/bin/env python3

import re
import sys
import codecs
import argparse
import markdown
from markdown.preprocessors import Preprocessor

try:
    from pygments import highlight
    from pygments.lexers import get_lexer_by_name
    from pygments.formatters import get_formatter_by_name
    have_pygments = True
except ImportError:
    have_pygments = False

class Ref(object):
    __slots__ = ['prefix', 'name']
    def __init__(self, prefix, name):
        self.prefix = prefix
        self.name = name

class LitPreprocessor(Preprocessor):
    CHUNK = re.compile(r'^([\t]+|[ ]{4,})<<\s*([^>]+)\s*>>=\s*$')
    REF = re.compile(r'^(\s*)<<\s*([^>]+)\s*>>\s*$')
    def __init__(self, chunkmap, *args, **kwargs):
        super(LitPreprocessor, self).__init__(*args, **kwargs)
        self.chunks = chunkmap

    def _endChunk(self, name, chunk):
        def safe_name(n, i):
            return '%s%s' % (n.replace(' ', '_'), i)
        prev = self.chunks.setdefault(name, [])
        prev += chunk
        hilines = []
        chunk_idx = self._chunk_index[name]
        chunk_count = self._chunk_counts[name]
        self._chunk_index[name] += 1
        assert chunk_idx < chunk_count, name
        for _, line in chunk:
            if isinstance(line, Ref):
                hilines.append(line.prefix + '/* ' + line.name + ' */')
            else:
                hilines.append(line)
        if have_pygments:
            lexer = get_lexer_by_name('c')
            formatter = get_formatter_by_name('html', nowrap=True)
            rawhl = highlight('\n'.join(hilines), lexer, formatter)
            links = '<a class="code-chunk" id="%s" href="#%s">%s</a>' % (safe_name(name, chunk_idx), safe_name(name, 0), name)
            if chunk_count > 1:
                numlinks = []
                for i in range(chunk_count):
                    if i == chunk_idx:
                        numlinks.append(str(i))
                    else:
                        numlinks.append('<a href="#%s">%u</a>' % (safe_name(name, i), i))
                links += '<span class="chunk-index"> (%s)</span>' % (' '.join(numlinks))

            html = ['<pre class="codehilite"><code>&lt;&lt; %s &gt;&gt=' % links]
            for hl, (_, orig) in zip(rawhl.split('\n'), chunk):
                if isinstance(orig, Ref):
                    html.append('%s&lt;&lt <a href="#%s">%s</a> &gt;&gt;' % (orig.prefix, safe_name(orig.name, 0), orig.name))
                else:
                    html.append(hl)
            html.append('</code></pre>')

            return [self.markdown.htmlStash.store('\n'.join(html), safe=True)]
        else:
            return hilines

    def _countChunks(self, lines):
        self._chunk_counts = {}
        for line in lines:
            chm = self.CHUNK.match(line)
            if chm is not None:
                name = chm.group(2).rstrip()
                if name in self._chunk_counts:
                    self._chunk_counts[name] += 1
                else:
                    self._chunk_counts[name] = 1

    def run(self, lines):
        self._countChunks(lines)
        self._chunk_index = {x:0 for x in self._chunk_counts}
        blanks = []
        new_lines = []
        cur_indent = None
        cur_chunk = None
        cur_name = None

        for lineNo, line in enumerate(lines):
            chm = self.CHUNK.match(line)
            if chm is not None:
                if cur_name is not None:
                    new_lines += self._endChunk(cur_name, cur_chunk)
                if len(blanks) > 0:
                    new_lines += [x[1] for x in blanks]
                    blanks = []
                cur_indent = chm.group(1)
                cur_name = chm.group(2).rstrip()
                cur_chunk = []
            elif cur_name is not None and line.startswith(cur_indent):
                cur_chunk += blanks
                blanks = []

                code = line[len(cur_indent):]
                r = self.REF.match(code)
                if r is None:
                    cur_chunk.append((lineNo, code))
                else:
                    cur_chunk.append((lineNo, Ref(r.group(1), r.group(2).rstrip())))
            elif len(line.lstrip()) == 0:
                blanks.append((lineNo, line))
            else:
                if cur_name is not None:
                    new_lines += self._endChunk(cur_name, cur_chunk)
                    cur_name = None
                if len(blanks) > 0:
                    new_lines += [x[1] for x in blanks]
                    blanks = []
                new_lines.append(line)
        if cur_name is not None:
            new_lines += self._endChunk(cur_name, cur_chunk)
        return new_lines

class LitExtension(markdown.Extension):
    def __init__(self, chunkmap):
        super(LitExtension, self).__init__()
        self.chunkmap = chunkmap

    def extendMarkdown(self, md, md_globals):
        self.pre = LitPreprocessor(self.chunkmap, md)
        md.registerExtension(self)
        md.preprocessors.add('lit', self.pre, '>normalize_whitespace')

def parse_args(args):
    parser = argparse.ArgumentParser(description='''Generate code or html document from literate source files.''')
    parser.add_argument('--css', default=[], action='append', help='Path to CSS to embed in generated HTML')
    parser.add_argument('--title', help='Title to embed in HTML document')
    parser.add_argument('--code', '-c', help='Generate code, starting with the given chunk name')
    parser.add_argument('--out', '-o', help='Where to write result')
    parser.add_argument('inputs', default=[], nargs='+', help='Input files to process, will be read in the provided order')
    return parser.parse_args(args)

def write_doc(args, path, html):
    def write(out):
        out.write('<!DOCTYPE HTML>\n')
        out.write('<html><head>\n')
        if args.title is not None:
            out.write('<title>%s</title>\n' % args.title)
        for css in args.css:
            out.write('<meta charset="UTF-8"><link rel="stylesheet" type="text/css" href="%s">\n' % css)
        out.write('</head><body>\n')
        out.write(html)
        out.write('</body></html>\n')
    if path is None:
        write(sys.stdout)
    else:
        with codecs.open(path, mode='w', encoding='utf-8') as out:
            write(out)

def write_code(args, path, chunks, name):
    def tangle(out, key, prefix):
        for lineNo, line in chunks[key]:
            if isinstance(line, Ref):
                tangle(out, line.name, prefix + line.prefix)
            else:
                out.write(prefix + line + '\n')

    if name not in chunks:
        raise ValueError('No code chunk named "%s" found' % name)

    if path is None:
        tangle(sys.stdout, name, '')
    else:
        with codecs.open(path, mode='w', encoding='utf-8') as out:
            tangle(out, name, '')

def main(cmdline):
    args = parse_args(cmdline)
    chunkmap = {}
    md = markdown.Markdown(extensions=['markdown.extensions.extra', 'markdown.extensions.toc', LitExtension(chunkmap)])

    html = ''
    for inPath in args.inputs:
        with codecs.open(inPath, mode='r', encoding='utf-8') as inFile:
            text = inFile.read()
        html += md.convert(text)
        md.reset()
    if args.code is not None:
        write_code(args, args.out, chunkmap, args.code)
    else:
        write_doc(args, args.out, html)

if __name__ == '__main__':
    main(sys.argv[1:])
