#!/usr/bin/env python3
"""Barr-C passes clang-format cannot do.  Run by tools/barr_format.sh,
   after clang-format.
   8.2d  every if / else-if chain ends with an else
   8.3a  each break is aligned with its case
   3.3c  every file ends with the end-of-file comment and a newline
"""
import re, sys
EOF_MARK = '/*** end of file ***/'

def mask(src):
    """Blank out comments, strings and char literals (same length)."""
    out = list(src)
    for m in re.finditer(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', src, re.S):
        for k in range(m.start(), m.end()):
            if out[k] != '\n':
                out[k] = ' '
    return ''.join(out)

def match_brace(msk, open_pos):
    depth = 0
    for k in range(open_pos, len(msk)):
        if msk[k] == '{':
            depth += 1
        elif msk[k] == '}':
            depth -= 1
            if depth == 0:
                return k
    return -1

def final_else(src):
    count = 0
    while True:
        msk = mask(src)
        done = True
        for m in re.finditer(r'^([ \t]*)else if\b', msk, re.M):
            indent = m.group(1)
            # skip the condition
            p = msk.index('(', m.end())
            depth = 0
            while True:
                if msk[p] == '(':
                    depth += 1
                elif msk[p] == ')':
                    depth -= 1
                    if depth == 0:
                        break
                p += 1
            ob = msk.index('{', p)
            if msk[p + 1:ob].strip():
                continue
            cb = match_brace(msk, ob)
            rest = msk[cb + 1:]
            if re.match(r'\s*else\b', rest):
                continue
            eol = src.index('\n', cb)
            bslash = src[cb + 1:eol].strip() == '\\'
            if bslash:
                continue            # inside a macro: handled by hand
            add = ('\n%selse\n%s{\n%s    /* No action required. */\n%s}'
                   % (indent, indent, indent, indent))
            src = src[:cb + 1] + add + src[cb + 1:]
            count += 1
            done = False
            break
        if done:
            return src, count

def align_breaks(src):
    """Dedent a case's terminating break to the column of its case label."""
    lines = src.split('\n')
    mlines = mask(src).split('\n')
    count = 0
    stack = []          # [kind, indent of the current case label]
    prev = ''           # previous non-blank masked line
    for n, ml in enumerate(mlines):
        st = ml.strip()
        m = re.match(r'([ \t]*)(case\b.*:|default\s*:)$', ml.rstrip())
        if m and stack and stack[-1][0] == 'switch':
            stack[-1][1] = m.group(1)
        if st == 'break;' and stack:
            top = stack[-1]
            if top[0] == 'switch' and top[1] is not None:
                want = top[1] + 'break;'
                if lines[n].rstrip() != want:
                    lines[n] = want
                    count += 1
            elif top[0] == 'caseblock':
                k = n + 1
                while not mlines[k].strip():
                    k += 1
                if mlines[k].strip() == '}':
                    # move the break to after the block's closing brace
                    lines[n] = None
                    lines[k] = lines[k] + '\n' + stack[-2][1] + 'break;'
                    count += 1
                else:
                    BRACED.append(n + 1)
        for ch in ml:
            if ch == '{':
                if st == '{' and re.match(r'switch\b', prev):
                    stack.append(['switch', None])
                elif (st == '{' and stack and stack[-1][0] == 'switch'
                      and re.match(r'(case\b.*:|default\s*:)$', prev)):
                    stack.append(['caseblock', None])
                else:
                    stack.append(['other', None])
            elif ch == '}':
                if stack:
                    stack.pop()
        if st:
            prev = st
    return '\n'.join(x for x in lines if x is not None), count

BRACED = []

def eof_comment(src):
    body = src.rstrip()
    if body.endswith(EOF_MARK):
        return body + '\n', 0
    return body + '\n\n' + EOF_MARK + '\n', 1

if __name__ == '__main__':
    tot = [0, 0, 0]
    for f in sys.argv[1:]:
        s = open(f).read()
        s, a = final_else(s)
        s, b = align_breaks(s)
        s, c = eof_comment(s)
        tot[0] += a; tot[1] += b; tot[2] += c
        open(f, 'w').write(s)
    print('final else %d, break %d, eof %d, braced-case breaks %d' % (tot[0], tot[1], tot[2], len(BRACED)))
