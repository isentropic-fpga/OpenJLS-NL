# Turn a single-lane VHDL block into a 4-lane time-shared one: every listed
# register X becomes a ring X_r(0..3) that rotates each clock; logic reads
# X = X_r(0) (the lane being served this cycle) and writes X_r(3).
import re, sys
src, dst, names = sys.argv[1], sys.argv[2], sys.argv[3].split(',')
L = open(src).read().split('\n')
out = []
decl_re = re.compile(r'^(\s*)signal\s+(\w+)\s*:\s*([^;]+);')
arch_begun = False
in_proc = False
for i, line in enumerate(L):
    m = decl_re.match(line)
    if m and m.group(2) in names:
        ind, n, ty = m.group(1), m.group(2), m.group(3).strip()
        out.append(line)
        m2 = re.match(r'(std_logic_vector|unsigned)\s*(\(.*\))$', ty)
        if ty == 'std_logic':
            out.append(f'{ind}signal {n}_r : std_logic_vector(0 to 3);  -- NL')
        elif m2:
            t = 't_slv_array' if m2.group(1) == 'std_logic_vector' else 't_unsigned_array'
            out.append(f'{ind}signal {n}_r : {t}(0 to 3){m2.group(2)};  -- NL')
        else:
            out.append(f'{ind}type {n}_ring_t is array (0 to 3) of {ty};  -- NL')
            out.append(f'{ind}signal {n}_r : {n}_ring_t;  -- NL')
        continue
    out.append(line)
    if not arch_begun and line == 'begin':
        arch_begun = True
        out.append('')
        out.append('  -- NL: lane served this cycle = ring slot 0')
        for n in names:
            out.append(f'  {n} <= {n}_r(0);')
L = out

def lhs(line):
    m = re.match(r'^(\s*)(\w+)', line)
    if not m or m.group(2) not in names: return None
    ind, n = m.group(1), m.group(2)
    j = m.end(); idx = ''
    if j < len(line) and line[j] == '(':
        d = 0
        for k in range(j, len(line)):
            d += line[k] == '('; d -= line[k] == ')'
            if d == 0: idx = line[j:k+1]; j = k + 1; break
    r = re.match(r'\s*<=\s*(.*)$', line[j:])
    if not r: return None
    return ind, n, idx, r.group(1)

out = []; procs = []
in_proc = False; rst_ind = None; edge_pos = None; assigned = set()
for line in L:
    s = line.strip()
    if re.match(r'^\w+\s*:\s*process\b', s):
        in_proc = True; assigned = set(); edge_pos = None
    if in_proc:
        if s.startswith('if rising_edge(') and edge_pos is None:
            out.append(line); edge_pos = (len(out), len(line) - len(line.lstrip()) + 2); continue
        m = re.match(r'^(\s*)if \(iRst = \'1\'\) then', line)
        if m and rst_ind is None:
            rst_ind = m.group(1); out.append(line); continue
        if rst_ind is not None and re.match(rf'^{rst_ind}(else|elsif)\b', line):
            rst_ind = None; out.append(line); continue
        a = lhs(line)
        if a:
            ind, n, idx, rhs = a; assigned.add(n)
            if rst_ind is not None:
                assert idx == '', line
                rhs = rhs.rstrip(';').strip()
                out.append(f'{ind}{n}_r <= (others => {rhs});'); continue
            out.append(f'{ind}{n}_r(3){idx} <= {rhs}'); continue
        if re.match(r'^end process', s):
            pos, ind = edge_pos
            rot = [' ' * ind + '-- NL: rotate lane rings'] + \
                  [' ' * ind + f'{n}_r <= {n}_r(1 to 3) & {n}_r(0);' for n in names if n in assigned]
            out[pos:pos] = rot
            in_proc = False
    out.append(line)
open(dst, 'w').write('\n'.join(out))
