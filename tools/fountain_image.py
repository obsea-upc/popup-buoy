# /// script
# dependencies = ["pillow>=11.2", "imagecodecs", "numpy"]
# ///
"""Image link of the pop-up buoy, from 1 Oct 2026: thumbnail + systematic pass + fountain.

What the lander writes (encode) and what the ground reads back (decode). The buoy does not
change: it keeps sending the data file line by line.

Two objects travel, each cut into symbols of S bytes:
  obj 0  THUMBNAIL  - a small AVIF of the whole scene (~5 % of the photo). Only the AV1
                      payload flies; the container around it stays on the ground, in the
                      manifest, the same trick as the JPEG header today.
  obj 1  IMAGE      - today's picture: the photo at 10 %, grey, in 22 vertical JPEG strips
                      (q75), headers stripped and the strips packed back to back. Kept as
                      strips so a partial image shows piece by piece.

File order: thumbnail sources, a few thumbnail mixtures, the image sources once in order
(the systematic pass: the first hours look exactly like today), then fountain mixtures
of the image with a thumbnail mixture every THUMB_EVERY rows. A mixture is the XOR of d
source symbols picked by a PRNG seeded with (object, index), so the ground rebuilds
which ones from the header alone. Any clean mixture helps; the ground solves the lot by
Gaussian elimination, so the "last missing frame" wait of repetition is gone.

Frame (one line of the data file, hex):
  2 B header  : object (2 bits) << 14 | symbol index (14 bits; < K = source, >= K = mixture)
  S B symbol
  1 B CRC-8   : over header + symbol. BCH alone lets 4.8 % of frames through wrong, and a
                single wrong mixture poisons everything solved with it.
  2 B BCH     : shortened BCH, t = 2, over header + symbol + CRC. Repairs 1-2 flipped bits.
  KIM1     23 B -> S = 18      (46 hex per line)
  Arribada 24 B -> S = 19      (48 hex per line; its LDA2 frame is one byte longer, no padding)

  uv run tools/fountain_image.py encode --photo <jpg> --out <dir> [--rows 4000]
  uv run tools/fountain_image.py decode --manifest <dir>/manifest_kim.json --cls "<glob>" --ref 216573 --out <dir>
  uv run tools/fountain_image.py simulate --manifest <dir>/manifest_kim.json --p 0.30 --txh 27
"""
import argparse, glob, io, json, math, os, random, sys
import numpy as np
from PIL import Image

LAYOUT = {"kim": {"frame": 23, "sym": 18, "hexoff": 8}, "arribada": {"frame": 24, "sym": 19, "hexoff": 0}}
OBJ_THUMB, OBJ_IMAGE = 0, 1
THUMB_EVERY = 20          # one thumbnail mixture every this many mixture rows, later on
THUMB_REPAIR_X = 3        # thumbnail mixtures right after its sources: this many times k
DENSE_K = 32              # objects this small use dense mixtures (see neighbours)

def thumb_repair(kt): return THUMB_REPAIR_X * kt

# Layout knobs, chosen with `simulate` (see the campaign notes): how many systematic
# passes of the image go out before the mixtures, and how often a thumbnail mixture is
# slipped in early on, so a weak buoy still gets its thumbnail in the first hours.
# Chosen 28 Sep 2026 at 20 deg, 27 rows/h: one pass, and a thumbnail mixture every 5
# rows over the first 600 - thumbnail in ~1.4 h at 30 % reception and ~6 h at 15 %
# (22 h without it), for ~3-4 h more to the full image.
PASSES = 1
EARLY_ROWS, EARLY_THUMB_EVERY = 600, 5

def file_order(kt, km, rows):
    """(object, symbol index) of every row of the data file, in order."""
    order = [(OBJ_THUMB, i) for i in range(kt)] + [(OBJ_THUMB, kt + i) for i in range(thumb_repair(kt))]
    t_next = kt + thumb_repair(kt)
    image = [i for _ in range(PASSES) for i in range(km)]           # the systematic passes
    m_next, n = km, 0
    while len(order) < rows:
        n += 1
        every = EARLY_THUMB_EVERY if len(order) < EARLY_ROWS else THUMB_EVERY
        if n % every == 0: order.append((OBJ_THUMB, t_next)); t_next += 1
        elif image: order.append((OBJ_IMAGE, image.pop(0)))
        else: order.append((OBJ_IMAGE, m_next)); m_next += 1
    return order

# ---------------------------------------------------------------- GF(2^8), BCH t=2
PRIM = 0x11D
EXP = [0] * 512; LOG = [0] * 256
x = 1
for i in range(255):
    EXP[i] = x; LOG[x] = i; x <<= 1
    if x & 0x100: x ^= PRIM
for i in range(255, 512): EXP[i] = EXP[i - 255]
def gmul(a, b): return 0 if a == 0 or b == 0 else EXP[LOG[a] + LOG[b]]

def minimal_poly(power):
    """Binary minimal polynomial of alpha^power, as an int (bit i = coeff of x^i)."""
    conj, e = [], power % 255
    while e not in conj: conj.append(e); e = (e * 2) % 255
    poly = [1]                                    # coefficients in GF(2^8), low degree first
    for c in conj:
        r = EXP[c]; new = [0] * (len(poly) + 1)
        for i, a in enumerate(poly):
            new[i + 1] ^= a; new[i] ^= gmul(a, r)
        poly = new
    return sum((1 << i) for i, a in enumerate(poly) if a)

def pmul2(a, b):
    r = 0
    while b:
        if b & 1: r ^= a
        a <<= 1; b >>= 1
    return r

GEN = pmul2(minimal_poly(1), minimal_poly(3))     # degree 16: BCH(255,239) t=2
NPAR = GEN.bit_length() - 1

def pmod(v, nbits):
    """v (nbits long, MSB first as highest degree) mod GEN."""
    for i in range(nbits - 1, NPAR - 1, -1):
        if v >> i & 1: v ^= GEN << (i - NPAR)
    return v

_SYN = {}
def syndrome_table(n):
    if n not in _SYN:
        t = {}
        single = [pmod(1 << i, n) if i >= NPAR else (1 << i) for i in range(n)]
        for i in range(n):
            t[single[i]] = (i,)
            for j in range(i):
                t.setdefault(single[i] ^ single[j], (i, j))
        _SYN[n] = t
    return _SYN[n]

def bch_encode(data: bytes) -> bytes:
    k = len(data) * 8; v = int.from_bytes(data, "big") << NPAR
    return (v | pmod(v, k + NPAR)).to_bytes(len(data) + NPAR // 8, "big")

def bch_decode(frame: bytes):
    """-> (data, bits fixed) or (None, -1)."""
    n = len(frame) * 8; v = int.from_bytes(frame, "big"); s = pmod(v, n)
    if s == 0: return frame[:-NPAR // 8], 0
    pos = syndrome_table(n).get(s)
    if pos is None: return None, -1
    for p in pos: v ^= 1 << p
    return v.to_bytes(len(frame), "big")[:-NPAR // 8], len(pos)

def crc8(b: bytes) -> int:
    c = 0
    for x in b:
        c ^= x
        for _ in range(8): c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c

# ---------------------------------------------------------------- fountain mixtures
def soliton_cdf(k, c=0.1, delta=0.5):
    R = c * math.log(k / delta) * math.sqrt(k)
    rho = [0, 1 / k] + [1 / (d * (d - 1)) for d in range(2, k + 1)]
    tau = [0.0] * (k + 1); pivot = max(1, int(round(k / R)))
    for d in range(1, k + 1):
        if d < pivot: tau[d] = R / (d * k)
        elif d == pivot: tau[d] = R * math.log(R / delta) / k
    mu = [rho[d] + tau[d] for d in range(k + 1)]; s = sum(mu)
    acc, cdf = 0.0, []
    for m in mu: acc += m / s; cdf.append(acc)
    return cdf

def neighbours(obj, esi, k):
    """Source symbols mixed into symbol esi of an object with k sources."""
    if esi < k: return [esi]
    rng = random.Random(obj * 1_000_003 + esi * 7919 + k)
    if k <= DENSE_K:
        # Small object (the thumbnail): every source in with probability 1/2. Solved by
        # Gaussian elimination, it needs about k + 2 clean mixtures, where LT degrees
        # at k ~ 11 need far more. Costs nothing: the ground solves 11 unknowns.
        s = [i for i in range(k) if rng.random() < 0.5]
        return s if s else [rng.randrange(k)]
    u = rng.random(); cdf = soliton_cdf(k)
    d = next(i for i, c in enumerate(cdf) if c >= u); d = max(1, min(d, k))
    return sorted(rng.sample(range(k), d))

def xor_all(syms):
    out = bytearray(len(syms[0]))
    for s in syms:
        for i, b in enumerate(s): out[i] ^= b
    return bytes(out)

# ---------------------------------------------------------------- the two objects
def jpeg_split(d: bytes):
    """-> (header up to the end of SOS, entropy-coded scan). EOI dropped."""
    sos = d.find(b"\xff\xda"); ln = int.from_bytes(d[sos + 2:sos + 4], "big")
    cut = sos + 2 + ln
    body = d[cut:]
    if body.endswith(b"\xff\xd9"): body = body[:-2]
    return d[:cut], body

def build_image(photo, scale=0.10, strips=22, q=75):
    img = photo.resize((round(photo.width * scale), round(photo.height * scale)), Image.LANCZOS)
    w, h = img.size; sw = w // strips
    parts, meta = [], []
    for i in range(strips):
        l = i * sw; r = (i + 1) * sw if i < strips - 1 else w
        b = io.BytesIO(); img.crop((l, 0, r, h)).save(b, "JPEG", quality=q); hdr, scan = jpeg_split(b.getvalue())
        meta.append({"x": l, "w": r - l, "offset": sum(len(p) for p in parts), "len": len(scan), "header": hdr.hex()})
        parts.append(scan)
    return b"".join(parts), {"size": [w, h], "strips": meta}

def avif_payload(d: bytes):
    """-> (prefix, AV1 payload in mdat, suffix) of an AVIF file."""
    i = 0
    while i < len(d):
        size = int.from_bytes(d[i:i + 4], "big"); typ = d[i + 4:i + 8]
        if typ == b"mdat": return d[:i + 8], d[i + 8:i + size], d[i + size:]
        i += size
    raise ValueError("no mdat box")

def build_thumb(photo, max_bytes, scale=0.05):
    import imagecodecs as ic
    img = photo.resize((round(photo.width * scale), round(photo.height * scale)), Image.LANCZOS)
    a = np.asarray(img); rgb = np.dstack([a, a, a])
    best = None
    for level in range(90, 0, -2):                # best quality that fits the budget
        d = ic.avif_encode(rgb, level=level, speed=0)
        pre, pay, suf = avif_payload(d)
        if len(pay) <= max_bytes: best = (level, pre, pay, suf); break
    if best is None: raise SystemExit("thumbnail does not fit the budget")
    level, pre, pay, suf = best
    return pay, {"size": list(img.size), "level": level, "prefix": pre.hex(), "suffix": suf.hex()}

# ---------------------------------------------------------------- encode
def symbols_of(data: bytes, S):
    k = math.ceil(len(data) / S); pad = data + bytes(k * S - len(data))
    return [pad[i * S:(i + 1) * S] for i in range(k)]

def frame_hex(obj, esi, sym):
    hdr = ((obj << 14) | esi).to_bytes(2, "big")
    body = hdr + sym; body += bytes([crc8(body)])
    return bch_encode(body).hex()

def encode(args):
    photo = Image.open(args.photo).convert("L")
    os.makedirs(args.out, exist_ok=True)
    img_bytes, img_meta = build_image(photo)
    thumb_budget = args.thumb_frames * 18        # the same thumbnail on both files
    th_bytes, th_meta = build_thumb(photo, thumb_budget)
    for mod, L in LAYOUT.items():
        S = L["sym"]
        th_syms = symbols_of(th_bytes, S); im_syms = symbols_of(img_bytes, S)
        kt, km = len(th_syms), len(im_syms)
        rows = file_order(kt, km, args.rows)
        if max(e for _, e in rows) >= 1 << 14: raise SystemExit("symbol index over 14 bits")
        lines = []
        for r, (obj, esi) in enumerate(rows, start=1):
            src = th_syms if obj == OBJ_THUMB else im_syms
            sym = xor_all([src[j] for j in neighbours(obj, esi, len(src))])
            h = frame_hex(obj, esi, sym)
            assert len(h) == 2 * L["frame"]
            lines.append(f"{r}:{h}")
        with open(os.path.join(args.out, f"dataFile_{mod}.txt"), "w", newline="\n") as f:
            f.write("\n".join(lines) + "\n")
        man = {"format": "fountain-v1", "module": mod, "frame_bytes": L["frame"], "symbol_bytes": S,
               "rows": len(rows), "thumb_every": THUMB_EVERY, "thumb_repair": thumb_repair(kt), "dense_k": DENSE_K,
               "passes": PASSES, "early_rows": EARLY_ROWS, "early_thumb_every": EARLY_THUMB_EVERY,
               "objects": {"thumb": {"obj": OBJ_THUMB, "k": kt, "bytes": len(th_bytes), **th_meta},
                           "image": {"obj": OBJ_IMAGE, "k": km, "bytes": len(img_bytes), **img_meta}},
               "photo": os.path.basename(args.photo)}
        with open(os.path.join(args.out, f"manifest_{mod}.json"), "w") as f: json.dump(man, f, indent=1)
        print(f"{mod}: S={S} B, thumbnail {len(th_bytes)} B = {kt} symbols (AVIF level {th_meta['level']}, "
              f"{th_meta['size'][0]}x{th_meta['size'][1]}), image {len(img_bytes)} B = {km} symbols, {len(rows)} rows")
    # ground-truth renders, for the web and the bench
    render(bytes(th_bytes), None, man["objects"]["thumb"], "thumb").save(os.path.join(args.out, "thumb_sent.png"))
    render(img_bytes, None, man["objects"]["image"], "image").save(os.path.join(args.out, "image_sent.png"))

# ---------------------------------------------------------------- ground: solve and render
class Solver:
    """Incremental Gauss-Jordan over GF(2): rows kept reduced, one per pivot."""
    def __init__(self, k, S): self.k, self.S, self.rows = k, S, {}      # pivot -> [mask, data int]
    def add(self, cols, sym: bytes):
        m = 0
        for c in cols: m ^= 1 << c
        d = int.from_bytes(sym, "big")
        for p, (pm, pd) in self.rows.items():
            if m >> p & 1: m ^= pm; d ^= pd
        if m == 0: return False
        p = (m & -m).bit_length() - 1
        for q, row in self.rows.items():
            if row[0] >> p & 1: row[0] ^= m; row[1] ^= d
        self.rows[p] = [m, d]
        return True
    def known(self):
        return {p: r[1].to_bytes(self.S, "big") for p, r in self.rows.items() if r[0] == 1 << p}

def render(data, known_mask, meta, kind):
    """Picture from the object bytes; bytes not known are None in known_mask."""
    if kind == "thumb":
        w, h = meta["size"]
        if known_mask is not None and not all(known_mask): return Image.new("L", (w, h), 0)
        try:
            import imagecodecs as ic
            a = ic.avif_decode(bytes.fromhex(meta["prefix"]) + data + bytes.fromhex(meta["suffix"]))
            return Image.fromarray(a).convert("L")
        except Exception:
            return Image.new("L", (w, h), 0)
    w, h = meta["size"]; canvas = Image.new("L", (w, h), 0)
    for s in meta["strips"]:
        a, b = s["offset"], s["offset"] + s["len"]
        if known_mask is not None and not all(known_mask[a:b]): continue
        try:
            strip = Image.open(io.BytesIO(bytes.fromhex(s["header"]) + data[a:b] + b"\xff\xd9")); strip.load()
            canvas.paste(strip, (s["x"], 0))
        except Exception:
            pass
    return canvas

def decode(args):
    man = json.load(open(args.manifest)); L = LAYOUT[man["module"]]; S = man["symbol_bytes"]
    objs = {o["obj"]: (name, o) for name, o in man["objects"].items()}
    solvers = {o["obj"]: Solver(o["k"], S) for o in man["objects"].values()}
    msgs = []
    for f in sorted(glob.glob(args.cls)):
        for m in json.load(open(f, encoding="utf-8"))["contents"]:
            if m.get("deviceRef") == args.ref: msgs.append(m)
    seen, frames = set(), []
    for m in sorted(msgs, key=lambda m: m["msgDatetime"]):
        if m["deviceMsgUid"] in seen: continue
        seen.add(m["deviceMsgUid"])
        raw = (m.get("rawData") or "").lower()[L["hexoff"]:]
        if len(raw) < 2 * L["frame"]: continue
        frames.append((m["msgDatetime"], bytes.fromhex(raw[:2 * L["frame"]])))
    stats = {"received": len(frames), "clean": 0, "fixed": 0, "bch_fail": 0, "crc_fail": 0, "timeline": []}
    for t, fr in frames:
        body, nfix = bch_decode(fr)
        if body is None: stats["bch_fail"] += 1; continue
        if crc8(body[:-1]) != body[-1]: stats["crc_fail"] += 1; continue
        stats["clean" if nfix == 0 else "fixed"] += 1
        hdr = int.from_bytes(body[:2], "big"); obj, esi = hdr >> 14, hdr & 0x3FFF
        if obj not in solvers: continue
        k = objs[obj][1]["k"]
        solvers[obj].add(neighbours(obj, esi, k), body[2:2 + S])
        stats["timeline"].append([t, obj, esi, len(solvers[OBJ_THUMB].known()), len(solvers[OBJ_IMAGE].known())])
    os.makedirs(args.out, exist_ok=True)
    for obj, (name, o) in objs.items():
        kn = solvers[obj].known(); k = o["k"]
        data = b"".join(kn.get(i, bytes(S)) for i in range(k))[:o["bytes"]]
        mask = []
        for i in range(k): mask += [i in kn] * S
        mask = mask[:o["bytes"]]
        render(data, mask, o, "thumb" if name == "thumb" else "image").save(os.path.join(args.out, f"{args.name}_{name}.png"))
        stats[name] = {"k": k, "known": len(kn)}
        if name == "image":
            stats[name]["strips"] = sum(all(mask[s["offset"]:s["offset"] + s["len"]]) for s in o["strips"])
    json.dump(stats, open(os.path.join(args.out, f"{args.name}_stats.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in stats.items() if k != "timeline"}))

# ---------------------------------------------------------------- expected performance
def simulate(args):
    """Hours to thumbnail, to the systematic pass, to the full image, if each row arrives
    usable with probability p at txh rows per hour. Uses the real file order and code."""
    man = json.load(open(args.manifest)); S = man["symbol_bytes"]
    kt, km = man["objects"]["thumb"]["k"], man["objects"]["image"]["k"]
    order = file_order(kt, km, man["rows"])
    rng = random.Random(1); res = {"thumb": [], "half": [], "full": [], "repeat_half": [], "repeat_full": []}
    R = man["rows"]
    curve = np.zeros(R + 1); tcurve = np.zeros(R + 1); rcurve = np.zeros(R + 1)
    for run in range(args.runs):
        st = {0: Solver(kt, 1), 1: Solver(km, 1)}; got = {"thumb": None, "half": None, "full": None}
        nt = nm = 0
        for r, (obj, esi) in enumerate(order, start=1):
            if rng.random() < args.p and st[obj].add(neighbours(obj, esi, kt if obj == 0 else km), b"\x00"):
                if obj == 0: nt = len(st[0].known())
                else: nm = len(st[1].known())
            curve[r] += nm / km; tcurve[r] += (nt == kt)
            if got["thumb"] is None and nt == kt: got["thumb"] = r
            if got["half"] is None and nm >= km / 2: got["half"] = r
            if nm == km: got["full"] = r; curve[r + 1:] += 1; tcurve[r + 1:] += 1; break
        for key in got: res[key].append(got[key] or R)
        # Today's method at the same p: the image frames repeated in order, no thumbnail.
        # Today's frame carries 19 B of image, so the same picture is ceil(bytes/19) frames.
        kr = math.ceil(man["objects"]["image"]["bytes"] / 19)
        seen, h50 = set(), None
        for r in range(1, R + 1):
            if rng.random() < args.p: seen.add((r - 1) % kr)
            rcurve[r] += len(seen) / kr
            if h50 is None and len(seen) >= kr / 2: h50 = r
            if len(seen) == kr: rcurve[r + 1:] += 1; break
        r_full = r if len(seen) == kr else None
        while r_full is None and len(seen) < kr:       # past the file, only for the number
            r += 1
            if rng.random() < args.p: seen.add((r - 1) % kr)
            if len(seen) == kr: r_full = r
        res["repeat_half"].append(h50 or R); res["repeat_full"].append(r_full)
    out = {k: round(float(np.median(v)) / args.txh, 1) for k, v in res.items()}
    at = lambda c, h: round(float(c[min(R, int(h * args.txh))]) / args.runs * 100)
    out["hours"] = list(range(0, 61))
    out["image_pct"] = [at(curve, h) for h in out["hours"]]
    out["thumb_pct_runs"] = [at(tcurve, h) for h in out["hours"]]      # % of runs with the thumbnail
    out["repeat_pct"] = [at(rcurve, h) for h in out["hours"]]
    out.update({"p": args.p, "txh": args.txh, "kt": kt, "km": km})
    print(json.dumps(out))
    if args.json: json.dump(out, open(args.json, "w"), indent=1)

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("encode"); e.add_argument("--photo", required=True); e.add_argument("--out", required=True)
    e.add_argument("--rows", type=int, default=4000); e.add_argument("--thumb-frames", type=int, default=11)
    d = sub.add_parser("decode"); d.add_argument("--manifest", required=True); d.add_argument("--cls", required=True)
    d.add_argument("--ref", required=True); d.add_argument("--out", required=True); d.add_argument("--name", default="buoy")
    s = sub.add_parser("simulate"); s.add_argument("--manifest", required=True); s.add_argument("--p", type=float, required=True)
    s.add_argument("--txh", type=float, required=True); s.add_argument("--runs", type=int, default=40); s.add_argument("--json")
    for p in (e, s):
        p.add_argument("--passes", type=int); p.add_argument("--early-rows", type=int); p.add_argument("--early-every", type=int)
    a = ap.parse_args()
    if getattr(a, "passes", None): PASSES = a.passes
    if getattr(a, "early_rows", None) is not None: EARLY_ROWS = a.early_rows
    if getattr(a, "early_every", None): EARLY_THUMB_EVERY = a.early_every
    {"encode": encode, "decode": decode, "simulate": simulate}[a.cmd](a)
