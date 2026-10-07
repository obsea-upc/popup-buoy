# /// script
# dependencies = ["pillow>=11.2", "imagecodecs", "numpy", "scikit-image"]
# ///
"""Study for the next image file (7 Oct 2026): which codec, and which fountain mixtures.

  uv run tools/image_study.py codecs  --photo <jpg>      bytes each codec needs for the quality of today's 22 JPEG strips
  uv run tools/image_study.py fountain --k 107 [--k 60]  clean frames the decoder needs: LT (today) vs dense GF(2)

Quality is SSIM (and PSNR) against the photo resized to the transmitted size, grey, as on the buoy.
"""
import argparse, io, importlib.util, json, math, os, random, sys
import numpy as np
from PIL import Image
from skimage.metrics import structural_similarity as ssim, peak_signal_noise_ratio as psnr

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("fi", os.path.join(HERE, "fountain_image.py"))
fi = importlib.util.module_from_spec(spec); spec.loader.exec_module(fi)

def ref_image(photo, scale):
    p = Image.open(photo).convert("L")
    return p.resize((round(p.width * scale), round(p.height * scale)), Image.LANCZOS)

def strips_decode(img, q=75, strips=22):
    """Today's image: 22 vertical JPEG strips, headers on the ground. -> (bytes sent, decoded array)."""
    data, meta = fi.build_image(img, scale=1.0, strips=strips, q=q)
    out = Image.new("L", img.size)
    for s in meta["strips"]:
        a, b = s["offset"], s["offset"] + s["len"]
        st = Image.open(io.BytesIO(bytes.fromhex(s["header"]) + data[a:b] + b"\xff\xd9")); st.load()
        out.paste(st, (s["x"], 0))
    return len(data), np.asarray(out)

def enc(codec, a, q):
    import imagecodecs as ic
    if codec == "jpeg":
        b = io.BytesIO(); Image.fromarray(a).save(b, "JPEG", quality=int(q), optimize=True)
        hdr, scan = fi.jpeg_split(b.getvalue()); d = b.getvalue()
        return len(scan), np.asarray(Image.open(io.BytesIO(d)).convert("L"))   # only the scan flies
    if codec == "webp":
        d = ic.webp_encode(a, level=int(q)); return len(d), ic.webp_decode(d)
    if codec == "avif":
        rgb = np.dstack([a, a, a])
        d = ic.avif_encode(rgb, level=int(q), speed=0)
        pre, pay, suf = fi.avif_payload(d)                                     # container stays on the ground
        dec = ic.avif_decode(d); dec = dec if dec.ndim == 2 else np.asarray(Image.fromarray(dec).convert("L"))
        return len(pay), dec
    if codec == "jxl":
        d = ic.jpegxl_encode(a, level=None, distance=float(q), effort=9)
        return len(d), ic.jpegxl_decode(d)
    raise ValueError(codec)

def match(codec, a, target, qs):
    """Smallest encoding whose SSIM >= target."""
    best = None
    for q in qs:
        try: n, dec = enc(codec, a, q)
        except Exception as e: continue
        s = ssim(a, dec.astype(np.uint8), data_range=255)
        if s >= target and (best is None or n < best[0]): best = (n, q, s, psnr(a, dec.astype(np.uint8), data_range=255))
    return best

def codecs(args):
    img = ref_image(args.photo, args.scale); a = np.asarray(img)
    n0, dec0 = strips_decode(img)
    s0 = ssim(a, dec0, data_range=255); p0 = psnr(a, dec0, data_range=255)
    print(f"image {img.size[0]}x{img.size[1]} grey. TODAY 22 JPEG strips q75: {n0} B, SSIM {s0:.3f}, PSNR {p0:.1f} dB")
    res = {"size": img.size, "today": {"bytes": n0, "ssim": s0, "psnr": p0}}
    grids = {"jpeg": range(20, 96, 1), "webp": range(5, 101, 1), "avif": range(0, 101, 1),
             "jxl": [x / 20 for x in range(4, 200)]}
    for c, qs in grids.items():
        m = match(c, a, s0, qs)
        if m: print(f"  {c:5s} single image, same SSIM: {m[0]:5d} B ({100*(1-m[0]/n0):4.0f} % less)  q={m[1]}  SSIM {m[2]:.3f} PSNR {m[3]:.1f}")
        else: print(f"  {c:5s}: no setting reaches SSIM {s0:.3f}")
        res[c] = m
    json.dump(res, open(args.out, "w"), indent=1, default=float) if args.out else None

def needed(k, mix, trials, seed=1):
    """Clean frames received (random subset of a long stream: sources first, then mixtures) until all k known."""
    rng = random.Random(seed); out = []
    for t in range(trials):
        order = list(range(k)) + list(range(k, k + 6 * k)); rng.shuffle(order)
        solver = fi.Solver(k, 1); n = 0
        for esi in order:
            n += 1
            if esi < k: cols = [esi]
            elif mix == "dense":
                r = random.Random(esi * 7919 + k + 99991 * t); cols = [i for i in range(k) if r.random() < 0.5] or [r.randrange(k)]
            else:
                old = fi.DENSE_K; fi.DENSE_K = 0; cols = fi.neighbours(1, esi + 10000 * t, k); fi.DENSE_K = old
            solver.add(cols, b"\x00")
            if len(solver.known()) == k: break
        out.append(n)
    return out

def fountain(args):
    for k in args.k:
        for mix in ("lt", "dense"):
            v = needed(k, mix, args.trials)
            v.sort(); print(f"k={k:4d} {mix:5s}: frames needed median {v[len(v)//2]} ({100*(v[len(v)//2]/k-1):.1f} % overhead), p90 {v[int(.9*len(v))]}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("codecs"); c.add_argument("--photo", required=True); c.add_argument("--scale", type=float, default=0.10); c.add_argument("--out")
    f = sub.add_parser("fountain"); f.add_argument("--k", type=int, nargs="+", required=True); f.add_argument("--trials", type=int, default=60)
    a = ap.parse_args(); {"codecs": codecs, "fountain": fountain}[a.cmd](a)
