"""Make shareable pictures of the arm: two JPG renders and one animated GIF.

Output: docs/img/exxeta_arm_1.jpg, exxeta_arm_2.jpg, exxeta_arm.gif
Run:    python tools/make_media.py     (needs the CAD environment, takes a few minutes)
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "cad"))
sys.path.insert(0, str(ROOT / "tools"))

from PIL import Image  # noqa: E402

import assembly  # noqa: E402
from render import COLORS, render, tessellate  # noqa: E402

IMG = ROOT / "docs" / "img"


def main():
    for i, src in enumerate(("assembly_home_front.png", "assembly_home.png"), 1):
        Image.open(IMG / src).convert("RGB").save(IMG / f"exxeta_arm_{i}.jpg", quality=90)

    # tessellate every part once in its link frame, then only move the vertices per frame
    cache = {}
    parts = []
    for pd, link, loc in assembly.part_instances():
        if pd.name not in cache:
            cache[pd.name] = pd.make()
        parts.append((link, tessellate(loc * cache[pd.name], 0.4, 0.7), COLORS[pd.color]))
    for _, link, solid in assembly.vendor_instances():
        parts.append((link, tessellate(solid, 0.4, 0.7), COLORS["vendor"]))

    import numpy as np
    keys = [assembly.POSES[k] for k in ("rest", "home", "reach", "home")]
    keys.append((40, 30, 60, 30, 60, 0))
    keys.append(assembly.POSES["rest"])
    frames, steps = [], 8
    for a, b in zip(keys, keys[1:]):
        for k in range(steps):
            s = k / steps
            s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q = [x + s * (y - x) for x, y in zip(a, b)]
            fr = assembly.link_frames(q)
            meshes = []
            for link, (v, t), col in parts:
                loc = fr[link]
                m = np.array([[loc.wrapped.Transformation().Value(r, c) for c in range(1, 5)] for r in range(1, 4)])
                meshes.append((v @ m[:, :3].T + m[:, 3], t, col))
            # fixed-size invisible corner points keep the camera framing identical in every frame
            box = np.array([[-110, -200, 0], [300, 200, 0], [-110, -200, 380], [300, 200, 380]], dtype=float)
            for corner in box:                                   # one tiny (invisible) triangle per corner
                tiny = corner + np.array([[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0]])
                meshes.append((tiny, np.array([[0, 1, 2]]), (255, 255, 255)))
            tmp = IMG / "_frame.png"
            render(meshes, tmp, size=(560, 480), azimuth=-55, elevation=18, supersample=1, outline=True)
            frames.append(Image.open(tmp).convert("P", palette=Image.ADAPTIVE, colors=64))
            print(f"frame {len(frames)}", flush=True)
    (IMG / "_frame.png").unlink()
    frames[0].save(IMG / "exxeta_arm.gif", save_all=True, append_images=frames[1:], duration=90, loop=0)
    print("done")


if __name__ == "__main__":
    main()
