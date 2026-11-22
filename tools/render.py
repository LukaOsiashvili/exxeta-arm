"""Tiny software renderer: triangle meshes -> shaded PNG (numpy + Pillow only).

Role in the system
------------------
tools/export_all.py uses this to make the pictures for the docs. A normal 3D
renderer needs a GPU or a window system; this one runs anywhere Python runs
(Windows, macOS, Linux, WSL, CI), which matters because every team member must
be able to regenerate the docs.

How it works: orthographic camera, z-buffer, flat shading, dark outlines where
the depth jumps. Good enough for assembly drawings, not for beauty shots.
"""
from __future__ import annotations

import math

import numpy as np
from PIL import Image

COLORS = {            # two-colour look of the arm + neutral bought parts
    "body": (232, 232, 228),
    "accent": (40, 40, 44),
    "vendor": (120, 140, 170),
    "ghost": (200, 80, 60),
}


def tessellate(solid, tol=0.25, ang=0.5):
    """build123d solid -> (vertices Nx3, triangles Mx3) numpy arrays."""
    # Done with the OCP kernel directly so that a face the mesher could not
    # triangulate (rare, tiny sliver faces) is skipped instead of raising.
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_REVERSED
    from OCP.TopLoc import TopLoc_Location

    BRepMesh_IncrementalMesh(solid.wrapped, tol, False, ang, True)
    verts, tris, base = [], [], 0
    for face in solid.faces():
        loc = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face.wrapped, loc)
        if poly is None:
            continue
        trsf = loc.Transformation()
        n = poly.NbNodes()
        for i in range(1, n + 1):
            p = poly.Node(i).Transformed(trsf)
            verts.append((p.X(), p.Y(), p.Z()))
        flip = face.wrapped.Orientation() == TopAbs_REVERSED
        for i in range(1, poly.NbTriangles() + 1):
            a, b, c = poly.Triangle(i).Get()
            tris.append((base + a - 1, base + c - 1, base + b - 1) if flip else (base + a - 1, base + b - 1, base + c - 1))
        base += n
    return np.array(verts, dtype=np.float64).reshape(-1, 3), np.array(tris, dtype=np.int64).reshape(-1, 3)


def _view_matrix(azimuth_deg, elevation_deg):
    """Rotation world -> camera. Camera looks along -Zc; Xc right, Yc up. World Z is up."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    # camera position direction (unit vector from the scene towards the camera)
    d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, d)
    right /= np.linalg.norm(right)
    cam_up = np.cross(d, right)
    return np.vstack([right, cam_up, d])


def render(meshes, path, size=(1400, 1050), azimuth=-55, elevation=22, margin=0.06,
           background=(255, 255, 255), supersample=2, outline=True):
    """meshes: list of (vertices Nx3, triangles Mx3, rgb tuple). Writes a PNG to `path`."""
    W, H = size[0] * supersample, size[1] * supersample
    R = _view_matrix(azimuth, elevation)
    cam = [(v @ R.T, t, c) for v, t, c in meshes if len(t)]
    allv = np.vstack([v for v, _, _ in cam])
    lo, hi = allv.min(axis=0), allv.max(axis=0)
    scale = min(W * (1 - 2 * margin) / (hi[0] - lo[0]), H * (1 - 2 * margin) / (hi[1] - lo[1]))
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2

    zbuf = np.full((H, W), -np.inf)
    img = np.empty((H, W, 3), dtype=np.float32)
    img[:] = background
    light = np.array([0.35, 0.45, 0.82])
    light /= np.linalg.norm(light)

    for v, tris, color in cam:
        px = (v[:, 0] - cx) * scale + W / 2
        py = H / 2 - (v[:, 1] - cy) * scale
        pz = v[:, 2]
        a, b, c = v[tris[:, 0]], v[tris[:, 1]], v[tris[:, 2]]
        n = np.cross(b - a, c - a)
        ln = np.linalg.norm(n, axis=1)
        ln[ln == 0] = 1
        n /= ln[:, None]
        shade = 0.38 + 0.62 * np.abs(n @ light)       # two-sided flat shading
        col = np.array(color, dtype=np.float32)
        for i, (i0, i1, i2) in enumerate(tris):
            x0, y0, x1, y1, x2, y2 = px[i0], py[i0], px[i1], py[i1], px[i2], py[i2]
            xmin, xmax = int(max(0, math.floor(min(x0, x1, x2)))), int(min(W - 1, math.ceil(max(x0, x1, x2))))
            ymin, ymax = int(max(0, math.floor(min(y0, y1, y2)))), int(min(H - 1, math.ceil(max(y0, y1, y2))))
            if xmin > xmax or ymin > ymax:
                continue
            den = (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2)
            if abs(den) < 1e-12:
                continue
            xs = np.arange(xmin, xmax + 1) + 0.5
            ys = np.arange(ymin, ymax + 1) + 0.5
            gx, gy = np.meshgrid(xs, ys)
            w0 = ((y1 - y2) * (gx - x2) + (x2 - x1) * (gy - y2)) / den
            w1 = ((y2 - y0) * (gx - x2) + (x0 - x2) * (gy - y2)) / den
            w2 = 1 - w0 - w1
            inside = (w0 >= -1e-6) & (w1 >= -1e-6) & (w2 >= -1e-6)
            if not inside.any():
                continue
            z = w0 * pz[i0] + w1 * pz[i1] + w2 * pz[i2]
            zb = zbuf[ymin:ymax + 1, xmin:xmax + 1]
            hit = inside & (z > zb)
            zb[hit] = z[hit]
            img[ymin:ymax + 1, xmin:xmax + 1][hit] = col * shade[i]

    if outline:
        # darken pixels where the depth jumps (silhouettes and steps)
        z = np.where(np.isfinite(zbuf), zbuf, lo[2] - 50)
        dz = np.zeros_like(z)
        dz[1:, :] = np.maximum(dz[1:, :], np.abs(z[1:, :] - z[:-1, :]))
        dz[:, 1:] = np.maximum(dz[:, 1:], np.abs(z[:, 1:] - z[:, :-1]))
        edge = dz > 1.2
        for _ in range(supersample - 1):              # thicken a little at high resolution
            e = edge.copy()
            e[1:, :] |= edge[:-1, :]
            e[:, 1:] |= edge[:, :-1]
            edge = e
        img[edge] = img[edge] * 0.25

    out = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8))
    if supersample > 1:
        out = out.resize(size, Image.LANCZOS)
    out.save(path)
    return path
