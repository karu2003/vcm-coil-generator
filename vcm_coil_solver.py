# SPDX-License-Identifier: GPL-2.0-only
# VCM Coil Generator — solver
# Computes waypoint coordinates for PCB spiral coil tracks.
# Coordinates are in KiCad internal units (nm), origin at coil centre.

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

CoilShape = Literal["rect", "circle", "sector"]


@dataclass
class CoilWaypoints:
    """Result from a solver call: list of (x, y) segment endpoint pairs."""
    shape: CoilShape
    n_turns: int
    # List of straight segments: each item is ((x0,y0), (x1,y1)) in nm
    segments: list[tuple[tuple[int, int], tuple[int, int]]]
    # Total wire length (mm, informational)
    l_wire_mm: float
    # Length of active segments ⊥ B (mm)
    l_active_mm: float
    # Start and end pads (x, y) nm — for connecting to via / terminal
    start_pt: tuple[int, int]
    end_pt: tuple[int, int]


def _nm(mm: float) -> int:
    """Convert mm → KiCad internal units (nm)."""
    return int(round(mm * 1e6))


# ─────────────────────────────────────────────────────────────────────────────
#  Rectangular spiral
# ─────────────────────────────────────────────────────────────────────────────

def rect_spiral(
    a_mm: float,          # outer dimension along X (width)
    b_mm: float,          # outer dimension along Y (height)
    w_mm: float,          # track width
    s_mm: float,          # gap between tracks
    cx_mm: float = 0.0,   # coil centre X
    cy_mm: float = 0.0,   # coil centre Y
    outward: bool = False, # False = outside→inside, True = inside→outside
    a_in_mm: float = 0.0,  # inner void width
    b_in_mm: float = 0.0,  # inner void height
    start_corner: int = 0, # 0=SW, 1=SE, 2=NE, 3=NW
    end_sides: int = 3,    # sides on last turn (1-4)
    start_mid_edge: bool = False,  # start at edge midpoint (for outer VIA)
    end_mid_edge: bool = False,    # end at edge midpoint (for outer VIA)
) -> CoilWaypoints:
    """
    Rectangular spiral — always CCW, arbitrary start corner.

    When *start_mid_edge* is True the spiral begins at the centre of the
    edge arriving at *start_corner* (e.g. mid-right for NE).  A half-edge
    segment connects the midpoint to the corner, then the normal spiral
    follows.  The outer VIA connects straight to the midpoint.

    When *end_mid_edge* is True the last turn ends at the centre of its
    final edge (half-side) instead of the full corner.  The outer VIA
    connects straight from the midpoint.

    Angle convention:  0° = right (mid-right),  90° = top,
    180° = left,  270° = bottom.
    """
    pitch = w_mm + s_mm
    cx, cy = cx_mm, cy_mm
    min_xa = a_in_mm / 2.0
    min_xb = b_in_mm / 2.0

    turns: list[tuple[float, float]] = []
    ha, hb = a_mm / 2.0, b_mm / 2.0
    while True:
        xa = ha - w_mm / 2.0
        xb = hb - w_mm / 2.0
        if xa <= min_xa or xb <= min_xb:
            break
        turns.append((xa, xb))
        ha -= pitch
        hb -= pitch

    ordered = list(reversed(turns)) if outward else turns
    n_turns = len(ordered)
    sc = start_corner % 4
    es = max(1, min(4, end_sides))

    _horiz = [True, False, True, False]   # bottom, right, top, left

    def _corners(xa, xb):
        return [
            (cx - xa, cy - xb),   # 0 SW
            (cx + xa, cy - xb),   # 1 SE
            (cx + xa, cy + xb),   # 2 NE
            (cx - xa, cy + xb),   # 3 NW
        ]

    def _edge_mid(c, c_from, c_to):
        """Midpoint of the edge from corner c_from to corner c_to."""
        return ((c[c_from][0] + c[c_to][0]) / 2.0,
                (c[c_from][1] + c[c_to][1]) / 2.0)

    raw_segs: list[tuple[tuple[float, float], tuple[float, float]]] = []
    l_wire = 0.0
    l_active = 0.0

    # ── start_mid_edge: compute the midpoint on the leaving edge ────────
    # Instead of a separate lead segment (which overlaps with the first
    # clean side), we REPLACE the first clean side with a half-side that
    # begins at mid and goes to SC+1.
    _sme_mid = None
    if start_mid_edge and n_turns > 0:
        c0 = _corners(*ordered[0])
        _sme_mid = _edge_mid(c0, sc, (sc + 1) % 4)

    # Suppress end_mid_edge for outward + es=4 + multi-turn because the
    # distributed step's arrival segment overlaps with the 4th half-side.
    # Also reduce es to 3: the full 4th side also overlaps with the arrival.
    eff_eme = end_mid_edge and not (outward and es == 4 and n_turns > 1)
    if end_mid_edge and not eff_eme:
        es = 3

    for i, (xa, xb) in enumerate(ordered):
        c = _corners(xa, xb)
        l_wire += 2.0 * (2.0 * xa) + 2.0 * (2.0 * xb)
        l_active += 2.0 * (2.0 * xb)

        is_first = (i == 0)
        skip_k0 = (_sme_mid is not None and is_first)

        if i < n_turns - 1:
            xa_n, xb_n = ordered[i + 1]
            c_n = _corners(xa_n, xb_n)

            if outward:
                for k in range(4):
                    si = (sc + k) % 4
                    p0i = si
                    p1i = (si + 1) % 4
                    if skip_k0 and k == 0:
                        raw_segs.append((_sme_mid, c[p1i]))
                    elif k < 2:
                        raw_segs.append((c[p0i], c[p1i]))
                    elif k == 2:
                        if _horiz[si]:
                            p1 = (c_n[p1i][0], c[p1i][1])
                        else:
                            p1 = (c[p1i][0], c_n[p1i][1])
                        raw_segs.append((c[p0i], p1))
                    else:
                        p0 = raw_segs[-1][1]
                        raw_segs.append((p0, c_n[sc]))
            else:
                if skip_k0:
                    raw_segs.append((_sme_mid, c[(sc + 1) % 4]))
                for k in range(1 if skip_k0 else 0, 3):
                    si = (sc + k) % 4
                    raw_segs.append((c[si], c[(si + 1) % 4]))
                corner_c = (sc + 3) % 4
                si_4 = corner_c
                if _horiz[si_4]:
                    mid = (c_n[sc][0], c[corner_c][1])
                else:
                    mid = (c[corner_c][0], c_n[sc][1])
                raw_segs.append((c[corner_c], mid))
                raw_segs.append((mid, c_n[sc]))
        else:
            # ── Last turn ────────────────────────────────────────────────
            if eff_eme:
                if skip_k0:
                    raw_segs.append((_sme_mid, c[(sc + 1) % 4]))
                for k in range(1 if skip_k0 else 0, max(0, es - 1)):
                    si = (sc + k) % 4
                    raw_segs.append((c[si], c[(si + 1) % 4]))
                pen_c = (sc + max(0, es - 1)) % 4
                nxt_c = (sc + es) % 4
                mid = _edge_mid(c, pen_c, nxt_c)
                raw_segs.append((c[pen_c], mid))
            else:
                if skip_k0:
                    raw_segs.append((_sme_mid, c[(sc + 1) % 4]))
                for k in range(1 if skip_k0 else 0, es):
                    si = (sc + k) % 4
                    raw_segs.append((c[si], c[(si + 1) % 4]))

    nm_segs = [
        ((_nm(p0[0]), _nm(p0[1])), (_nm(p1[0]), _nm(p1[1])))
        for p0, p1 in raw_segs
    ]

    if nm_segs:
        start_pt = nm_segs[0][0]
        end_pt   = nm_segs[-1][1]
    else:
        start_pt = (_nm(cx), _nm(cy))
        end_pt   = (_nm(cx), _nm(cy))

    return CoilWaypoints(
        shape="rect",
        n_turns=n_turns,
        segments=nm_segs,
        l_wire_mm=l_wire,
        l_active_mm=l_active,
        start_pt=start_pt,
        end_pt=end_pt,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Circular spiral  (approximated as polygon segments for pcbnew PCB_ARC)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArcWaypoints:
    """Circular/sector coil uses arcs; segments are (centre, start, end) in nm."""
    shape: CoilShape
    n_turns: int
    # List of arc descriptors: (cx, cy, radius_nm, start_angle_rad, end_angle_rad)
    arcs: list[tuple[int, int, int, float, float]]
    # Radial connector segments between turns: ((x0,y0),(x1,y1)) in nm
    segments: list[tuple[tuple[int, int], tuple[int, int]]]
    l_wire_mm: float
    l_active_mm: float
    start_pt: tuple[int, int]
    end_pt: tuple[int, int]


@dataclass(frozen=True)
class SectorInnerViaZone:
    """Fixed through-board inner VIA zone shared by all sector layers."""
    centres_mm: list[tuple[float, float]]
    keep_radius_mm: float
    active_by_layer: dict[int, int]

    @property
    def required_area_mm2(self) -> float:
        return len(self.centres_mm) * math.pi * self.keep_radius_mm * self.keep_radius_mm


def circular_spiral(
    r_out_mm: float,
    r_in_mm: float,
    w_mm: float,
    s_mm: float,
    cx_mm: float = 0.0,
    cy_mm: float = 0.0,
    direction: str = "CW",
    outward: bool = False,
    start_angle_deg: float = 0.0,
    end_angle_deg: float = 0.0,
    segs_per_turn: int = 72,
) -> ArcWaypoints:
    """
    Archimedean spiral for ONE layer.

    outward=False: outer→inner (inward).
    outward=True:  inner→outer (outward).

    start_angle_deg: angle (degrees) where the spiral starts (= previous via angle).
    end_angle_deg:   angle (degrees) where the spiral ends   (= next via angle).

    The spiral winds n_turns full turns PLUS the delta needed to move from
    start_angle to end_angle.  This ensures the spiral endpoint lands exactly
    on the via position at end_angle.
    """
    pitch = w_mm + s_mm
    sign = +1.0 if direction.upper() == "CW" else -1.0

    if outward:
        r_start = r_in_mm  + w_mm / 2.0
        r_end   = r_out_mm - w_mm / 2.0
    else:
        r_start = r_out_mm - w_mm / 2.0
        r_end   = r_in_mm  + w_mm / 2.0

    r_span = abs(r_end - r_start)
    if r_span < pitch or pitch <= 0:
        return ArcWaypoints(shape="circle", n_turns=0, arcs=[],
                            segments=[], l_wire_mm=0.0, l_active_mm=0.0,
                            start_pt=(0, 0), end_pt=(0, 0))

    n_turns = math.floor(r_span / pitch)
    if n_turns < 1:
        return ArcWaypoints(shape="circle", n_turns=0, arcs=[],
                            segments=[], l_wire_mm=0.0, l_active_mm=0.0,
                            start_pt=(0, 0), end_pt=(0, 0))

    # Delta angle to add after n_turns full turns so spiral ends at end_angle.
    # For CW (sign=+1): angles increase.  delta must be in [0, 2π).
    a_start = math.radians(start_angle_deg)
    a_end   = math.radians(end_angle_deg)
    delta   = (sign * (a_end - a_start)) % (2 * math.pi)  # always ≥ 0

    total_angle = sign * (n_turns * 2 * math.pi + delta)
    total_steps = max(1, int(round(abs(total_angle) / (2 * math.pi) * segs_per_turn)))
    dth         = total_angle / total_steps
    dr_per_step = (r_end - r_start) / total_steps

    cx_int = _nm(cx_mm)
    cy_int = _nm(cy_mm)

    segs: list[tuple[tuple[int, int], tuple[int, int]]] = []
    l_wire = 0.0

    th  = a_start
    r   = r_start
    px0 = cx_int + int(_nm(r) * math.cos(th))
    py0 = cy_int + int(_nm(r) * math.sin(th))
    start_pt = (px0, py0)

    for _ in range(total_steps):
        th  += dth
        r   += dr_per_step
        px1 = cx_int + int(_nm(r) * math.cos(th))
        py1 = cy_int + int(_nm(r) * math.sin(th))
        segs.append(((px0, py0), (px1, py1)))
        dx, dy = px1 - px0, py1 - py0
        l_wire += math.sqrt(dx * dx + dy * dy)
        px0, py0 = px1, py1

    end_pt   = (px0, py0)
    l_wire_mm = l_wire / 1e6

    return ArcWaypoints(
        shape="circle",
        n_turns=n_turns,
        arcs=[],
        segments=segs,
        l_wire_mm=l_wire_mm,
        l_active_mm=l_wire_mm,
        start_pt=start_pt,
        end_pt=end_pt,
    )


# ─────────────────────────────────────────────────────────────────────────────
#  Sector / arc spiral
# ─────────────────────────────────────────────────────────────────────────────

def minor_sector_opening_deg(start_angle_deg: float, end_angle_deg: float) -> float:
    """
    Angular opening of the *minor* sector between two rays (degrees).

    Example: start=15, end=345 → minor arc is 30° (not 330°).
    """
    sa = start_angle_deg % 360.0
    ea = end_angle_deg % 360.0
    dccw = (ea - sa) % 360.0
    if dccw <= 180.0:
        return dccw
    return 360.0 - dccw


def _minor_sector_rays_rad(
    start_angle_deg: float, end_angle_deg: float
) -> tuple[float, float, float]:
    """
    Minor sector: CCW math-positive arc from th_a to th_b has angle phi_rad.

    Returns (phi_rad, th_a_rad, th_b_rad).
    """
    sa = start_angle_deg % 360.0
    ea = end_angle_deg % 360.0
    dccw = (ea - sa) % 360.0
    if dccw <= 180.0:
        th_a_deg, th_b_deg = sa, ea
    else:
        th_a_deg, th_b_deg = ea, sa
    phi_deg = minor_sector_opening_deg(start_angle_deg, end_angle_deg)
    phi_rad = math.radians(phi_deg)
    return (
        phi_rad,
        math.radians(th_a_deg),
        math.radians(th_b_deg),
    )


def sector_annulus_corner_points_mm(
    cx_mm: float,
    cy_mm: float,
    r_in_mm: float,
    r_out_mm: float,
    start_angle_deg: float,
    end_angle_deg: float,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float]]:
    """
    Four corners of the annular sector in the vertex order expected by
    section boundary traversal:

    * ``V1 → V2`` — straight side on ray ``θ_a``,
    * ``V2 → V3`` — outer arc,
    * ``V3 → V4`` — straight side on ray ``θ_b``,
    * ``V4 → V1`` — inner arc.

    So:
    * **V1** = inner@θ_a
    * **V2** = outer@θ_a
    * **V3** = outer@θ_b
    * **V4** = inner@θ_b
    """
    phi, th_a, _th_b = _minor_sector_rays_rad(start_angle_deg, end_angle_deg)
    theta_c = th_a + phi * 0.5
    th_left = theta_c + phi * 0.5
    th_right = theta_c - phi * 0.5

    def pol(r_mm: float, th: float) -> tuple[float, float]:
        return (cx_mm + r_mm * math.cos(th), cy_mm + r_mm * math.sin(th))

    return (
        pol(r_in_mm,  th_left),    # V1
        pol(r_out_mm, th_left),    # V2
        pol(r_out_mm, th_right),   # V3
        pol(r_in_mm,  th_right),   # V4
    )


def _segment_intersection_xy(
    p0: tuple[float, float],
    p1: tuple[float, float],
    q0: tuple[float, float],
    q1: tuple[float, float],
) -> tuple[float, float] | None:
    """Intersection of two infinite lines p0–p1 and q0–q1; None if parallel."""
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = q0
    x3, y3 = q1
    denom = (x0 - x1) * (y2 - y3) - (y0 - y1) * (x2 - x3)
    if abs(denom) < 1e-18:
        return None
    t = ((x0 - x2) * (y2 - y3) - (y0 - y2) * (x2 - x3)) / denom
    return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))


def _norm2(dx: float, dy: float) -> tuple[float, float]:
    """Normalise 2-D vector; returns (0, 0) for degenerate input."""
    L = math.hypot(dx, dy)
    return (dx / L, dy / L) if L > 1e-15 else (0.0, 0.0)


def _sector_vertex_diagonal_dir(
    v_from: tuple[float, float],
    v_to: tuple[float, float],
    v_inward: tuple[float, float],
) -> tuple[float, float]:
    """Direction of the 45° diagonal from *v_from* into the sector quad.

    The "ray" at *v_from* points toward *v_to*.  The diagonal bisects the
    angle between that ray and the inward perpendicular (the perpendicular
    that faces *v_inward*).  This is the 45° line into the sector interior
    as described by the user: diagonal = rotate(ray, ±45°) toward interior.

    *v_inward* — any vertex on the interior side (used only to pick the
    correct perpendicular direction; typically the opposite-corner vertex).
    """
    ux, uy = _norm2(v_to[0] - v_from[0], v_to[1] - v_from[1])
    # Left perpendicular (CCW rotation of ray direction)
    lp = (-uy, ux)
    # Choose the perpendicular that points toward the interior (v_inward side)
    ix = v_inward[0] - v_from[0]
    iy = v_inward[1] - v_from[1]
    perp = lp if (lp[0] * ix + lp[1] * iy >= 0.0) else (uy, -ux)
    # Diagonal = bisector of ray and inward-perp (45° to each)
    return _norm2(ux + perp[0], uy + perp[1])


def sector_inner_via_zone_mm(
    cx_mm: float,
    cy_mm: float,
    r_in_mm: float,
    r_out_mm: float,
    start_angle_deg: float,
    end_angle_deg: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Reference points of the inner-VIA zone inside the section.

    In the current routing strategy inner VIA are placed in the spiral inner
    keep-out area (``r < r_in``), so this helper only provides geometric
    references along the two straight section sides:

    * ``zone_start`` — midpoint of side ``V1–V2`` (ray ``θ_a``),
    * ``zone_end``   — midpoint of side ``V4–V3`` (ray ``θ_b``).
    """
    v1, v2, v3, v4 = sector_annulus_corner_points_mm(
        cx_mm, cy_mm, r_in_mm, r_out_mm, start_angle_deg, end_angle_deg
    )
    zone_start = ((v1[0] + v2[0]) * 0.5, (v1[1] + v2[1]) * 0.5)
    zone_end = ((v4[0] + v3[0]) * 0.5, (v4[1] + v3[1]) * 0.5)
    return zone_start, zone_end


def sector_annulus_diagonal_hub_mm(
    cx_mm: float,
    cy_mm: float,
    r_in_mm: float,
    r_out_mm: float,
    start_angle_deg: float,
    end_angle_deg: float,
) -> tuple[float, float]:
    """
    Intersection point of the sector quadrilateral diagonals **V1–V3** and **V2–V4**
    (V1=inner@θ_a, V2=inner@θ_b, V3=outer@θ_b, V4=outer@θ_a).

    Returned as the sector "centroid"; used as a reference point for
    overflow series of inner VIAs (see ``sector_coil_via_centres_mm``).
    For a convex sector this is the unique intersection of the diagonals
    through opposite vertices.

    Falls back to the centroid of all four vertices if degenerate.
    """
    v1, v2, v3, v4 = sector_annulus_corner_points_mm(
        cx_mm, cy_mm, r_in_mm, r_out_mm, start_angle_deg, end_angle_deg
    )
    hit = _segment_intersection_xy(v1, v3, v2, v4)
    if hit is not None:
        return hit
    return (
        (v1[0] + v2[0] + v3[0] + v4[0]) / 4.0,
        (v1[1] + v2[1] + v3[1] + v4[1]) / 4.0,
    )


def _shortest_arc_delta_rad(a_from: float, a_to: float) -> float:
    """Signed CCW angle from *a_from* to *a_to* in (−π, π] (minor arc)."""
    d = (a_to - a_from) % (2.0 * math.pi)
    if d > math.pi:
        d -= 2.0 * math.pi
    return d


def _minor_arc_stub(a_from: float, a_to: float) -> tuple[float, float]:
    """
    Canonical (a0, a1) for the shorter circular arc centre→boundary,
    same convention as racetrack arcs (positive CCW step if d≥0).

    Degenerate (|d| < 1e-15) callers should skip the arc.
    """
    d = _shortest_arc_delta_rad(a_from, a_to)
    return (a_from, a_from + d)


def _line_offset_xy(
    p1: tuple[float, float],
    p2: tuple[float, float],
    r: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Port of kimotor_linalg.line_offset (2-D): shift line by *r* along
    its left-hand normal (positive r → left, negative r → right of p1→p2).
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    L = math.hypot(dx, dy)
    if L < 1e-15:
        return (p1, p2)
    ux, uy = dx / L, dy / L
    nx, ny = -uy, ux  # z × u
    return (
        (p1[0] + r * nx, p1[1] + r * ny),
        (p2[0] + r * nx, p2[1] + r * ny),
    )


def sector_via_pad_radius_mm(vd_mm: float, clr_mm: float) -> float:
    """Copper pad radius + electrical clearance for via–trace spacing."""
    return vd_mm * 0.5 + clr_mm


def sector_first_via_vertex_id(direction: str, layer0_outward: bool) -> int:
    """Метка вершины Секции **V1…V4** (см. ``sector_annulus_corner_points_mm``)
    для VIA[0] спирального слоя.

    * CW  + Inside→Outside (``layer0_outward`` True)  → **V1** (inner@θ_a),
    * CW  + Outside→Inside (``layer0_outward`` False) → **V3** (outer@θ_b),
    * CCW + Inside→Outside (``layer0_outward`` True)  → **V4** (outer@θ_a),
    * CCW + Outside→Inside (``layer0_outward`` False) → **V2** (inner@θ_b).
    """
    cw = direction.strip().upper() == "CW"
    if cw:
        return 1 if layer0_outward else 3
    return 4 if layer0_outward else 2


def sector_first_anchor_is_inner(direction: str, layer0_outward: bool) -> bool:
    """Лежит ли VIA[0] на **внутренней** дуге Секции (V1 / V2 — radius r_in)?

    В текущей геометрии внутренние вершины — V1 и V4; внешние — V2 и V3.
    Из таблицы ``sector_first_via_vertex_id``:

    * CW  + Inside  → V1 (inner) → True
    * CW  + Outside → V3 (outer) → False
    * CCW + Inside  → V4 (inner) → True
    * CCW + Outside → V2 (outer) → False

    Direction only chooses left/right; ``layer0_outward`` chooses inside/outside.
    """
    return bool(layer0_outward)


def sector_via_topology_counts(
    n_layers: int,
    layer0_outward: bool,
    direction: str = "CW",
) -> tuple[int, int]:
    """How many **inner** vs **outer** VIAs in an N-layer series sector coil.

    There are always ``n_layers + 1`` VIAs.  **Inner** VIAs live in the
    central hole (stack on the sector bisector toward the centre).
    **Outer** VIAs live **outside** the winding (stack on the bisector
    beyond ``r_out``).

    The first anchor alternates with **both** ``layer0_outward`` and
    ``direction`` (CW/CCW); see ``sector_first_anchor_is_inner``.
    """
    n_via = n_layers + 1
    first_is_inner = sector_first_anchor_is_inner(direction, layer0_outward)
    if first_is_inner:
        n_inner = (n_via + 1) // 2
    else:
        n_inner = n_via // 2
    n_outer = n_via - n_inner
    return n_inner, n_outer


def sector_inner_via_reserve_area_mm2(
    n_inner: int,
    vd_mm: float,
    clr_mm: float,
) -> float:
    """Conservative total copper footprint for inner VIAs (disks, no overlap)."""
    if n_inner <= 0:
        return 0.0
    pr = sector_via_pad_radius_mm(vd_mm, clr_mm)
    return float(n_inner) * math.pi * pr * pr


def sector_outer_via_reserve_area_mm2(
    n_outer: int,
    vd_mm: float,
    clr_mm: float,
) -> float:
    """Same model for outer VIAs (outside ``r_out``)."""
    if n_outer <= 0:
        return 0.0
    pr = sector_via_pad_radius_mm(vd_mm, clr_mm)
    return float(n_outer) * math.pi * pr * pr


def sector_inner_hole_min_r_in_mm(
    n_inner: int,
    vd_mm: float,
    clr_mm: float,
    w_mm: float,
) -> float:
    """Minimum ``r_in_mm`` so all inner VIA fit inside ``r < r_in``."""
    if n_inner <= 0:
        return 0.0
    step_mm = vd_mm + clr_mm
    pad_clear_mm = sector_via_pad_radius_mm(vd_mm, clr_mm)
    nudge_mm = 1.0e-3
    return (
        pad_clear_mm
        + max(0, n_inner - 1) * step_mm
        + w_mm * 0.5
        + clr_mm
        + vd_mm * 0.5
        + nudge_mm
    )


def sector_coil_via_centres_mm(
    anchor_kind: list[str],
    *,
    cx_mm: float,
    cy_mm: float,
    r_in_mm: float,
    r_out_mm: float,
    start_angle_deg: float,
    end_angle_deg: float,
    vd_mm: float,
    clr_mm: float,
    w_mm: float,
    direction: str = "CW",
    layer0_outward: bool = False,
) -> list[tuple[float, float]]:
    """Series-VIA centres (mm) for the sector coil.

    * VIA[0] is placed at the selected section vertex V1..V4.
    * Inner VIA are placed in the inner keep-out area ``r < r_in`` on the
      sector bisector (so spiral copper does not pass through this VIA area).
    * Outer VIA are stacked beyond ``r_out`` on the bisector.
    """
    step_mm = vd_mm + clr_mm
    nudge_mm = 1.0e-3
    inner_base_r_mm = r_in_mm - w_mm * 0.5 - clr_mm - vd_mm * 0.5 - nudge_mm
    outer_base_r_mm = r_out_mm + w_mm * 0.5 + clr_mm + vd_mm * 0.5 + nudge_mm

    _phi, ta_rad, tb_rad = _minor_sector_rays_rad(start_angle_deg, end_angle_deg)
    delta = tb_rad - ta_rad
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    theta_c = ta_rad + 0.5 * delta

    # Вершины Секции в новой нумерации
    v1, v2, v3, v4 = sector_annulus_corner_points_mm(
        cx_mm, cy_mm, r_in_mm, r_out_mm, start_angle_deg, end_angle_deg,
    )
    vertex_xy = {1: v1, 2: v2, 3: v3, 4: v4}
    vid0 = sector_first_via_vertex_id(direction, layer0_outward)
    first_vertex_xy = vertex_xy[vid0]

    out: list[tuple[float, float]] = []
    inner_stack_slot = 0
    outer_stack_slot = 0  # counts outer VIAs placed at index > 0

    for i, kind in enumerate(anchor_kind):
        if i == 0:
            out.append(first_vertex_xy)

        elif kind == "inner":
            r_via_mm = inner_base_r_mm - inner_stack_slot * step_mm
            inner_stack_slot += 1
            out.append((cx_mm + r_via_mm * math.cos(theta_c),
                        cy_mm + r_via_mm * math.sin(theta_c)))

        else:
            # Внешние VIA: стек на биссектрисе за r_out
            r_via_mm = outer_base_r_mm + outer_stack_slot * step_mm
            th_out = theta_c + outer_stack_slot * 0.035
            outer_stack_slot += 1
            out.append((cx_mm + r_via_mm * math.cos(th_out),
                        cy_mm + r_via_mm * math.sin(th_out)))

    return out


def sector_terminal_via_pocket_centres_mm(
    *,
    anchor_xy_mm: tuple[float, float],
    cx_mm: float,
    cy_mm: float,
    n_vias: int,
    vd_mm: float,
    clr_mm: float,
    w_mm: float,
    side: float = -1.0,
) -> list[tuple[float, float]]:
    """Compact terminal VIA pocket at the end of a sector spiral.

    VIA[0] is the active via for the current layer and is placed directly on
    the radial continuation of the spiral endpoint, so the KiCad action can
    connect it with one straight non-T stub.  Remaining VIAs are reserved for
    following layers and are placed tangentially beside the active via.
    """
    if n_vias <= 0:
        return []

    ax, ay = anchor_xy_mm
    dx = ax - cx_mm
    dy = ay - cy_mm
    r = math.hypot(dx, dy)
    if r < 1.0e-12:
        er = (1.0, 0.0)
    else:
        er = (dx / r, dy / r)
    et = (-er[1], er[0])

    keep_mm = vd_mm * 0.5 + clr_mm + w_mm * 0.5
    step_mm = vd_mm + clr_mm

    offsets: list[tuple[float, float]] = [(keep_mm, 0.0)]
    if n_vias >= 2:
        # Compact two-via footprint: 2 x VIA + one via-to-via gap.  Connections
        # are drawn as direct endpoint-to-via segments, so the side-by-side
        # pocket no longer creates T/F-shaped orthogonal branches.  The second
        # via is nudged slightly inward so direct traces from both neighbouring
        # layers see both pads separately rather than one pad hiding behind the
        # other.
        radial_back_mm = min(step_mm * 0.14, keep_mm * 0.25)
        tangent_mm = math.sqrt(max(0.0, step_mm * step_mm - radial_back_mm * radial_back_mm))
        offsets.append((keep_mm - radial_back_mm, side * tangent_mm))
    for idx in range(2, n_vias):
        row = (idx + 1) // 2
        sign = -side if idx % 2 == 0 else side
        offsets.append((keep_mm + row * step_mm, sign * step_mm * 0.5))

    return [
        (
            ax + er[0] * radial + et[0] * tangent,
            ay + er[1] * radial + et[1] * tangent,
        )
        for radial, tangent in offsets[:n_vias]
    ]


def sector_make_inner_via_zone_mm(
    *,
    anchor_xy_mm: tuple[float, float],
    cx_mm: float,
    cy_mm: float,
    inner_via_indices: list[int],
    n_layers: int,
    vd_mm: float,
    clr_mm: float,
    w_mm: float,
    side: float = -1.0,
) -> SectorInnerViaZone:
    """Create the fixed inner VIA zone after the first inner-ending layer.

    ``inner_via_indices`` are the global series VIA indices that live inside
    the spiral after VIA[0].  The returned zone is shared by all copper layers
    because these are through-board vias.
    """
    centres = sector_terminal_via_pocket_centres_mm(
        anchor_xy_mm=anchor_xy_mm,
        cx_mm=cx_mm,
        cy_mm=cy_mm,
        n_vias=len(inner_via_indices),
        vd_mm=vd_mm,
        clr_mm=clr_mm,
        w_mm=w_mm,
        side=side,
    )
    active_by_layer: dict[int, int] = {}
    for local_idx, via_idx in enumerate(inner_via_indices):
        if 0 < via_idx <= n_layers:
            active_by_layer[via_idx - 1] = local_idx
    return SectorInnerViaZone(
        centres_mm=centres,
        keep_radius_mm=vd_mm * 0.5 + clr_mm + w_mm * 0.5,
        active_by_layer=active_by_layer,
    )


def sector_inner_via_required_area_mm2(
    n_vias: int,
    vd_mm: float,
    clr_mm: float,
    w_mm: float,
) -> float:
    """Required compact area for the through inner-VIA group.

    The area is the physical VIA group footprint, not a sum of circular
    centreline keepouts:

      radial size     = via_dia + 2 * via_to_track_gap
      tangential size = N * via_dia + (N - 1) * via_to_via_gap
                        + 2 * via_to_track_gap

    ``clr_mm`` is used for both via-to-via edge gap and via-to-track edge gap.
    Track-centre clearance is still checked separately with
    ``vd/2 + clr + w/2``.
    """
    if n_vias <= 0:
        return 0.0
    del w_mm
    radial_size = vd_mm + 2.0 * clr_mm
    tangential_size = (
        float(n_vias) * vd_mm
        + float(max(0, n_vias - 1)) * clr_mm
        + 2.0 * clr_mm
    )
    return radial_size * tangential_size


def _sector_free_area_mm2(phi_rad: float, r_inner_mm: float, r_outer_mm: float) -> float:
    if r_outer_mm <= r_inner_mm:
        return 0.0
    return 0.5 * phi_rad * (r_outer_mm * r_outer_mm - r_inner_mm * r_inner_mm)


def sector_dynamic_inner_via_zone_mm(
    *,
    cx_mm: float,
    cy_mm: float,
    r_in_mm: float,
    r_out_mm: float,
    w_mm: float,
    s_mm: float,
    start_angle_deg: float,
    end_angle_deg: float,
    inner_via_indices: list[int],
    n_layers: int,
    vd_mm: float,
    clr_mm: float,
    side: float = -1.0,
) -> SectorInnerViaZone | None:
    """Fix the inner VIA zone by consuming spiral area segment-by-segment.

    The zone is not picked by static radius search.  It follows the remaining
    free pocket while the first layer is routed.  Before accepting the next
    pitch step we compare the remaining annular-sector area with the required
    VIA keepout area; the zone is fixed at the last available pocket when the
    next step would make the remaining area too small.
    """
    if not inner_via_indices:
        return None

    phi_rad, th_a, _th_b = _minor_sector_rays_rad(start_angle_deg, end_angle_deg)
    pitch = w_mm + s_mm
    if phi_rad <= 0.0 or pitch <= 0.0:
        return None

    required_area = sector_inner_via_required_area_mm2(
        len(inner_via_indices), vd_mm, clr_mm, w_mm,
    )
    keep = vd_mm * 0.5 + clr_mm + w_mm * 0.5
    theta_c = th_a + phi_rad * 0.5

    r_inner = r_in_mm
    r_outer = r_out_mm
    last_anchor_r = r_inner + keep
    while True:
        r_next_inner = r_inner + pitch
        r_next_outer = r_outer - pitch
        free_now = _sector_free_area_mm2(phi_rad, r_inner, r_outer)
        free_next = _sector_free_area_mm2(phi_rad, r_next_inner, r_next_outer)

        if free_now <= required_area or free_next < required_area:
            # Fix the zone in the centre of the remaining pocket.  Clamp so the
            # active via row still fits radially inside the pocket.
            pocket_mid = 0.5 * (r_inner + r_outer)
            # ``sector_terminal_via_pocket_centres_mm`` places the active via
            # one keepout radius outward from the anchor.  Clamp the resulting
            # via centre to the remaining free pocket, not the anchor itself.
            last_anchor_r = max(r_inner, min(pocket_mid - keep, r_outer - 2.0 * keep))
            break

        r_inner = r_next_inner
        r_outer = r_next_outer

    anchor_xy = (
        cx_mm + last_anchor_r * math.cos(theta_c),
        cy_mm + last_anchor_r * math.sin(theta_c),
    )
    return sector_make_inner_via_zone_mm(
        anchor_xy_mm=anchor_xy,
        cx_mm=cx_mm,
        cy_mm=cy_mm,
        inner_via_indices=inner_via_indices,
        n_layers=n_layers,
        vd_mm=vd_mm,
        clr_mm=clr_mm,
        w_mm=w_mm,
        side=side,
    )


def _point_segment_distance_mm(
    p: tuple[float, float],
    a: tuple[float, float],
    b: tuple[float, float],
) -> float:
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    wx = p[0] - a[0]
    wy = p[1] - a[1]
    length2 = vx * vx + vy * vy
    if length2 < 1.0e-18:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length2))
    qx = a[0] + t * vx
    qy = a[1] + t * vy
    return math.hypot(p[0] - qx, p[1] - qy)


def _angle_on_short_arc(angle: float, a0: float, a1: float) -> bool:
    sweep = a1 - a0
    while sweep > math.pi:
        sweep -= 2.0 * math.pi
    while sweep <= -math.pi:
        sweep += 2.0 * math.pi
    delta = angle - a0
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta <= -math.pi:
        delta += 2.0 * math.pi
    if sweep >= 0.0:
        return -1.0e-12 <= delta <= sweep + 1.0e-12
    return sweep - 1.0e-12 <= delta <= 1.0e-12


def _point_arc_distance_mm(
    p: tuple[float, float],
    cx_mm: float,
    cy_mm: float,
    radius_mm: float,
    a0: float,
    a1: float,
) -> float:
    dx = p[0] - cx_mm
    dy = p[1] - cy_mm
    rp = math.hypot(dx, dy)
    if rp < 1.0e-12:
        return radius_mm
    angle = math.atan2(dy, dx)
    if _angle_on_short_arc(angle, a0, a1):
        return abs(rp - radius_mm)
    p0 = (cx_mm + radius_mm * math.cos(a0), cy_mm + radius_mm * math.sin(a0))
    p1 = (cx_mm + radius_mm * math.cos(a1), cy_mm + radius_mm * math.sin(a1))
    return min(math.hypot(p[0] - p0[0], p[1] - p0[1]),
               math.hypot(p[0] - p1[0], p[1] - p1[1]))


def _piece_clear_of_inner_zone(
    piece: tuple,
    *,
    cx_mm: float,
    cy_mm: float,
    zone: SectorInnerViaZone | None,
) -> bool:
    if zone is None or not zone.centres_mm:
        return True
    for via in zone.centres_mm:
        if piece[0] == "line":
            _kind, r0, a0, r1, a1 = piece
            p0 = (cx_mm + r0 * math.cos(a0), cy_mm + r0 * math.sin(a0))
            p1 = (cx_mm + r1 * math.cos(a1), cy_mm + r1 * math.sin(a1))
            distance = _point_segment_distance_mm(via, p0, p1)
        else:
            _kind, radius, a0, a1 = piece
            distance = _point_arc_distance_mm(via, cx_mm, cy_mm, radius, a0, a1)
        if distance < zone.keep_radius_mm - 1.0e-9:
            return False
    return True


def sector_inner_stub_path(
    end_xy_mm: tuple[float, float],
    via_xy_mm: tuple[float, float],
    avoid_xy_mm: list[tuple[float, float]],
    keep_mm: float,
) -> list[tuple[float, float]]:
    """Direct inner-zone connection without extra bends.

    Keep a single segment to avoid sharp-angle artefacts in compact pockets.
    ``avoid_xy_mm`` / ``keep_mm`` are preserved only for API compatibility.
    """
    del avoid_xy_mm, keep_mm
    return [end_xy_mm, via_xy_mm]


def _arc_min_clearance_mm(
    cx_mm: float,
    cy_mm: float,
    radius_mm: float,
    a0: float,
    a1: float,
    avoid_pts_mm: list[tuple[float, float]],
) -> float:
    if radius_mm <= 0.0 or not avoid_pts_mm:
        return float("inf")
    best = float("inf")
    for px, py in avoid_pts_mm:
        d = _point_arc_distance_mm(
            (px, py), cx_mm, cy_mm, radius_mm, a0, a1,
        )
        if d < best:
            best = d
    return best


def sector_optimize_direct_inner_stub(
    coil: ArcWaypoints,
    *,
    via_xy_mm: tuple[float, float],
    avoid_vias_mm: list[tuple[float, float]],
    keep_mm: float,
    cx_mm: float,
    cy_mm: float,
    max_delta_deg: float = 18.0,
    step_deg: float = 0.05,
) -> ArcWaypoints:
    """Trim the last inner arc so the direct VIA stub clears foreign pads.

    The inner-zone connection is a single straight segment from
    ``coil.end_pt`` to the active through-via.  In the compact two-via pocket
    this segment can pass within keepout of the reserved (foreign) via.

    The fix is geometric, not topological: we shorten the *last existing arc*
    of the spiral by reducing its sweep, which moves the endpoint along the
    natural arc curve (no new chord across the zone).  The arc itself stays
    visible as a regular spiral arc, so there is no disconnected artefact and
    no long horizontal segment cutting through the keepout zone.
    """
    if not avoid_vias_mm:
        return coil

    end_mm = (coil.end_pt[0] / 1e6, coil.end_pt[1] / 1e6)

    def _stub_clearance(p: tuple[float, float]) -> float:
        return min(
            _point_segment_distance_mm(avoid, p, via_xy_mm)
            for avoid in avoid_vias_mm
        )

    if _stub_clearance(end_mm) >= keep_mm - 1.0e-9:
        return coil

    if coil.arcs:
        cx_nm, cy_nm, r_nm, a0, a1 = coil.arcs[-1]
        radius_mm = r_nm / 1e6
        sweep_a1 = a1 - a0

        steps = max(1, int(round(max_delta_deg / step_deg)))
        best_angle = a1
        best_clearance = _stub_clearance(end_mm)
        first_safe_angle: float | None = None

        for step in range(1, steps + 1):
            improved = False
            for sign in (1.0, -1.0):
                delta = math.radians(step * step_deg * sign)
                cand_angle = a1 + delta
                cand_sweep = cand_angle - a0
                # Refuse trims that flip arc direction or extend it beyond a
                # half turn, both would tangle the spiral path.
                if sweep_a1 > 0 and cand_sweep <= 0.0:
                    continue
                if sweep_a1 < 0 and cand_sweep >= 0.0:
                    continue
                if abs(cand_sweep) > math.pi:
                    continue
                cand_pt = (
                    cx_mm + radius_mm * math.cos(cand_angle),
                    cy_mm + radius_mm * math.sin(cand_angle),
                )
                clearance = _stub_clearance(cand_pt)
                # The (now shortened) terminal arc must itself stay clear of
                # all foreign vias.  Active via is excluded -- it is the
                # endpoint of the stub.
                arc_clear = _arc_min_clearance_mm(
                    cx_mm, cy_mm, radius_mm, a0, cand_angle,
                    avoid_vias_mm,
                )
                effective = min(clearance, arc_clear)
                if effective > best_clearance:
                    best_clearance = effective
                    best_angle = cand_angle
                    improved = True
                if effective >= keep_mm - 1.0e-9:
                    first_safe_angle = cand_angle
                    break
            if first_safe_angle is not None:
                best_angle = first_safe_angle
                break
            if not improved and step > steps // 4:
                # Saturated -- further trimming will not change clearance.
                continue

        if best_angle == a1:
            return coil

        new_end = (
            _nm(cx_mm + radius_mm * math.cos(best_angle)),
            _nm(cy_mm + radius_mm * math.sin(best_angle)),
        )
        new_sweep = best_angle - a0
        # Never drop the terminal arc completely. Removing it while moving
        # end_pt can leave a tiny disconnected copper artefact on reversed
        # inner-start layers (e.g. L3).
        new_arcs = list(coil.arcs[:-1]) + [
            (cx_nm, cy_nm, r_nm, a0, best_angle),
        ]
        new_l_wire = (
            coil.l_wire_mm
            - radius_mm * abs(sweep_a1)
            + radius_mm * abs(new_sweep)
        )
        return ArcWaypoints(
            shape=coil.shape,
            n_turns=coil.n_turns,
            arcs=new_arcs,
            segments=list(coil.segments),
            l_wire_mm=new_l_wire,
            l_active_mm=new_l_wire,
            start_pt=coil.start_pt,
            end_pt=new_end,
        )

    if coil.segments:
        (x0_nm, y0_nm), (x1_nm, y1_nm) = coil.segments[-1]
        p0 = (x0_nm / 1e6, y0_nm / 1e6)
        p1 = (x1_nm / 1e6, y1_nm / 1e6)
        best_t = 1.0
        best_clearance = _stub_clearance(p1)
        first_safe: tuple[float, float] | None = None
        for i in range(1, 101):
            t = 1.0 - i * 0.005
            cand = (p0[0] + (p1[0] - p0[0]) * t,
                    p0[1] + (p1[1] - p0[1]) * t)
            clearance = _stub_clearance(cand)
            if clearance > best_clearance:
                best_clearance = clearance
                best_t = t
            if clearance >= keep_mm - 1.0e-9:
                first_safe = (t, clearance)
                break
        if first_safe is not None:
            best_t = first_safe[0]
        if best_t == 1.0:
            return coil
        new_end = (
            _nm(p0[0] + (p1[0] - p0[0]) * best_t),
            _nm(p0[1] + (p1[1] - p0[1]) * best_t),
        )
        new_segments = list(coil.segments)
        new_segments[-1] = ((x0_nm, y0_nm), new_end)
        return ArcWaypoints(
            shape=coil.shape,
            n_turns=coil.n_turns,
            arcs=list(coil.arcs),
            segments=new_segments,
            l_wire_mm=coil.l_wire_mm,
            l_active_mm=coil.l_active_mm,
            start_pt=coil.start_pt,
            end_pt=new_end,
        )

    return coil


def _circle_line_intersect_upper(
    p1: tuple[float, float],
    p2: tuple[float, float],
    r: float,
) -> tuple[float, float] | None:
    """Intersection of the line through *p1, p2* with circle x²+y² = r².
    Returns the intersection with larger y (upper half), or None when there
    is no intersection.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    a = dx * dx + dy * dy
    if a < 1e-30:
        return None
    fx, fy = p1[0], p1[1]
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    sd = math.sqrt(disc)
    t1 = (-b + sd) / (2.0 * a)
    t2 = (-b - sd) / (2.0 * a)
    p_a = (p1[0] + t1 * dx, p1[1] + t1 * dy)
    p_b = (p1[0] + t2 * dx, p1[1] + t2 * dy)
    return p_a if p_a[1] >= p_b[1] else p_b


def _sector_spiral_ray_arc(
    r_out_mm: float,
    r_in_mm: float,
    w_mm: float,
    s_mm: float,
    cx_mm: float,
    cy_mm: float,
    direction: str,
    outward: bool,
    start_angle_deg: float,
    end_angle_deg: float,
    terminal_via_count: int,
    via_dia_mm: float,
    via_clr_mm: float,
    terminal_trim: bool,
    inner_via_zone: SectorInnerViaZone | None = None,
) -> ArcWaypoints:
    """Sector spiral using the verified V1->V2 ray, arc, ray, arc topology."""
    phi_rad, th_a, _th_b = _minor_sector_rays_rad(start_angle_deg, end_angle_deg)
    cx_int = _nm(cx_mm)
    cy_int = _nm(cy_mm)
    pitch = w_mm + s_mm

    def _empty() -> ArcWaypoints:
        return ArcWaypoints("sector", 0, [], [], 0.0, 0.0, (cx_int, cy_int), (cx_int, cy_int))

    if phi_rad < 1e-15 or pitch <= 0.0:
        return _empty()

    theta_c = th_a + phi_rad * 0.5
    left0 = theta_c + phi_rad * 0.5
    right0 = theta_c - phi_rad * 0.5
    is_cw = direction.strip().upper() == "CW"
    inside_start = sector_first_anchor_is_inner(direction, outward)

    via_keep = via_dia_mm * 0.5 + via_clr_mm + w_mm * 0.5 if terminal_via_count > 0 else 0.0
    # The active and reserved terminal vias share one radial row; radial depth
    # only needs one via keepout on each side of that row.
    terminal_depth = 2.0 * via_keep if terminal_via_count > 0 and via_dia_mm > 0.0 else 0.0

    def pol(r_mm: float, a_rad: float) -> tuple[int, int]:
        return (
            _nm(cx_mm + r_mm * math.cos(a_rad)),
            _nm(cy_mm + r_mm * math.sin(a_rad)),
        )

    arcs: list[tuple[int, int, int, float, float]] = []
    segs: list[tuple[tuple[int, int], tuple[int, int]]] = []
    l_wire_mm = 0.0
    start_pt: tuple[int, int] | None = None
    end_pt: tuple[int, int] | None = None

    r_inner = r_in_mm
    r_outer = r_out_mm
    turn = 0

    while True:
        r_next = r_inner + pitch
        if r_next + terminal_depth > r_outer + 1e-9:
            break

        r_mid = 0.5 * (r_inner + r_outer)
        angular_step = pitch / max(r_mid, 1e-9)
        left = left0 - turn * angular_step
        right = right0 + turn * angular_step
        left_next = left0 - (turn + 1) * angular_step
        right_next = right0 + (turn + 1) * angular_step
        if left - right <= math.radians(1.0):
            break

        final_turn = (r_next + pitch + terminal_depth > r_outer - pitch + 1e-9)
        trim_final = terminal_trim and terminal_via_count > 0 and final_turn
        end_angle = theta_c if trim_final else left_next

        if is_cw and inside_start:
            pieces = [
                ("line", r_inner, left, r_outer, left),
                ("arc", r_outer, left, right),
                ("line", r_outer, right, r_next, right),
                ("arc", r_next, right, end_angle),
            ]
        elif is_cw and not inside_start:
            pieces = [
                ("line", r_outer, right, r_inner, right),
                ("arc", r_inner, right, left_next),
                ("line", r_inner, left_next, r_outer - pitch, left_next),
                ("arc", r_outer - pitch, left_next, theta_c if trim_final else right_next),
            ]
        elif (not is_cw) and inside_start:
            pieces = [
                ("line", r_inner, right, r_outer, right),
                ("arc", r_outer, right, left),
                ("line", r_outer, left, r_next, left),
                ("arc", r_next, left, theta_c if trim_final else right_next),
            ]
        else:
            pieces = [
                ("line", r_outer, left, r_inner, left),
                ("arc", r_inner, left, right_next),
                ("line", r_inner, right_next, r_outer - pitch, right_next),
                ("arc", r_outer - pitch, right_next, theta_c if trim_final else left_next),
            ]

        # Do not add a degenerate terminal arc after trimming to the centre.
        for piece in pieces:
            if not _piece_clear_of_inner_zone(
                piece,
                cx_mm=cx_mm,
                cy_mm=cy_mm,
                zone=inner_via_zone,
            ):
                return ArcWaypoints(
                    shape="sector",
                    n_turns=turn,
                    arcs=arcs,
                    segments=segs,
                    l_wire_mm=l_wire_mm,
                    l_active_mm=l_wire_mm,
                    start_pt=start_pt or (cx_int, cy_int),
                    end_pt=end_pt or (cx_int, cy_int),
                )
            if piece[0] == "line":
                _kind, r0, a0, r1, a1 = piece
                p0 = pol(r0, a0)
                p1 = pol(r1, a1)
                if start_pt is None:
                    start_pt = p0
                segs.append((p0, p1))
                end_pt = p1
                l_wire_mm += math.hypot(r1 * math.cos(a1) - r0 * math.cos(a0),
                                        r1 * math.sin(a1) - r0 * math.sin(a0))
            else:
                _kind, radius, a0, a1 = piece
                if abs(a1 - a0) < 1e-12:
                    continue
                if start_pt is None:
                    start_pt = pol(radius, a0)
                arcs.append((cx_int, cy_int, _nm(radius), a0, a1))
                end_pt = pol(radius, a1)
                l_wire_mm += radius * abs(a1 - a0)

        r_inner = r_next
        r_outer -= pitch
        turn += 1

    if start_pt is None or end_pt is None:
        return _empty()

    return ArcWaypoints(
        shape="sector",
        n_turns=turn,
        arcs=arcs,
        segments=segs,
        l_wire_mm=l_wire_mm,
        l_active_mm=l_wire_mm,
        start_pt=start_pt,
        end_pt=end_pt,
    )


def sector_spiral(
    r_out_mm: float,
    r_in_mm: float,
    w_mm: float,
    s_mm: float,
    cx_mm: float = 0.0,
    cy_mm: float = 0.0,
    direction: str = "CW",
    outward: bool = False,
    start_angle_deg: float = 0.0,
    end_angle_deg: float = 180.0,
    segs_per_sweep: int = 36,
    terminal_via_count: int = 0,
    via_dia_mm: float = 0.0,
    via_clr_mm: float = 0.0,
    terminal_trim: bool = True,
    inner_via_zone: SectorInnerViaZone | None = None,
) -> ArcWaypoints:
    """
    Спираль в кольцевом секторе (cooked/kimotor :: kimotor_solver.radial),
    но с **внешним витком, выровненным точно по границе Секции
    V1‑V2‑V3‑V4** (см. ``sector_annulus_corner_points_mm``).

    * Внешний виток (k=0) = граница Секции (без pitch‑смещения), поэтому
      первый сегмент идёт от одной вершины Секции к соседней.
    * Каждый следующий виток получается перпендикулярным сдвигом
      «верхней» стороны предыдущего витка на ``pitch = w + s`` внутрь
      Секции, как и раньше.

    На каждом витке трасса: внутренняя дуга → внешняя радиаль → внешняя
    дуга → переход к следующему витку (последний виток — без перехода).

    Старт спирали зависит от пары ``(direction, outward)`` так же, как
    в ``sector_first_via_vertex_id``:

    * **Inside‑режим** (``first_is_inner=True``): спираль стартует в
      вершине **внутренней** дуги (V1 для CW, V2 для CCW = ``wp4_0`` в
      локальной системе), первый сегмент — внутренняя дуга к соседней
      вершине, дальше идёт полный путь по внешнему витку.
    * **Outside‑режим** (``first_is_inner=False``): спираль стартует в
      вершине **внешней** дуги (V3 для CW, V4 для CCW = ``wp2_0`` в
      локальной системе); первая ½ внешнего витка (``wp4→wp1→wp2``)
      пропускается, первый сегмент — внешняя дуга к соседней вершине.

    Конец спирали в обоих случаях — ``wp3_{n-1}`` (нижне‑внешний угол
    самого внутреннего витка).

    If ``terminal_via_count`` is non-zero, the innermost final arc is trimmed
    and the turn count is limited so a compact terminal VIA pocket can sit
    outside the final endpoint without copper under the vias.

    ``segs_per_sweep`` сохранён для совместимости.
    """
    del segs_per_sweep

    return _sector_spiral_ray_arc(
        r_out_mm=r_out_mm,
        r_in_mm=r_in_mm,
        w_mm=w_mm,
        s_mm=s_mm,
        cx_mm=cx_mm,
        cy_mm=cy_mm,
        direction=direction,
        outward=outward,
        start_angle_deg=start_angle_deg,
        end_angle_deg=end_angle_deg,
        terminal_via_count=terminal_via_count,
        via_dia_mm=via_dia_mm,
        via_clr_mm=via_clr_mm,
        terminal_trim=terminal_trim,
        inner_via_zone=inner_via_zone,
    )

    phi_rad, th_a, th_b = _minor_sector_rays_rad(start_angle_deg, end_angle_deg)
    cx_int = _nm(cx_mm)
    cy_int = _nm(cy_mm)
    pitch = w_mm + s_mm

    def _empty() -> ArcWaypoints:
        return ArcWaypoints(
            shape="sector",
            n_turns=0,
            arcs=[],
            segments=[],
            l_wire_mm=0.0,
            l_active_mm=0.0,
            start_pt=(cx_int, cy_int),
            end_pt=(cx_int, cy_int),
        )

    if phi_rad < 1e-15 or pitch <= 0.0:
        return _empty()

    half_phi = phi_rad * 0.5
    is_cw = direction.upper() == "CW"
    theta_c = th_a + half_phi  # CCW math angle of the sector bisector
    # full_forward — стартуем во «внутренней» вершине Секции (wp4_0); иначе
    # пропускаем ½ внешнего витка и стартуем во «внешней» вершине (wp2_0).
    # Совпадает с sector_first_anchor_is_inner.
    full_forward = (is_cw == bool(outward))

    # ── KiMotor radial planner в локальной системе ──────────────────────────
    # Биссектриса вдоль +X. wp1 — верхне‑внутренний угол витка; внешний
    # виток (k=0) выровнен ровно по границе Секции (V2 в локальной системе
    # для CW лежит на луче +half_phi на радиусе r_in).

    # Per-turn local-frame data
    turns_loc: list[
        tuple[
            tuple[float, float],  # wp4 (lower inner)
            tuple[float, float],  # wp1 (upper inner)
            tuple[float, float],  # wp2 (upper outer)
            tuple[float, float],  # wp3 (lower outer)
            float,                # r_in_k
            float,                # r_out_k
            float,                # alpha_k (radial half-angle)
        ]
    ] = []

    p1 = (0.0, 0.0)
    p2 = (r_in_mm * math.cos(half_phi), r_in_mm * math.sin(half_phi))

    k = 0
    while True:
        r_in_k = r_in_mm + k * pitch
        r_out_k = r_out_mm - k * pitch
        if r_in_k + 1e-9 >= r_out_k:
            break

        if k == 0:
            # Внешний виток выровнен по границе Секции: wp1 = inner@+half_phi
            wp1 = (r_in_mm * math.cos(half_phi), r_in_mm * math.sin(half_phi))
        else:
            # Сдвигаем верхнюю радиаль предыдущего витка на -pitch
            p1o, p2o = _line_offset_xy(p1, p2, -pitch)
            wp1 = _circle_line_intersect_upper(p1o, p2o, r_in_k)
            if wp1 is None or wp1[1] <= 1e-12:
                break  # клин схлопнулся за биссектрису

        n1 = math.hypot(wp1[0], wp1[1])
        if n1 < 1e-15:
            break
        wp2 = (wp1[0] * r_out_k / n1, wp1[1] * r_out_k / n1)
        wp3 = (wp2[0], -wp2[1])
        wp4 = (wp1[0], -wp1[1])
        alpha_k = math.atan2(wp1[1], wp1[0])

        turns_loc.append((wp4, wp1, wp2, wp3, r_in_k, r_out_k, alpha_k))

        p1, p2 = wp1, wp2
        k += 1

    n_turns = len(turns_loc)
    if n_turns == 0:
        return _empty()

    if terminal_via_count > 0 and via_dia_mm > 0.0:
        via_keep = via_dia_mm * 0.5 + via_clr_mm + w_mm * 0.5
        via_step = via_dia_mm + via_clr_mm
        via_depth = 2.0 * via_keep + max(0, terminal_via_count - 1) * via_step
        while len(turns_loc) > 1:
            _wp4, _wp1, _wp2, _wp3, r_in_last, r_out_last, _alpha_last = turns_loc[-1]
            if r_in_last + via_depth <= r_out_last + 1.0e-9:
                break
            turns_loc.pop()
        n_turns = len(turns_loc)

    # ── Локальная → мировая система ────────────────────────────────────────
    cos_c = math.cos(theta_c)
    sin_c = math.sin(theta_c)
    sgn_y = 1.0 if is_cw else -1.0

    def to_world(p: tuple[float, float]) -> tuple[int, int]:
        x, y_loc = p
        y = sgn_y * y_loc
        wx = cos_c * x - sin_c * y + cx_mm
        wy = sin_c * x + cos_c * y + cy_mm
        return (_nm(wx), _nm(wy))

    def angle_world(a_loc: float) -> float:
        return theta_c + sgn_y * a_loc

    arcs: list[tuple[int, int, int, float, float]] = []
    segs: list[tuple[tuple[int, int], tuple[int, int]]] = []
    l_wire_mm = 0.0
    final_end_loc = turns_loc[-1][3]

    for k_idx, (wp4, wp1, wp2, wp3, r_in_k, r_out_k, alpha_k) in enumerate(turns_loc):
        a_inner_start = angle_world(-alpha_k)  # at wp4
        a_inner_end = angle_world(+alpha_k)    # at wp1
        a_outer_start = angle_world(+alpha_k)  # at wp2
        final_outer_end_alpha = 0.0 if terminal_via_count > 0 and k_idx == n_turns - 1 else -alpha_k
        if terminal_via_count > 0 and k_idx == n_turns - 1:
            # Stop the final arc near the pocket centre, leaving angular room
            # from both side rays for the straight active-via stub.
            wp3 = (r_out_k * math.cos(final_outer_end_alpha),
                   r_out_k * math.sin(final_outer_end_alpha))
        if k_idx == n_turns - 1:
            final_end_loc = wp3
        a_outer_end = angle_world(final_outer_end_alpha)    # at wp3 / trimmed end

        # В Outside‑режиме на k=0 пропускаем внутреннюю дугу и верхнюю
        # радиаль (wp4→wp1→wp2), стартуя сразу в wp2 = внешняя вершина
        # Секции (V3 для CW, V4 для CCW).
        skip_first_half = (k_idx == 0) and not full_forward

        if not skip_first_half:
            arcs.append((cx_int, cy_int, _nm(r_in_k), a_inner_start, a_inner_end))
            l_wire_mm += r_in_k * (2.0 * alpha_k)
            segs.append((to_world(wp1), to_world(wp2)))
            l_wire_mm += r_out_k - r_in_k

        arcs.append((cx_int, cy_int, _nm(r_out_k), a_outer_start, a_outer_end))
        l_wire_mm += r_out_k * abs(alpha_k - final_outer_end_alpha)

        if k_idx + 1 < n_turns:
            wp4_next = turns_loc[k_idx + 1][0]
            segs.append((to_world(wp3), to_world(wp4_next)))
            dx = wp4_next[0] - wp3[0]
            dy = wp4_next[1] - wp3[1]
            l_wire_mm += math.hypot(dx, dy)

    if full_forward:
        start_pt = to_world(turns_loc[0][0])      # wp4_0 = V1 (CW) / V2 (CCW)
    else:
        start_pt = to_world(turns_loc[0][2])      # wp2_0 = V3 (CW) / V4 (CCW)
    end_pt = to_world(final_end_loc)              # wp3_{n-1}, possibly trimmed

    return ArcWaypoints(
        shape="sector",
        n_turns=n_turns,
        arcs=arcs,
        segments=segs,
        l_wire_mm=l_wire_mm,
        l_active_mm=l_wire_mm,
        start_pt=start_pt,
        end_pt=end_pt,
    )
