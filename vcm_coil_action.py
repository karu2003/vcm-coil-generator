# SPDX-License-Identifier: GPL-2.0-only
# VCM Coil Generator — KiCad ActionPlugin
# Generates PCB spiral coils for VCM actuators.
# Architecture inspired by KiMotor (https://github.com/cooked/kimotor).

from __future__ import annotations

import os
import json
import math
from datetime import datetime

import wx
import pcbnew

if __name__ == "__main__":
    # Allow running standalone for quick import checks
    from vcm_coil_solver import (
        rect_spiral,
        circular_spiral,
        sector_spiral,
        minor_sector_opening_deg,
        _minor_sector_rays_rad,
        sector_via_topology_counts,
        sector_first_anchor_is_inner,
        sector_inner_stub_path,
        sector_inner_via_zone_mm,
        sector_inner_via_reserve_area_mm2,
        sector_outer_via_reserve_area_mm2,
        sector_inner_hole_min_r_in_mm,
        sector_coil_via_centres_mm,
        sector_dynamic_inner_via_zone_mm,
        sector_optimize_direct_inner_stub,
        sector_make_inner_via_zone_mm,
        sector_terminal_via_pocket_centres_mm,
    )
    from vcm_coil_gui import VCMCoilGUI
else:
    from .vcm_coil_solver import (
        rect_spiral,
        circular_spiral,
        sector_spiral,
        minor_sector_opening_deg,
        _minor_sector_rays_rad,
        sector_via_topology_counts,
        sector_first_anchor_is_inner,
        sector_inner_stub_path,
        sector_inner_via_zone_mm,
        sector_inner_via_reserve_area_mm2,
        sector_outer_via_reserve_area_mm2,
        sector_inner_hole_min_r_in_mm,
        sector_coil_via_centres_mm,
        sector_dynamic_inner_via_zone_mm,
        sector_optimize_direct_inner_stub,
        sector_make_inner_via_zone_mm,
        sector_terminal_via_pocket_centres_mm,
    )
    from .vcm_coil_gui import VCMCoilGUI


# ─── KiCad version compatibility helpers ────────────────────────────────────

def _kicad_version() -> int:
    return int(pcbnew.Version().split(".")[0])


def _make_point(ver: int):
    """Return the correct point factory for KiCad 6 vs 7+."""
    if ver < 7:
        return pcbnew.wxPoint
    return pcbnew.VECTOR2I


def _scale(ver: int) -> int:
    if ver < 7:
        return pcbnew.IU_PER_MM
    return pcbnew.FromMM(1)


# ─── ActionPlugin entry ──────────────────────────────────────────────────────

class VCMCoilPlugin(pcbnew.ActionPlugin):
    """KiCad ActionPlugin that opens the VCM Coil Generator dialog."""

    def defaults(self):
        meta_path = os.path.join(os.path.dirname(__file__), "metadata.json")
        self.version = "0.1.0"
        try:
            with open(meta_path, "r") as f:
                data = json.load(f)
            self.version = data["versions"][0]["version"]
        except Exception:
            pass

        self.name = "VCM Coil Generator"
        self.category = "Modify Drawing PCB"
        self.description = "Generate spiral PCB coil for VCM actuator"
        self.show_toolbar_button = True

        icon_path = os.path.join(os.path.dirname(__file__), "icon_24x24.png")
        if os.path.exists(icon_path):
            self.icon_file_name = icon_path

    def Run(self):
        frame = wx.FindWindowByName("PcbFrame")
        board = pcbnew.GetBoard()
        # Store reference on self — prevents Python GC from destroying the dialog
        # while it is still open (common KiCad plugin pitfall).
        self._dlg = VCMCoilDialog(frame, board)
        self._dlg.SetTitle(f"{self.name}  v{self.version}")
        self._dlg.Show()


# ─── Main dialog ─────────────────────────────────────────────────────────────

_PRESET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "vcm_coil_presets.json")

# Parameter keys written to / read from a preset (GUI ctrl name → dict key)
_PRESET_KEYS = [
    ("m_ctrlA",           "a_mm"),
    ("m_ctrlB",           "b_mm"),
    ("m_ctrlRout",        "r_out_mm"),
    ("m_ctrlRin",         "r_in_mm"),
    ("m_ctrlStartAngle",  "start_angle_deg"),
    ("m_ctrlEndAngle",    "end_angle_deg"),
    ("m_ctrlTrackW",      "w_mm"),
    ("m_ctrlGap",         "s_mm"),
    ("m_ctrlLayers",      "n_layers"),
    ("m_ctrlViaDia",      "via_dia_mm"),
    ("m_ctrlViaDrill",    "via_drill_mm"),
    ("m_ctrlViaClearance","via_clr_mm"),
    ("m_ctrlStartViaOffset", "start_via_offset_mm"),
    ("m_ctrlCX",          "cx_mm"),
    ("m_ctrlCY",          "cy_mm"),
    ("m_cbAxis",          "axis"),
    ("m_cbDirection",     "direction"),
    ("m_cbShape",         "shape"),
    ("m_chkOutline",      "outline"),
    ("m_ctrlOutlineW",    "outline_w_mm"),
]


def _load_presets() -> dict:
    if os.path.exists(_PRESET_FILE):
        try:
            with open(_PRESET_FILE, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}


def _save_presets(presets: dict) -> None:
    with open(_PRESET_FILE, "w", encoding="utf-8") as f:
        json.dump(presets, f, indent=2, ensure_ascii=False)


class VCMCoilDialog(VCMCoilGUI):
    """
    Extends the wxFormBuilder-generated base class (VCMCoilGUI) with
    business logic: reads parameters, calls the solver, writes pcbnew objects.
    """

    def __init__(self, parent, board: pcbnew.BOARD):
        super().__init__(parent)
        self.board = board
        self._kver = _kicad_version()
        self._scale = _scale(self._kver)
        self._pt = _make_point(self._kver)

        self._group: pcbnew.PCB_GROUP | None = None
        self._presets: dict = _load_presets()
        self._refresh_preset_list()

    # ── helpers ──────────────────────────────────────────────────────────────

    def _mm(self, mm: float) -> int:
        """Convert mm to KiCad internal units."""
        return int(round(mm * self._scale))

    def _pt2(self, x_nm: int, y_nm: int):
        """Wrap raw nm coordinates in the appropriate KiCad point type."""
        # vcm_coil_solver already works in nm (1 mm = 1 000 000 nm = 1e6),
        # but pcbnew.FromMM(1) == 1 000 000 in KiCad 7, so values match directly.
        if self._kver < 7:
            return pcbnew.wxPoint(x_nm, y_nm)
        return pcbnew.VECTOR2I(x_nm, y_nm)

    def _add_track(self, x0, y0, x1, y1, layer, width_nm, net):
        t = pcbnew.PCB_TRACK(self.board)
        t.SetStart(self._pt2(x0, y0))
        t.SetEnd(self._pt2(x1, y1))
        t.SetLayer(layer)
        t.SetWidth(width_nm)
        t.SetNet(net)
        self.board.Add(t)
        if self._group:
            self._group.AddItem(t)
        return t

    def _add_arc(self, cx, cy, r_nm, a_start, a_end, layer, width_nm, net):
        """
        Add a PCB_ARC given centre + radius + start/end angles (radians).
        KiCad's PCB_ARC is defined by start, end, mid points.
        """
        delta = a_end - a_start
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta <= -math.pi:
            delta += 2 * math.pi
        a_mid = a_start + delta / 2.0
        sx = cx + int(r_nm * math.cos(a_start))
        sy = cy + int(r_nm * math.sin(a_start))
        mx = cx + int(r_nm * math.cos(a_mid))
        my = cy + int(r_nm * math.sin(a_mid))
        ex = cx + int(r_nm * math.cos(a_end))
        ey = cy + int(r_nm * math.sin(a_end))

        arc = pcbnew.PCB_ARC(self.board)
        arc.SetStart(self._pt2(sx, sy))
        arc.SetMid(self._pt2(mx, my))
        arc.SetEnd(self._pt2(ex, ey))
        arc.SetLayer(layer)
        arc.SetWidth(width_nm)
        arc.SetNet(net)
        self.board.Add(arc)
        if self._group:
            self._group.AddItem(arc)
        return arc

    def _sector_inner_stub_two_seg(
        self,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        cx_nm: int,
        cy_nm: int,
        bis_ux: float,
        bis_uy: float,
        r_knee_mm: float,
        layer,
        width_nm: int,
        net,
        knee_first: bool,
    ) -> None:
        """
        Connects the inner VIA (inside the sector wedge, ≈r_mid) to
        the canonical spiral point ``wp3_{n-1}`` with a short stub.

        If the VIA lies between ``r_in`` and ``r_out`` (new scheme: zone_start
        / zone_end / overflow), a radial knee inside the central hole is
        physically incorrect — in that case a direct track is placed
        to avoid crossing spiral turns.

        The knee at radius ``r_knee_mm`` is used only when it actually
        lies between the radii of both endpoints.
        """
        if x0 == x1 and y0 == y1:
            return
        ax = x1 if knee_first else x0
        ay = y1 if knee_first else y0
        dx = ax - cx_nm
        dy = ay - cy_nm
        r_anchor = math.hypot(dx, dy)
        if r_anchor < 100:
            self._add_track(x0, y0, x1, y1, layer, width_nm, net)
            return
        dk_nm = self._mm(r_knee_mm)
        kx = cx_nm + int(round(dk_nm * dx / r_anchor))
        ky = cy_nm + int(round(dk_nm * dy / r_anchor))
        r0 = math.hypot(x0 - cx_nm, y0 - cy_nm)
        rk = math.hypot(kx - cx_nm, ky - cy_nm)
        r1 = math.hypot(x1 - cx_nm, y1 - cy_nm)
        lo, hi = (r0, r1) if r0 <= r1 else (r1, r0)
        knee_between = lo + 80 < rk < hi - 80
        if dk_nm < 100 or not knee_between:
            self._add_track(x0, y0, x1, y1, layer, width_nm, net)
            return
        self._add_track(x0, y0, kx, ky, layer, width_nm, net)
        self._add_track(kx, ky, x1, y1, layer, width_nm, net)

    def _sector_outer_stub_two_seg(
        self,
        x_anchor_nm: int,
        y_anchor_nm: int,
        x_via_nm: int,
        y_via_nm: int,
        cx_nm: int,
        cy_nm: int,
        bis_ux: float,
        bis_uy: float,
        layer,
        width_nm: int,
        net,
    ) -> None:
        """Anchor (wp3) → foot on bisector → outer via — avoids diagonal shorts."""
        if x_anchor_nm == x_via_nm and y_anchor_nm == y_via_nm:
            return
        ax_mm = (x_anchor_nm - cx_nm) / 1e6
        ay_mm = (y_anchor_nm - cy_nm) / 1e6
        t_mm = ax_mm * bis_ux + ay_mm * bis_uy
        if t_mm <= 1e-6:
            self._add_track(
                x_anchor_nm, y_anchor_nm, x_via_nm, y_via_nm,
                layer, width_nm, net,
            )
            return
        kx = cx_nm + int(round(t_mm * 1e6 * bis_ux))
        ky = cy_nm + int(round(t_mm * 1e6 * bis_uy))
        ra = math.hypot(x_anchor_nm - cx_nm, y_anchor_nm - cy_nm)
        rv = math.hypot(x_via_nm - cx_nm, y_via_nm - cy_nm)
        rk = math.hypot(kx - cx_nm, ky - cy_nm)
        lo, hi = (ra, rv) if ra <= rv else (rv, ra)
        knee_between = lo + 80 < rk < hi - 80
        if not knee_between:
            self._add_track(
                x_anchor_nm, y_anchor_nm, x_via_nm, y_via_nm,
                layer, width_nm, net,
            )
            return
        self._add_track(x_anchor_nm, y_anchor_nm, kx, ky, layer, width_nm, net)
        self._add_track(kx, ky, x_via_nm, y_via_nm, layer, width_nm, net)

    def _sector_inner_zone_stub_l(
        self,
        x_anchor_nm: int,
        y_anchor_nm: int,
        x_via_nm: int,
        y_via_nm: int,
        cx_nm: int,
        cy_nm: int,
        layer,
        width_nm: int,
        net,
        avoid_vias_nm: list[tuple[int, int]] | None = None,
        avoid_keepout_nm: int = 0,
    ) -> None:
        """Wave-style endpoint-to-via stub inside the fixed inner VIA zone.

        Direct segment when foreign vias are clear, otherwise a single bend
        on the side opposite to the offending pad so the path stays L-shaped
        and never appears to short across the via zone.
        """
        if x_anchor_nm == x_via_nm and y_anchor_nm == y_via_nm:
            return
        del cx_nm, cy_nm
        anchor_mm = (x_anchor_nm / 1e6, y_anchor_nm / 1e6)
        via_mm = (x_via_nm / 1e6, y_via_nm / 1e6)
        avoid_mm: list[tuple[float, float]] = []
        if avoid_vias_nm:
            avoid_mm = [(x / 1e6, y / 1e6) for x, y in avoid_vias_nm]
        keep_mm = max(1.0e-6, avoid_keepout_nm / 1e6)
        path = sector_inner_stub_path(
            anchor_mm, via_mm, avoid_mm, keep_mm,
        )
        for (x0, y0), (x1, y1) in zip(path, path[1:]):
            self._add_track(
                int(round(x0 * 1e6)), int(round(y0 * 1e6)),
                int(round(x1 * 1e6)), int(round(y1 * 1e6)),
                layer, width_nm, net,
            )

    def _add_via(self, x, y, drill_nm, dia_nm, net):
        via = pcbnew.PCB_VIA(self.board)
        # Internal interlayer vias are through-board vias.  Reserved vias are
        # handled by copper keepout geometry, not by hiding pads on layers.
        _vtype = getattr(pcbnew, "VIATYPE_T_THROUGH",
                         getattr(pcbnew, "VIATYPE_THROUGH", None))
        if _vtype is not None:
            via.SetViaType(_vtype)
        via.SetPosition(self._pt2(x, y))
        via.SetDrill(drill_nm)
        via.SetWidth(dia_nm)
        via.SetNet(net)
        self.board.Add(via)
        if self._group:
            self._group.AddItem(via)
        return via

    def _ensure_net(self, name: str) -> pcbnew.NETINFO_ITEM:
        net = self.board.FindNet(name)
        if net is None or net.GetNetCode() == 0:
            net = pcbnew.NETINFO_ITEM(self.board, name)
            self.board.Add(net)
        return net

    # ── layer set builder ────────────────────────────────────────────────────

    @staticmethod
    def _layer_set(n_layers: int) -> list:
        """Return ordered list of copper layers for the coil winding."""
        inner_map = [
            pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.In3_Cu, pcbnew.In4_Cu,
            pcbnew.In5_Cu, pcbnew.In6_Cu, pcbnew.In7_Cu, pcbnew.In8_Cu,
        ]
        layers = [pcbnew.F_Cu]
        inner_needed = n_layers - 2
        for i in range(min(inner_needed, len(inner_map))):
            layers.append(inner_map[i])
        if n_layers >= 2:
            layers.append(pcbnew.B_Cu)
        return layers

    # ── parameter reader ─────────────────────────────────────────────────────

    def _get_params(self) -> dict | None:
        """Read all GUI values; return dict or None on validation error.

        Shape is resolved via get_derived_shape():
          - catalog magnet selected → shape from magnet (disc→circle, block→rect)
          - Custom              → shape from m_cbShape combobox
        """
        try:
            p: dict = {}
            # Shape is always derived from magnet or explicit override
            p["shape"]      = self.get_derived_shape()              # "rect" / "circle" / "sector"
            p["a_mm"]       = float(self.m_ctrlA.GetValue())        # outer dim A (mm)
            p["b_mm"]       = float(self.m_ctrlB.GetValue())        # outer dim B (mm)
            p["r_out_mm"]   = float(self.m_ctrlRout.GetValue())     # outer radius (mm)
            p["r_in_mm"]    = float(self.m_ctrlRin.GetValue())      # inner keep-out (mm)
            # sector_deg is no longer a separate input; derived from angle span
            p["start_angle_deg"] = float(self.m_ctrlStartAngle.GetValue())
            p["end_angle_deg"]   = float(self.m_ctrlEndAngle.GetValue())
            # Derived sector angle (used for silkscreen and solver)
            raw_span = p["end_angle_deg"] - p["start_angle_deg"]
            p["sector_deg"] = raw_span % 360.0 or 360.0  # 0 → full circle
            p["w_mm"]       = float(self.m_ctrlTrackW.GetValue())   # track width
            p["s_mm"]       = float(self.m_ctrlGap.GetValue())      # track gap
            p["n_layers"]   = int(self.m_ctrlLayers.GetValue())     # copper layers
            p["via_dia_mm"] = float(self.m_ctrlViaDia.GetValue())   # via diameter
            p["via_drill_mm"]    = float(self.m_ctrlViaDrill.GetValue())     # via drill
            p["via_clr_mm"]    = float(self.m_ctrlViaClearance.GetValue())  # via-to-coil clearance
            p["start_via_offset_mm"] = float(self.m_ctrlStartViaOffset.GetValue())
            p["outline"]    = self.m_chkOutline.GetValue()           # draw silkscreen outline
            p["outline_w_mm"] = float(self.m_ctrlOutlineW.GetValue())  # outline line width
            p["cx_mm"]      = float(self.m_ctrlCX.GetValue())       # coil centre X
            p["cy_mm"]      = float(self.m_ctrlCY.GetValue())       # coil centre Y
            p["axis"]       = self.m_cbAxis.GetStringSelection()    # "X" / "Y"
            p["direction"]  = self.m_cbDirection.GetStringSelection()  # "CW" / "CCW"
            # True = inner→outer (starts at inner radius)
            p["outward"]    = (self.m_cbStart.GetStringSelection()
                               == "Inside \u2192 Outside")
        except ValueError as e:
            wx.LogError(f"VCM Coil: invalid parameter — {e}")
            return None

        # Validate minimums
        if p["w_mm"] <= 0 or p["s_mm"] <= 0:
            wx.LogError("VCM Coil: track width and gap must be > 0")
            return None
        if p["start_via_offset_mm"] < 0:
            wx.LogError("VCM Coil: Start VIA offset must be >= 0")
            return None

        return p

    # ── main generator ───────────────────────────────────────────────────────

    def generate(self):
        p = self._get_params()
        if p is None:
            return

        # Remove stale generator groups from previous dialog sessions so old
        # copper never overlays new routing (seen as "dangling" L3 traces).
        self._clear_stale_vcm_groups()
        # Keep only one generated coil group per dialog session.  Without this,
        # repeated Generate clicks stack old copper on top of new geometry and
        # can look like stray/unconnected remnants on upper layers.
        self._clear_current_group()

        # Store p and clearance as instance attrs so draw helpers can access them
        self._p = p
        self._via_clr_mm = p["via_clr_mm"]

        # Create a named group to allow easy undo/redo
        group_name = f"vcm_coil_{p['axis']}_{datetime.now().strftime('%H%M%S')}"
        self._group = pcbnew.PCB_GROUP(self.board)
        self._group.SetName(group_name)
        self.board.Add(self._group)

        net = self._ensure_net(f"coil_{p['axis'].lower()}")
        lset = self._layer_set(p["n_layers"])
        w_nm  = self._mm(p["w_mm"])
        vd_nm = self._mm(p["via_dia_mm"])
        vr_nm = self._mm(p["via_drill_mm"])

        # ── Solve geometry ──────────────────────────────────────────────────
        shape = p["shape"].lower()
        if shape == "rect":
            coil = self._draw_rect_coil(None, lset, w_nm, vd_nm, vr_nm, net,
                                        start_outward=p["outward"])

        elif shape == "circle":
            coil = self._generate_circle_coil(p, lset, w_nm, vd_nm, vr_nm, net)

        else:  # sector
            coil = self._generate_sector_coil(p, lset, w_nm, vd_nm, vr_nm, net)

        # ── Silkscreen outline ──────────────────────────────────────────────
        if p.get("outline"):
            self._draw_outline(p)

        # ── Update stats labels ─────────────────────────────────────────────
        self.m_lblTurns.SetLabel(str(coil.n_turns))
        self.m_lblLwire.SetLabel(f"{coil.l_wire_mm * p['n_layers']:.1f}")
        self.m_lblLactive.SetLabel(f"{coil.l_active_mm * p['n_layers']:.1f}")

        pcbnew.Refresh()
        self.btn_clear.Enable(True)

    def _clear_current_group(self) -> None:
        if not self._group:
            return
        items = []
        self._group.RunOnChildren(lambda item: items.append(item))
        for item in items:
            self.board.Remove(item)
        self.board.Remove(self._group)
        self._group = None
        self.btn_clear.Enable(False)

    def _clear_stale_vcm_groups(self) -> None:
        """Remove groups from previous plugin runs (even from old dialogs)."""
        groups: list = []
        if hasattr(self.board, "Groups"):
            try:
                groups = list(self.board.Groups())
            except Exception:
                groups = []
        elif hasattr(self.board, "GetGroups"):
            try:
                groups = list(self.board.GetGroups())
            except Exception:
                groups = []

        for grp in groups:
            try:
                name = grp.GetName()
            except Exception:
                continue
            if not isinstance(name, str) or not name.startswith("vcm_coil_"):
                continue
            items = []
            try:
                grp.RunOnChildren(lambda item: items.append(item))
            except Exception:
                items = []
            for item in items:
                try:
                    self.board.Remove(item)
                except Exception:
                    pass
            try:
                self.board.Remove(grp)
            except Exception:
                pass

        # The active handle may now point to a deleted group.
        self._group = None
        self.btn_clear.Enable(False)

    # ── drawing helpers ──────────────────────────────────────────────────────

    def _add_stub_90(self, x0, y0, x1, y1, layer, w_nm, net,
                      horiz_first: bool = True):
        """L-shaped 90° Manhattan stub between two points.

        horiz_first=True:  (x0,y0) → (x1,y0) → (x1,y1)
        horiz_first=False: (x0,y0) → (x0,y1) → (x1,y1)

        Falls back to a single segment when the points share an axis.
        """
        if x0 == x1 and y0 == y1:
            return
        if x0 == x1 or y0 == y1:
            self._add_track(x0, y0, x1, y1, layer, w_nm, net)
            return
        if horiz_first:
            self._add_track(x0, y0, x1, y0, layer, w_nm, net)
            self._add_track(x1, y0, x1, y1, layer, w_nm, net)
        else:
            self._add_track(x0, y0, x0, y1, layer, w_nm, net)
            self._add_track(x0, y1, x1, y1, layer, w_nm, net)

    # Inner VIA offsets (at corners, toward centre, with L-stubs).
    _INNER_VIA = [
        (+1, +1, +1,  0),   # SW: base up-right, spread right
        (-1, +1, -1,  0),   # SE: base up-left, spread left
        (-1, -1, -1,  0),   # NE: base down-left, spread left
        (+1, -1, +1,  0),   # NW: base down-right, spread right
    ]

    # Outer VIA at corner (for intermediate outer VIAs, L-stubs).
    _OUTER_VIA_CORNER = [
        (-1, -1, +1,  0),   # SW: base outward, spread right
        (+1, -1, -1,  0),   # SE: base outward, spread left
        (+1, +1, -1,  0),   # NE: base outward, spread left
        (-1, +1, +1,  0),   # NW: base outward, spread right
    ]

    # Outer VIA at edge midpoint (first/last VIA only, straight connection).
    # edge 0=right, 1=top, 2=left, 3=bottom.
    _EDGE_PERP   = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    # Spread along edge, away from the associated corner:
    _EDGE_SPREAD = [(0, 1), (-1, 0), (0, -1), (1, 0)]

    @staticmethod
    def _corner_to_edge(corner: int) -> int:
        """Leaving edge from *corner* in CCW order.
        SE→right(0), NE→top(1), NW→left(2), SW→bottom(3)."""
        return (corner + 3) % 4

    @staticmethod
    def _edge_to_corner(edge: int) -> int:
        """Corner that *edge* leaves from in CCW.
        right→SE(1), top→NE(2), left→NW(3), bottom→SW(0)."""
        return (edge + 1) % 4

    def _draw_rect_coil(self, _coil_unused, lset, w_nm, vd_nm, vr_nm, net,
                         start_outward: bool = False):
        """
        Rectangular N-layer coil.

        Outer VIAs sit at edge midpoints (0°=mid-right, 90°=mid-top …)
        and connect to the spiral with a straight perpendicular track.
        Inner VIAs stay at corners with short L-stubs.
        """
        n_layers     = len(lset)
        n_vias_total = n_layers + 1

        vd_mm  = vd_nm / 1e6
        clr_mm = self._via_clr_mm
        w_mm   = self._p["w_mm"]
        s_mm   = self._p["s_mm"]
        a_mm   = self._p["a_mm"]
        b_mm   = self._p["b_mm"]
        cx     = self._p["cx_mm"]
        cy     = self._p["cy_mm"]
        d      = w_mm / 2.0 + clr_mm + vd_mm / 2.0
        spacing = vd_mm + clr_mm

        is_cw = self._p.get("direction", "CCW").upper() == "CW"
        cy_nm = self._mm(cy)

        def _my(y_nm: int) -> int:
            """Mirror Y around centre if CW to flip winding direction."""
            return (2 * cy_nm - y_nm) if is_cw else y_nm

        # ── Angle → edge → corner ────────────────────────────────────────────
        sa = self._p.get("start_angle_deg", 0.0)
        ea = self._p.get("end_angle_deg", sa + 360.0)
        start_edge = int(round(sa / 90.0)) % 4
        end_edge   = int(round(ea / 90.0)) % 4
        sc0 = self._edge_to_corner(start_edge)
        ec  = self._edge_to_corner(end_edge)

        via_corners: list[int] = [(sc0 + i * 3) % 4 for i in range(n_vias_total)]

        last_layer_sc = via_corners[-2]
        # end_sides so the half-side lands on end_edge:
        # last side's edge = (last_sc + es + 2) % 4 = end_edge
        es = (end_edge - last_layer_sc - 2) % 4
        if es == 0:
            es = 4
        via_corners[-1] = (last_layer_sc + es) % 4

        via_is_outer: list[bool] = [
            (i % 2 == 0) != start_outward for i in range(n_vias_total)]
        last_outward = start_outward ^ (n_layers % 2 == 0)
        suppress_end_mid = (es == 4 and last_outward)

        # ── Inner void ────────────────────────────────────────────────────────
        inner_corner_counts = [0] * 4
        for i, is_o in enumerate(via_is_outer):
            if not is_o:
                inner_corner_counts[via_corners[i]] += 1
        max_per_corner = max(inner_corner_counts) if any(not o for o in via_is_outer) else 0

        if max_per_corner > 0:
            void_dim = 2.0 * (d + (max_per_corner - 1) * spacing)
        else:
            void_dim = 0.0
        min_room = 2.0 * (w_mm + s_mm)
        a_in_mm = max(0.0, min(void_dim, a_mm - min_room))
        b_in_mm = max(0.0, min(void_dim, b_mm - min_room))

        # ── Reference spiral for boundary dimensions ──────────────────────────
        ref = rect_spiral(a_mm, b_mm, w_mm, s_mm, cx_mm=cx, cy_mm=cy,
                          outward=False, a_in_mm=a_in_mm, b_in_mm=b_in_mm,
                          start_corner=0)
        outer_sw = (ref.start_pt[0] / 1e6, ref.start_pt[1] / 1e6)
        inner_nw = (ref.end_pt[0] / 1e6, ref.end_pt[1] / 1e6)

        xa_outer = cx - outer_sw[0]
        xb_outer = cy - outer_sw[1]
        xa_inner = cx - inner_nw[0]
        xb_inner = inner_nw[1] - cy

        def _rc(xa, xb):
            return [(cx - xa, cy - xb), (cx + xa, cy - xb),
                    (cx + xa, cy + xb), (cx - xa, cy + xb)]

        inner_corners = _rc(xa_inner, xb_inner)

        # Edge midpoints on the outer boundary
        _out_mid = [
            (cx + xa_outer, cy),          # 0 right
            (cx,            cy + xb_outer),  # 1 top
            (cx - xa_outer, cy),          # 2 left
            (cx,            cy - xb_outer),  # 3 bottom
        ]

        # ── Via positions ─────────────────────────────────────────────────────
        outer_corners = _rc(xa_outer, xb_outer)
        outer_edge_idx    = [0] * 4
        outer_corner_idx  = [0] * 4
        inner_corner_idx  = [0] * 4

        via_positions: list[tuple[float, float]] = []
        via_edges: list[int] = []
        via_is_mid_edge: list[bool] = []

        for i in range(n_vias_total):
            corn = via_corners[i]
            is_terminal = (i == 0 or i == n_vias_total - 1)
            mid_edge = via_is_outer[i] and is_terminal

            via_is_mid_edge.append(mid_edge)

            if mid_edge:
                if i == n_vias_total - 1:
                    edge = (corn + 2) % 4   # arriving edge (end VIA)
                else:
                    edge = self._corner_to_edge(corn)  # leaving edge (start VIA)
                via_edges.append(edge)
                j = outer_edge_idx[edge]
                outer_edge_idx[edge] += 1
                px, py = self._EDGE_PERP[edge]
                sx, sy = self._EDGE_SPREAD[edge]
                mx, my = _out_mid[edge]
                via_positions.append((
                    mx + px * d + sx * j * spacing,
                    my + py * d + sy * j * spacing,
                ))
            elif via_is_outer[i]:
                via_edges.append(-1)
                base = outer_corners[corn]
                bx, by, ssx, ssy = self._OUTER_VIA_CORNER[corn]
                j = outer_corner_idx[corn]
                outer_corner_idx[corn] += 1
                via_positions.append((
                    base[0] + bx * d + ssx * j * spacing,
                    base[1] + by * d + ssy * j * spacing,
                ))
            else:
                via_edges.append(-1)
                base = inner_corners[corn]
                bx, by, ssx, ssy = self._INNER_VIA[corn]
                j = inner_corner_idx[corn]
                inner_corner_idx[corn] += 1
                via_positions.append((
                    base[0] + bx * d + ssx * j * spacing,
                    base[1] + by * d + ssy * j * spacing,
                ))

        # ── Place all vias ────────────────────────────────────────────────────
        for vx_mm, vy_mm in via_positions:
            self._add_via(self._mm(vx_mm), _my(self._mm(vy_mm)), vr_nm, vd_nm, net)

        # ── Per-layer spirals ─────────────────────────────────────────────────
        _horiz = [True, False, True, False]
        first_coil = None

        for idx, layer in enumerate(lset):
            layer_outward = start_outward ^ (idx % 2 == 1)
            layer_sc = via_corners[idx]
            is_last  = (idx == n_layers - 1)
            if is_last and suppress_end_mid:
                layer_es = 3
            elif is_last:
                layer_es = es
            else:
                layer_es = 3

            is_start_mid = via_is_mid_edge[idx]
            is_end_mid   = via_is_mid_edge[idx + 1]
            eff_end_mid  = is_end_mid and not (is_last and suppress_end_mid)

            layer_coil = rect_spiral(
                a_mm=a_mm, b_mm=b_mm,
                w_mm=w_mm, s_mm=s_mm,
                cx_mm=cx, cy_mm=cy,
                outward=layer_outward,
                a_in_mm=a_in_mm,
                b_in_mm=b_in_mm,
                start_corner=layer_sc,
                end_sides=layer_es,
                start_mid_edge=is_start_mid,
                end_mid_edge=eff_end_mid,
            )
            if first_coil is None:
                first_coil = layer_coil
            if not layer_coil.segments:
                continue

            spx, spy = layer_coil.start_pt[0], _my(layer_coil.start_pt[1])
            epx, epy = layer_coil.end_pt[0],   _my(layer_coil.end_pt[1])
            vx0 = self._mm(via_positions[idx][0])
            vy0 = _my(self._mm(via_positions[idx][1]))
            vx1 = self._mm(via_positions[idx + 1][0])
            vy1 = _my(self._mm(via_positions[idx + 1][1]))

            # ── Start connection ──────────────────────────────────────────
            if is_start_mid:
                edge = via_edges[idx]
                px, py = self._EDGE_PERP[edge]
                bpx = via_positions[idx][0] - px * d
                bpy = via_positions[idx][1] - py * d
                bpx_nm, bpy_nm = self._mm(bpx), _my(self._mm(bpy))
                self._add_track(vx0, vy0, bpx_nm, bpy_nm, layer, w_nm, net)
                if bpx_nm != spx or bpy_nm != spy:
                    self._add_track(bpx_nm, bpy_nm, spx, spy, layer, w_nm, net)
            else:
                first_side = layer_sc % 4
                self._add_stub_90(vx0, vy0, spx, spy, layer, w_nm, net,
                                  horiz_first=_horiz[first_side])

            # ── Spiral tracks ─────────────────────────────────────────────
            for (x0, y0), (x1, y1) in layer_coil.segments:
                self._add_track(x0, _my(y0), x1, _my(y1), layer, w_nm, net)

            # ── End connection ────────────────────────────────────────────
            if eff_end_mid:
                edge = via_edges[idx + 1]
                px, py = self._EDGE_PERP[edge]
                bpx = via_positions[idx + 1][0] - px * d
                bpy = via_positions[idx + 1][1] - py * d
                bpx_nm, bpy_nm = self._mm(bpx), _my(self._mm(bpy))
                if bpx_nm != epx or bpy_nm != epy:
                    self._add_track(epx, epy, bpx_nm, bpy_nm, layer, w_nm, net)
                self._add_track(bpx_nm, bpy_nm, vx1, vy1, layer, w_nm, net)
            elif is_last and suppress_end_mid:
                arrival_side = (layer_sc + 3) % 4
                self._add_stub_90(epx, epy, vx1, vy1, layer, w_nm, net,
                                  horiz_first=not _horiz[arrival_side])
            else:
                last_side = (layer_sc + max(0, layer_es - 1)) % 4
                self._add_stub_90(epx, epy, vx1, vy1, layer, w_nm, net,
                                  horiz_first=not _horiz[last_side])

        return first_coil

    def _generate_circle_coil(self, p, lset, w_nm, vd_nm, vr_nm, net):
        """
        Build N-layer circular spiral coil with correct series connection.

        All layers wind in the same rotational direction (CW or CCW) so
        magnetic fields ADD UP.

        Topology (N layers, N+1 through-hole vias):

          Via[0] ──L0──> Via[1] ──L1──> Via[2] ── ... ──L(N-1)──> Via[N]

        Via[0]   = coil start connection pad
        Via[N]   = coil end   connection pad
        Via[1..N-1] = interlayer transition vias

        Via pad clearance rule
        ─────────────────────
        Via pad diameter > track width.  Placing a via ON the spiral
        centerline would overlap the neighbour track.  Instead:

          outer via:  r_via_outer = r_out + s + vd/2  (beyond coil OD)
          inner via:  r_via_inner = r_in  - s - vd/2  (inside coil ID)

        A short radial stub on the same layer bridges gap between the
        via centre and the spiral endpoint at track-centre radius.

        Via radius per via index i (0 … N):
          Layer i  (between via[i] and via[i+1]):
            even → outward (inner→outer): starts at inner, ends at outer
            odd  → inward  (outer→inner): starts at outer, ends at inner
          So via[i] is the START of layer i  →  radius = start radius of layer i
          via[N] is the END of the last layer.
        """
        n_layers      = len(lset)
        n_vias_total  = n_layers + 1       # includes start and end pads

        vd_mm = vd_nm / 1e6

        # Safe via-centre radii outside the active winding zone
        clr_mm = p["via_clr_mm"]
        r_via_outer = p["r_out_mm"] + clr_mm + vd_mm / 2.0
        r_via_inner = max(vd_mm / 2.0,
                          p["r_in_mm"] - clr_mm - vd_mm / 2.0)

        # ── Compute via positions ────────────────────────────────────────
        # Evenly spaced angles; unique XY for every via.
        # p["outward"]=False / GUI «Outside→Inside»: layer 0 winds inward → via at r_out+
        # p["outward"]=True  / GUI «Inside→Outside»: layer 0 winds outward → via at r_in−
        p_outward = p.get("outward", False)
        via_info: list[tuple[float, float]] = []   # (r_via_mm, angle_deg)
        for i in range(n_vias_total):
            # Distribute vias linearly from start_angle to end_angle.
            # If start == end (mod 360), distribute evenly over full circle.
            sa = p.get("start_angle_deg", 0.0)
            ea = p.get("end_angle_deg", sa + 360.0)
            delta = (ea - sa) % 360.0 or 360.0  # 0 → full 360°
            angle_deg = sa + i * delta / n_layers
            if i < n_layers:
                # via[i] = start of layer i
                # layer i is outward when: (p_outward XOR (i is odd))
                layer_is_outward = p_outward ^ (i % 2 == 1)
                if layer_is_outward:    # outward: starts at inner
                    r_via = r_via_inner
                else:                   # inward:  starts at outer
                    r_via = r_via_outer
            else:
                # via[N] = end of last layer (layer N-1)
                last_is_outward = p_outward ^ ((n_layers - 1) % 2 == 1)
                if last_is_outward:     # outward: ends at outer
                    r_via = r_via_outer
                else:                   # inward:  ends at inner
                    r_via = r_via_inner
            via_info.append((r_via, angle_deg))

        # ── Place all vias ───────────────────────────────────────────────
        for r_via, a_deg in via_info:
            a_rad = math.radians(a_deg)
            vx_nm = self._mm(p["cx_mm"] + r_via * math.cos(a_rad))
            vy_nm = self._mm(p["cy_mm"] + r_via * math.sin(a_rad))
            self._add_via(vx_nm, vy_nm, vr_nm, vd_nm, net)

        # ── Generate spirals + connector stubs ───────────────────────────
        first_coil = None
        for idx, layer in enumerate(lset):
            # Respect user "Start from" setting for layer 0 direction
            outward = p_outward ^ (idx % 2 == 1)
            start_ang = via_info[idx][1]
            end_ang   = via_info[idx + 1][1]

            layer_coil = circular_spiral(
                r_out_mm=p["r_out_mm"], r_in_mm=p["r_in_mm"],
                w_mm=p["w_mm"], s_mm=p["s_mm"],
                cx_mm=p["cx_mm"], cy_mm=p["cy_mm"],
                direction=p["direction"],
                outward=outward,
                start_angle_deg=start_ang,
                end_angle_deg=end_ang,
            )
            if first_coil is None:
                first_coil = layer_coil

            # Stub: start via → spiral start
            r_sv, a_sv = via_info[idx]
            a_rad = math.radians(a_sv)
            vx = self._mm(p["cx_mm"] + r_sv * math.cos(a_rad))
            vy = self._mm(p["cy_mm"] + r_sv * math.sin(a_rad))
            sx, sy = layer_coil.start_pt
            self._add_track(vx, vy, sx, sy, layer, w_nm, net)

            # Spiral body
            for (x0, y0), (x1, y1) in layer_coil.segments:
                self._add_track(x0, y0, x1, y1, layer, w_nm, net)

            # Stub: spiral end → end via
            r_ev, a_ev = via_info[idx + 1]
            a_rad = math.radians(a_ev)
            vx = self._mm(p["cx_mm"] + r_ev * math.cos(a_rad))
            vy = self._mm(p["cy_mm"] + r_ev * math.sin(a_rad))
            ex, ey = layer_coil.end_pt
            self._add_track(ex, ey, vx, vy, layer, w_nm, net)

        return first_coil

    def _draw_circle_coil(self, coil_outward, coil_inward, lset, w_nm, vd_nm, vr_nm, net):
        """Legacy stub — no longer called."""
        pass

    # ── sector coil (N-layer, N+1 via topology) ──────────────────────────────

    def _generate_sector_coil(self, p, lset, w_nm, vd_nm, vr_nm, net):
        """
        Многослойная секторная катушка с правильным последовательным
        соединением слоёв.

        Геометрия спирали (см. ``sector_spiral``):

        * Внешний виток выровнен ровно по границе Секции **V1‑V2‑V3‑V4**
          (V1=inner@θ_a, V2=inner@θ_b, V3=outer@θ_b, V4=outer@θ_a).
        * Старт спирали — в одной из вершин V1…V4 согласно
          ``sector_first_via_vertex_id(direction, outward)``:

          - CW + Inside  → V1 (первый сегмент V1→V2 по внутренней дуге),
          - CW + Outside → V3 (первый сегмент V3→V4 по внешней дуге),
          - CCW + Inside  → V4 (V4→V3 по внешней дуге),
          - CCW + Outside → V2 (V2→V1 по внутренней дуге).
        * Конец спирали — ``wp3_{n-1}`` (нижне‑внешний угол самого
          вложенного витка).

        VIA‑топология
        --------------
        * **VIA[0]** — точно в стартовой вершине Секции (V1/V2/V3/V4).
          Первый трек спирали идёт от вершины к соседней вершине **без**
          диагонального стаба.
        * **VIA вида ``inner``** (промежуточные) — теперь **внутри**
          Секции, на координатах ``zone_start`` / ``zone_end`` (45°-зона
          от вершин V1/V4 и V2/V3 соответственно), переполнение шагом к
          центру катушки от середины зоны.
        * **VIA вида ``outer``** — стек на биссектрисе **за** ``r_out``,
          с лёгким разводом по углу.

        Соединение спирали с внутренним VIA (``zone_start``/``zone_end``)
        делается прямым треком: VIA лежит уже в плоскости клина, и
        радиальное «колено» в центральной дыре больше не требуется.
        Соединение с outer VIA — через колено на биссектрисе (как и
        раньше), чтобы избежать диагональных коротышей через витки.
        """
        n_layers = len(lset)
        if n_layers == 0:
            return None

        vd_mm  = vd_nm / 1e6
        clr_mm = p["via_clr_mm"]

        p_outward = p.get("outward", False)
        cx_mm = p["cx_mm"]
        cy_mm = p["cy_mm"]
        sa = p["start_angle_deg"]
        ea = p["end_angle_deg"]
        r_in_mm = p["r_in_mm"]
        r_out_mm = p["r_out_mm"]
        pitch_mm = p["w_mm"] + p["s_mm"]
        w_mm = p["w_mm"]

        cx_nm = self._mm(cx_mm)
        cy_nm = self._mm(cy_mm)

        def _fy(y_nm: int) -> int:
            """Mirror Y around the coil centre so KiCad (Y-down) renders the
            sector with the same visual orientation as the math-frame
            algorithm (V1..V4 placement matches spiral_sector.py)."""
            return 2 * cy_nm - y_nm

        # ── Via topology **before** spiral / tracks (series winding order) ──
        topo_i, topo_o = sector_via_topology_counts(
            n_layers, p_outward, p["direction"],
        )
        first_is_inner = sector_first_anchor_is_inner(p["direction"], p_outward)
        # VIA[0] sits at the chosen sector vertex (inner-radius for «Inside»,
        # outer-radius for «Outside»). Every other series VIA alternates
        # zone/outer-rail by parity, so VIA[1] *always* lands in the inner
        # zone — that is the natural geometric end of layer 0's spiral and the
        # routing target the user expects to see on the first layer.
        anchor_kind: list[str] = []
        for i in range(n_layers + 1):
            if i == 0:
                anchor_kind.append("inner" if first_is_inner else "outer")
            else:
                anchor_kind.append("inner" if (i % 2 == 1) else "outer")
        n_inner = anchor_kind.count("inner")
        n_outer = anchor_kind.count("outer")

        # Sector bisector (minor arc midpoint).
        _phi, ta_rad, tb_rad = _minor_sector_rays_rad(sa, ea)
        delta = tb_rad - ta_rad
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        theta_c = ta_rad + 0.5 * delta
        bis_ux = math.cos(theta_c)
        # Bisector is Y-mirrored together with the rest of the sector copper so
        # that stub-knee maths stay consistent with the placed geometry.
        bis_uy = -math.sin(theta_c)

        step_mm = vd_mm + clr_mm
        pad_clear_mm = vd_mm / 2.0 + clr_mm
        nudge_mm = 1.0e-3
        # Радиус «колена» на биссектрисе для стабов outer‑VIA — оставлен
        # на случай переполнения inner‑серии (когда позиции уходят за
        # дугу r_in внутрь центральной дыры).
        r_stub_knee_mm = max(
            vd_mm * 0.5 + clr_mm + nudge_mm,
            r_in_mm - w_mm * 0.5 - clr_mm - nudge_mm,
        )

        terminal_inner_indices = [
            i for i, kind in enumerate(anchor_kind)
            if kind == "inner" and i != 0
        ]

        # Build initial via positions (VIA[0] vertex + outer fallback).  Inner
        # interlayer vias are replaced below with the optimized terminal pocket
        # after the spiral endpoint is known.
        centres_mm = sector_coil_via_centres_mm(
            anchor_kind,
            cx_mm=cx_mm,
            cy_mm=cy_mm,
            r_in_mm=r_in_mm,
            r_out_mm=r_out_mm,
            start_angle_deg=sa,
            end_angle_deg=ea,
            vd_mm=vd_mm,
            clr_mm=clr_mm,
            w_mm=w_mm,
            direction=p["direction"],
            layer0_outward=p_outward,
        )
        # Move VIA[0] farther away from copper on other layers:
        # - inner start: push towards centre (into the hole),
        # - outer start: push outside the winding.
        # This keeps the start terminal via while reducing interlayer touch risk.
        if n_layers > 1 and centres_mm:
            vx0, vy0 = centres_mm[0]
            dx0 = vx0 - cx_mm
            dy0 = vy0 - cy_mm
            r0 = math.hypot(dx0, dy0)
            if r0 > 1.0e-12:
                unit_x = dx0 / r0
                unit_y = dy0 / r0
                start_via_shift_mm = max(
                    0.0,
                    p.get("start_via_offset_mm", 0.80),
                )
                radial_sign = -1.0 if anchor_kind[0] == "inner" else 1.0
                r1 = max(0.0, r0 + radial_sign * start_via_shift_mm)
                centres_mm[0] = (
                    cx_mm + unit_x * r1,
                    cy_mm + unit_y * r1,
                )

        def _layer_direction(layer_idx: int) -> str:
            """Alternate spiral handedness per layer for additive Lorentz force.

            After layer 0 the direction follows the real series start terminal:
            layers starting from an outer VIA use the base handedness, while
            layers starting from an inner VIA use the mirror and are drawn in
            reverse.  This keeps radial current direction additive without
            forcing an outer-start layer to begin at an inner sector vertex.
            """
            if layer_idx == 0 or anchor_kind[layer_idx] == "outer":
                return p["direction"]
            return "CCW" if p["direction"].strip().upper() == "CW" else "CW"

        def _layer_outward(layer_idx: int) -> bool:
            # Layer 0 preserves the user-selected start side.  Every following
            # series layer is generated outer→inner; inner-start layers are
            # drawn backwards below, so their actual current path is inner→outer.
            return layer_idx == 0 and first_is_inner

        def _build_layer_coils(inner_zone):
            coils = []
            for layer_idx in range(n_layers):
                touches_inner_zone = (
                    layer_idx in terminal_inner_indices
                    or (layer_idx + 1) in terminal_inner_indices
                )
                coils.append(sector_spiral(
                    r_out_mm=r_out_mm, r_in_mm=r_in_mm,
                    w_mm=p["w_mm"], s_mm=p["s_mm"],
                    cx_mm=cx_mm, cy_mm=cy_mm,
                    direction=_layer_direction(layer_idx),
                    outward=_layer_outward(layer_idx),
                    start_angle_deg=sa,
                    end_angle_deg=ea,
                    terminal_via_count=len(terminal_inner_indices) if inner_zone else 0,
                    via_dia_mm=vd_mm,
                    via_clr_mm=clr_mm,
                    terminal_trim=touches_inner_zone,
                    inner_via_zone=inner_zone,
                ))
            return coils

        # ── Fixed inner VIA zone: route-and-measure first-layer pocket ─────
        inner_zone = None
        layer_coils = _build_layer_coils(None)
        if terminal_inner_indices:
            inner_zone = sector_dynamic_inner_via_zone_mm(
                cx_mm=cx_mm,
                cy_mm=cy_mm,
                r_in_mm=r_in_mm,
                r_out_mm=r_out_mm,
                w_mm=p["w_mm"],
                s_mm=p["s_mm"],
                start_angle_deg=sa,
                end_angle_deg=ea,
                inner_via_indices=terminal_inner_indices,
                n_layers=n_layers,
                vd_mm=vd_mm,
                clr_mm=clr_mm,
                side=-1.0 if p["direction"].strip().upper() == "CW" else 1.0,
            )
            if inner_zone is not None:
                layer_coils = _build_layer_coils(inner_zone)
                # Inner-zone VIA assignment is geometric, not series-positional.
                # Each terminal_inner_indices entry is a peer pocket position;
                # we map series VIA indices to whichever free pocket sits
                # closest to the actual spiral endpoint(s) that touch this VIA.
                pocket_centres = list(inner_zone.centres_mm)
                free_pocket_idx = list(range(len(pocket_centres)))

                def _layer_zone_targets_mm(via_idx: int) -> list[tuple[float, float]]:
                    # Route this through-via pocket for BOTH adjacent layers:
                    # previous layer ends here, next layer starts here.
                    # Using only one side can pick a pocket that forces the
                    # other side to draw a long "extra" segment inside the zone.
                    targets: list[tuple[float, float]] = []
                    end_layer = via_idx - 1
                    if 0 <= end_layer < n_layers:
                        ex, ey = layer_coils[end_layer].end_pt
                        targets.append((ex / 1e6, ey / 1e6))
                    start_layer = via_idx
                    if 0 <= start_layer < n_layers and start_layer in terminal_inner_indices:
                        sx, sy = layer_coils[start_layer].end_pt
                        targets.append((sx / 1e6, sy / 1e6))
                    return targets

                for via_idx in terminal_inner_indices:
                    if not free_pocket_idx:
                        break
                    targets = _layer_zone_targets_mm(via_idx)
                    if targets:
                        cx_t = sum(t[0] for t in targets) / len(targets)
                        cy_t = sum(t[1] for t in targets) / len(targets)
                    else:
                        cx_t, cy_t = pocket_centres[free_pocket_idx[0]]
                    best = min(
                        free_pocket_idx,
                        key=lambda p: (
                            (pocket_centres[p][0] - cx_t) ** 2
                            + (pocket_centres[p][1] - cy_t) ** 2
                        ),
                    )
                    free_pocket_idx.remove(best)
                    centres_mm[via_idx] = pocket_centres[best]

                keep_mm = vd_mm * 0.5 + clr_mm + w_mm * 0.5
                # Global routing optimization: run several relaxation passes so
                # endpoint tweaks on one layer are re-evaluated against all
                # active inner-zone targets on neighbouring layers.
                for _pass in range(4):
                    changed = False
                    for layer_idx, layer_coil in enumerate(layer_coils):
                        inner_targets = []
                        if layer_idx in terminal_inner_indices:
                            inner_targets.append(layer_idx)
                        if (layer_idx + 1) in terminal_inner_indices:
                            inner_targets.append(layer_idx + 1)
                        for via_idx in inner_targets:
                            optimized = sector_optimize_direct_inner_stub(
                                layer_coils[layer_idx],
                                via_xy_mm=centres_mm[via_idx],
                                avoid_vias_mm=[
                                    centres_mm[other_idx]
                                    for other_idx in terminal_inner_indices
                                    if other_idx != via_idx
                                ],
                                keep_mm=keep_mm,
                                cx_mm=cx_mm,
                                cy_mm=cy_mm,
                            )
                            if optimized.end_pt != layer_coils[layer_idx].end_pt:
                                changed = True
                            layer_coils[layer_idx] = optimized
                    if not changed:
                        break

        if not layer_coils[0].arcs and not layer_coils[0].segments:
            return layer_coils[0]

        via_xy_nm = [
            (self._mm(vx_mm), _fy(self._mm(vy_mm))) for vx_mm, vy_mm in centres_mm
        ]
        inner_zone_keepout_nm = self._mm(vd_mm * 0.5 + clr_mm + w_mm * 0.5)

        def _inactive_inner_vias_nm(active_idx: int) -> list[tuple[int, int]]:
            return [
                via_xy_nm[other_idx]
                for other_idx in terminal_inner_indices
                if other_idx != active_idx
            ]

        r_hole_required_mm = sector_inner_hole_min_r_in_mm(
            n_inner, vd_mm, clr_mm, w_mm,
        )
        if n_inner > 0 and r_in_mm + 1e-6 < r_hole_required_mm:
            try:
                wx.LogWarning(
                    "VCM Coil: центральная область inner-VIA слишком мала — "
                    f"r_in={r_in_mm:.3f} mm < {r_hole_required_mm:.3f} mm "
                    f"для {n_inner} inner VIA в области r<r_in."
                )
            except Exception:
                pass

        if topo_i != n_inner or topo_o != n_outer:
            try:
                wx.LogWarning(
                    "VCM Coil: internal via topology inconsistency "
                    f"{topo_i}+{topo_o} vs {n_inner}+{n_outer}."
                )
            except Exception:
                pass

        area_inner_mm2 = sector_inner_via_reserve_area_mm2(n_inner, vd_mm, clr_mm)
        area_outer_mm2 = sector_outer_via_reserve_area_mm2(n_outer, vd_mm, clr_mm)
        inner_strip_mm = max(0, n_inner - 1) * step_mm + 2.0 * pad_clear_mm
        outer_strip_mm = max(0, n_outer - 1) * step_mm + 2.0 * pad_clear_mm
        try:
            sm = "Outside→Inside" if not p_outward else "Inside→Outside"
            # Avoid intrusive Info popup after Generate; keep generation silent.
            _msg = (
                f"VCM Coil: {n_layers} слоёв → {n_layers + 1} VIA "
                f"({n_inner} внутри спирали, r<r_in + {n_outer} за r_out); "
                f"старт {sm}. VIA[0] — в вершине Секции. "
                f"Внутри спирали ≈ {area_inner_mm2:.2f} mm² "
                f"(полоса ≈{inner_strip_mm:.2f} mm); "
                f"за r_out ≈ {area_outer_mm2:.2f} mm² "
                f"(полоса ≥{outer_strip_mm:.2f} mm за r_out={r_out_mm:.2f} mm)."
            )
            del _msg
        except Exception:
            pass

        # ── Inner VIAs: terminal pocket / start vertex before track copper ──
        for i, (vx, vy) in enumerate(via_xy_nm):
            # VIA[0] is the user-visible start terminal of the spiral.
            if anchor_kind[i] == "inner":
                self._add_via(vx, vy, vr_nm, vd_nm, net)

        # ── Outer VIAs (outside r_out) **before** stubs/arcs/segments ──
        for i, (vx, vy) in enumerate(via_xy_nm):
            if anchor_kind[i] == "outer":
                self._add_via(vx, vy, vr_nm, vd_nm, net)

        # ── Copper: arcs + segments + соединительные стабы ─────────────
        # VIA[0] теперь точно совпадает со стартом спирали слоя 0
        # (см. sector_coil_via_centres_mm) — стартового стаба нет.
        # Между слоями: спираль заканчивается в wp3_{n-1}; VIA в плоскости
        # клина (zone_start / zone_end) или за r_out.
        first_coil = None
        for idx, layer in enumerate(lset):
            layer_coil = layer_coils[idx]
            if first_coil is None:
                first_coil = layer_coil
            if not layer_coil.arcs and not layer_coil.segments:
                continue

            starts_on_inner_zone = idx in terminal_inner_indices
            ends_on_inner_zone = (idx + 1) in terminal_inner_indices

            if starts_on_inner_zone:
                # This layer starts at a through inner VIA.  Draw the generated
                # spiral backwards so the only inner-zone copper is the short
                # active-VIA connection; do not route from the zone to the
                # outer boundary with a long start stub.
                vx0, vy0 = via_xy_nm[idx]
                zx, zy = layer_coil.end_pt[0], _fy(layer_coil.end_pt[1])
                if (vx0, vy0) != (zx, zy):
                    self._sector_inner_zone_stub_l(
                        zx, zy, vx0, vy0, cx_nm, cy_nm, layer, w_nm, net,
                        avoid_vias_nm=_inactive_inner_vias_nm(idx),
                        avoid_keepout_nm=inner_zone_keepout_nm,
                    )

                for cx0, cy0, r_nm, a0, a1 in reversed(layer_coil.arcs):
                    self._add_arc(cx0, _fy(cy0), r_nm, -a1, -a0, layer, w_nm, net)

                for (x0, y0), (x1, y1) in reversed(layer_coil.segments):
                    self._add_track(x1, _fy(y1), x0, _fy(y0), layer, w_nm, net)

                vx1, vy1 = via_xy_nm[idx + 1]
                ex, ey = layer_coil.start_pt[0], _fy(layer_coil.start_pt[1])
                if (ex, ey) != (vx1, vy1):
                    if anchor_kind[idx + 1] == "outer":
                        self._sector_outer_stub_two_seg(
                            ex, ey, vx1, vy1, cx_nm, cy_nm,
                            bis_ux, bis_uy, layer, w_nm, net,
                        )
                    else:
                        self._sector_inner_stub_two_seg(
                            ex, ey, vx1, vy1, cx_nm, cy_nm,
                            bis_ux, bis_uy, r_stub_knee_mm, layer, w_nm, net,
                            knee_first=False,
                        )
                continue

            # ── Стартовый стаб VIA[idx] → start_pt спирали ─────────
            vx0, vy0 = via_xy_nm[idx]
            sx, sy = layer_coil.start_pt[0], _fy(layer_coil.start_pt[1])
            if (vx0, vy0) != (sx, sy):
                if anchor_kind[idx] == "inner":
                    self._sector_inner_stub_two_seg(
                        vx0, vy0, sx, sy, cx_nm, cy_nm,
                        bis_ux, bis_uy, r_stub_knee_mm, layer, w_nm, net,
                        knee_first=True,
                    )
                else:
                    self._sector_outer_stub_two_seg(
                        sx, sy, vx0, vy0, cx_nm, cy_nm,
                        bis_ux, bis_uy, layer, w_nm, net,
                    )

            for cx0, cy0, r_nm, a0, a1 in layer_coil.arcs:
                # Y-mirror around coil centre: arc centre stays at cy_nm, the
                # angles flip sign because (cos t, -sin t) = (cos -t, sin -t).
                self._add_arc(cx0, _fy(cy0), r_nm, -a0, -a1, layer, w_nm, net)

            for (x0, y0), (x1, y1) in layer_coil.segments:
                self._add_track(x0, _fy(y0), x1, _fy(y1), layer, w_nm, net)

            # ── Финальный стаб end_pt → VIA[idx+1] ─────────────────
            vx1, vy1 = via_xy_nm[idx + 1]
            ex, ey = layer_coil.end_pt[0], _fy(layer_coil.end_pt[1])
            if (ex, ey) != (vx1, vy1):
                if ends_on_inner_zone:
                    self._sector_inner_zone_stub_l(
                        ex, ey, vx1, vy1, cx_nm, cy_nm, layer, w_nm, net,
                        avoid_vias_nm=_inactive_inner_vias_nm(idx + 1),
                        avoid_keepout_nm=inner_zone_keepout_nm,
                    )
                elif anchor_kind[idx + 1] == "inner":
                    self._sector_inner_stub_two_seg(
                        ex, ey, vx1, vy1, cx_nm, cy_nm,
                        bis_ux, bis_uy, r_stub_knee_mm, layer, w_nm, net,
                        knee_first=False,
                    )
                else:
                    self._sector_outer_stub_two_seg(
                        ex, ey, vx1, vy1, cx_nm, cy_nm,
                        bis_ux, bis_uy, layer, w_nm, net,
                    )

        return first_coil

    # ── silkscreen outline ────────────────────────────────────────────────────

    def _silk_shape(self, lw_nm: int) -> pcbnew.PCB_SHAPE:
        """Create a PCB_SHAPE on F_SilkS with the given line width."""
        s = pcbnew.PCB_SHAPE(self.board)
        s.SetLayer(pcbnew.F_SilkS)
        s.SetWidth(lw_nm)
        return s

    def _draw_outline(self, p: dict):
        """Draw a silkscreen outline matching the coil boundary."""
        lw_nm = self._mm(p["outline_w_mm"])
        shape = p["shape"].lower()
        cx_nm = self._mm(p["cx_mm"])
        cy_nm = self._mm(p["cy_mm"])

        def _add(s):
            self.board.Add(s)
            if self._group:
                self._group.AddItem(s)

        if shape == "circle":
            for r_mm in (p["r_out_mm"], p["r_in_mm"]):
                if r_mm <= 0:
                    continue
                s = self._silk_shape(lw_nm)
                _SHAPE_CIRCLE = getattr(pcbnew, "SHAPE_T_CIRCLE",
                                        getattr(pcbnew, "S_CIRCLE", None))
                if _SHAPE_CIRCLE is not None:
                    s.SetShape(_SHAPE_CIRCLE)
                s.SetCenter(self._pt2(cx_nm, cy_nm))
                # SetEnd sets a point on the circle (radius = distance centre→end)
                s.SetEnd(self._pt2(cx_nm + self._mm(r_mm), cy_nm))
                _add(s)

        elif shape == "rect":
            hw = self._mm(p["a_mm"] / 2.0)
            hh = self._mm(p["b_mm"] / 2.0)
            corners = [
                (cx_nm - hw, cy_nm - hh),
                (cx_nm + hw, cy_nm - hh),
                (cx_nm + hw, cy_nm + hh),
                (cx_nm - hw, cy_nm + hh),
            ]
            _SHAPE_SEG = getattr(pcbnew, "SHAPE_T_SEGMENT",
                                  getattr(pcbnew, "S_SEGMENT", None))
            for i in range(4):
                x0, y0 = corners[i]
                x1, y1 = corners[(i + 1) % 4]
                s = self._silk_shape(lw_nm)
                if _SHAPE_SEG is not None:
                    s.SetShape(_SHAPE_SEG)
                s.SetStart(self._pt2(x0, y0))
                s.SetEnd(self._pt2(x1, y1))
                _add(s)

        elif shape == "sector":
            sa_d = p.get("start_angle_deg", 0.0)
            ea_d = p.get("end_angle_deg", 180.0)
            sector_deg_val = minor_sector_opening_deg(sa_d, ea_d)
            if sector_deg_val < 1e-6:
                sector_deg_val = 360.0
            dccw = (ea_d % 360.0 - sa_d % 360.0) % 360.0
            if dccw <= 180.0:
                a_arc0 = math.radians(sa_d)
            else:
                a_arc0 = math.radians(ea_d)
            _SHAPE_ARC = getattr(pcbnew, "SHAPE_T_ARC",
                                  getattr(pcbnew, "S_ARC", None))
            _SHAPE_SEG = getattr(pcbnew, "SHAPE_T_SEGMENT",
                                  getattr(pcbnew, "S_SEGMENT", None))
            for r_mm in (p["r_out_mm"], p["r_in_mm"]):
                if r_mm <= 0:
                    continue
                r_nm = self._mm(r_mm)
                s = self._silk_shape(lw_nm)
                if _SHAPE_ARC is not None:
                    s.SetShape(_SHAPE_ARC)
                s.SetCenter(self._pt2(cx_nm, cy_nm))
                s.SetStart(self._pt2(cx_nm + int(r_nm * math.cos(a_arc0)),
                                     cy_nm + int(r_nm * math.sin(a_arc0))))
                s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(sector_deg_val,
                                                     pcbnew.DEGREES_T), False)
                _add(s)
            # Radial lines connecting inner and outer arcs
            a_start = math.radians(sa_d)
            a_end = math.radians(ea_d)
            for a in (a_start, a_end):
                x0 = cx_nm + int(self._mm(p["r_in_mm"]) * math.cos(a))
                y0 = cy_nm + int(self._mm(p["r_in_mm"]) * math.sin(a))
                x1 = cx_nm + int(self._mm(p["r_out_mm"]) * math.cos(a))
                y1 = cy_nm + int(self._mm(p["r_out_mm"]) * math.sin(a))
                s = self._silk_shape(lw_nm)
                if _SHAPE_SEG is not None:
                    s.SetShape(_SHAPE_SEG)
                s.SetStart(self._pt2(x0, y0))
                s.SetEnd(self._pt2(x1, y1))
                _add(s)

    def _draw_arc_coil(self, coil, lset, w_nm, vd_nm, vr_nm, net,
                        start_outward: bool = False):
        """Legacy stub — replaced by _generate_sector_coil."""
        pass

    # ── event handlers ───────────────────────────────────────────────────────

    def on_btn_generate(self, event):
        try:
            self.generate()
        except Exception as e:
            wx.LogError(f"VCM Coil Generator error:\n{e}")
            import traceback
            wx.LogError(traceback.format_exc())
            return
        event.Skip()

    def on_btn_clear(self, event):
        self._clear_current_group()
        pcbnew.Refresh()
        event.Skip()

    def on_close(self, event):
        self.Destroy()

    # ── preset helpers ────────────────────────────────────────────────────────

    def _refresh_preset_list(self):
        """Rebuild the preset ComboBox from the current presets dict."""
        names = sorted(self._presets.keys(), key=str.lower)
        self.m_cbPreset.Set(names)

    def _read_all_fields(self) -> dict:
        """Read every parameter field into a dict for serialisation."""
        data = {}
        for ctrl_name, key in _PRESET_KEYS:
            ctrl = getattr(self, ctrl_name, None)
            if ctrl is None:
                continue
            if isinstance(ctrl, wx.SpinCtrl):
                data[key] = ctrl.GetValue()
            elif isinstance(ctrl, wx.CheckBox):
                data[key] = ctrl.GetValue()
            elif isinstance(ctrl, wx.ComboBox):
                data[key] = ctrl.GetStringSelection()
            else:
                data[key] = ctrl.GetValue()
        return data

    def _apply_preset(self, data: dict):
        """Write a preset dict back into all parameter fields."""
        for ctrl_name, key in _PRESET_KEYS:
            if key not in data:
                continue
            ctrl = getattr(self, ctrl_name, None)
            if ctrl is None:
                continue
            val = data[key]
            if isinstance(ctrl, wx.SpinCtrl):
                ctrl.SetValue(int(val))
            elif isinstance(ctrl, wx.CheckBox):
                ctrl.SetValue(bool(val))
            elif isinstance(ctrl, wx.ComboBox):
                ctrl.SetStringSelection(str(val))
            else:
                ctrl.SetValue(str(val))
        # Refresh panel visibility after loading
        self.on_magnet_selected(None)

    def on_preset_save(self, event):
        name = self.m_cbPreset.GetValue().strip()
        if not name:
            wx.MessageBox("Введите имя пресета.", "Сохранение пресета",
                          wx.OK | wx.ICON_WARNING, self)
            return
        self._presets[name] = self._read_all_fields()
        _save_presets(self._presets)
        self._refresh_preset_list()
        self.m_cbPreset.SetStringSelection(name)

    def on_preset_load(self, event):
        name = self.m_cbPreset.GetValue().strip()
        if name not in self._presets:
            wx.MessageBox(f"Пресет «{name}» не найден.", "Загрузка пресета",
                          wx.OK | wx.ICON_WARNING, self)
            return
        self._apply_preset(self._presets[name])

    def on_preset_delete(self, event):
        name = self.m_cbPreset.GetValue().strip()
        if name not in self._presets:
            wx.MessageBox(f"Пресет «{name}» не найден.", "Удаление пресета",
                          wx.OK | wx.ICON_WARNING, self)
            return
        if wx.MessageBox(f"Удалить пресет «{name}»?", "Подтверждение",
                         wx.YES_NO | wx.ICON_QUESTION, self) != wx.YES:
            return
        del self._presets[name]
        _save_presets(self._presets)
        self._refresh_preset_list()
        self.m_cbPreset.SetValue("")
