# SPDX-License-Identifier: GPL-2.0-only
# VCM Coil Generator — wxPython dialog (hand-coded, no wxFormBuilder)
# Provides the same role as kimotor_gui.py in KiMotor.
#
# Coil shape is derived from the magnet:
#   disc  → circle spiral
#   block → rect   spiral
# When "Custom" is selected the shape combobox becomes editable.

from __future__ import annotations

import json
import os
from pathlib import Path
import wx


# ─── Magnet catalog loader ────────────────────────────────────────────────────

_CATALOG_SEARCH = [
    # dev layout: plugin sits inside the project folder
    Path(__file__).parent / ".." / "magnet_catalog.json",
    # bundled copy (for KiCad plugin install)
    Path(__file__).parent / "magnet_catalog.json",
]


def _load_catalog() -> list[dict]:
    """Return filtered list of NdFeB disc/block magnets suitable for VCM."""
    for p in _CATALOG_SEARCH:
        p = p.resolve()
        if p.exists():
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
            result = []
            for m in data.get("magnets", []):
                if m.get("material") != "NdFeB":
                    continue
                shape = m.get("shape", "")
                if shape not in ("disc", "block"):
                    continue
                if m.get("magnetization_direction") == "diametral":
                    continue
                br = m.get("br_typ_T", 0)
                h  = m.get("height_mm", 0)
                if br <= 0 or h <= 0:
                    continue
                if shape == "disc":
                    d = m.get("diameter_mm", 0)
                    if d < 5:
                        continue
                    label = (f"{m['sku']}  \u00d8{d:.0f}\u00d7{h:.0f}mm  "
                             f"{m.get('grade','')}  Br={br:.2f}T")
                    result.append(dict(
                        label=label, mag_shape="disc", coil_shape="circle",
                        br=br, h_mm=h, a_mm=d, b_mm=d,
                        r_out_mm=d / 2, r_in_mm=0.0,
                    ))
                else:
                    l = m.get("length_mm", 0)
                    w = m.get("width_mm", 0)
                    if min(l, w) < 5:
                        continue
                    label = (f"{m['sku']}  {l:.0f}\u00d7{w:.0f}\u00d7{h:.0f}mm  "
                             f"{m.get('grade','')}  Br={br:.2f}T")
                    result.append(dict(
                        label=label, mag_shape="block", coil_shape="rect",
                        br=br, h_mm=h, a_mm=l, b_mm=w,
                        r_out_mm=max(l, w) / 2, r_in_mm=0.0,
                    ))
            result.sort(key=lambda x: -(x["a_mm"] * x["b_mm"]))
            return result
    return []


# ─────────────────────────────────────────────────────────────────────────────

class VCMCoilGUI(wx.Dialog):
    """
    Base dialog for the VCM Coil Generator plugin.

    Top:  Magnet selector from catalog  →  coil shape derived automatically
          disc  → circle spiral
          block → rect   spiral
          Custom → manual shape + dims

    Layout:
      [Magnet]  →  [Coil dimensions]  |  [Results]
      [Track rules]  [Layers/via]  [Position]
      [Generate]  [Clear]  [Close]
    """

    _CUSTOM_LABEL = "— Custom (manual) —"

    def __init__(self, parent):
        super().__init__(
            parent,
            id=wx.ID_ANY,
            title="VCM Coil Generator",
            pos=wx.DefaultPosition,
            size=wx.Size(560, 660),
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER,
        )
        self.SetSizeHints(wx.DefaultSize, wx.DefaultSize)

        # Load magnet catalog once at dialog creation
        self._catalog: list[dict] = _load_catalog()

        root_sizer = wx.BoxSizer(wx.VERTICAL)

        # ── 1. Magnet selector ──────────────────────────────────────────────
        gb_mag = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Magnet (from catalog)"), wx.VERTICAL)

        mag_labels = [self._CUSTOM_LABEL] + [m["label"] for m in self._catalog]
        self.m_cbMagnet = wx.ComboBox(
            self, choices=mag_labels, style=wx.CB_READONLY)
        self.m_cbMagnet.SetSelection(0)
        gb_mag.Add(self.m_cbMagnet, 0, wx.EXPAND | wx.ALL, 4)

        # Magnet info line: shape + Br
        info_sizer = wx.BoxSizer(wx.HORIZONTAL)
        info_sizer.Add(wx.StaticText(self, label="Shape:"), 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.m_lblDerivedShape = wx.StaticText(self, label="—")
        bold = self.m_lblDerivedShape.GetFont()
        bold.SetWeight(wx.FONTWEIGHT_BOLD)
        self.m_lblDerivedShape.SetFont(bold)
        info_sizer.Add(self.m_lblDerivedShape, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 16)

        info_sizer.Add(wx.StaticText(self, label="Br:"), 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.m_lblBr = wx.StaticText(self, label="—")
        info_sizer.Add(self.m_lblBr, 0, wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 16)

        info_sizer.Add(wx.StaticText(self, label="Size:"), 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.m_lblMagSize = wx.StaticText(self, label="—")
        info_sizer.Add(self.m_lblMagSize, 0, wx.ALIGN_CENTER_VERTICAL)

        gb_mag.Add(info_sizer, 0, wx.EXPAND | wx.LEFT | wx.BOTTOM, 4)
        root_sizer.Add(gb_mag, 0, wx.EXPAND | wx.ALL, 4)

        # ── 2. Axis / Direction / Start-from / manual shape override ───────
        top_row = wx.BoxSizer(wx.HORIZONTAL)

        gb_axis = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Axis"), wx.HORIZONTAL)
        self.m_cbAxis = wx.ComboBox(
            self, choices=["X", "Y"], style=wx.CB_READONLY)
        self.m_cbAxis.SetSelection(0)
        gb_axis.Add(self.m_cbAxis, 0, wx.ALL, 4)
        top_row.Add(gb_axis, 0, wx.EXPAND | wx.RIGHT, 6)

        gb_dir = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Direction"), wx.HORIZONTAL)
        self.m_cbDirection = wx.ComboBox(
            self, choices=["CW", "CCW"], style=wx.CB_READONLY)
        self.m_cbDirection.SetSelection(0)
        gb_dir.Add(self.m_cbDirection, 0, wx.ALL, 4)
        top_row.Add(gb_dir, 0, wx.EXPAND | wx.RIGHT, 6)

        gb_start = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Start from"), wx.HORIZONTAL)
        self.m_cbStart = wx.ComboBox(
            self, choices=["Outside \u2192 Inside", "Inside \u2192 Outside"],
            style=wx.CB_READONLY)
        self.m_cbStart.SetSelection(0)  # default: outside → inside
        self.m_cbStart.SetToolTip(
            "Sector coil: Outside→Inside → series starts at the inner anchor "
            "(VIA in central hole, r<r_in). Inside→Outside → series starts at "
            "the outer anchor (first VIA beyond r_out). Matches KiMotor wp4/wp3."
        )
        gb_start.Add(self.m_cbStart, 0, wx.ALL, 4)
        top_row.Add(gb_start, 0, wx.EXPAND | wx.RIGHT, 6)

        gb_shape = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Shape (Custom only)"),
            wx.HORIZONTAL)
        self.m_cbShape = wx.ComboBox(
            self, choices=["rect", "circle", "sector"], style=wx.CB_READONLY)
        self.m_cbShape.SetSelection(0)
        gb_shape.Add(self.m_cbShape, 0, wx.ALL, 4)
        top_row.Add(gb_shape, 0, wx.EXPAND)

        root_sizer.Add(top_row, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 4)

        # ── 3. Two-column: dims left, results right ─────────────────────────
        col_sizer = wx.BoxSizer(wx.HORIZONTAL)
        left = wx.BoxSizer(wx.VERTICAL)

        # --- Rect dims panel ---
        self.gb_rect = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Rect coil dimensions (mm)"), wx.VERTICAL)
        fg2 = wx.FlexGridSizer(2, 2, 4, 8)
        fg2.AddGrowableCol(1)
        fg2.Add(wx.StaticText(self, label="Width A (X):"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlA = wx.TextCtrl(self, value="15.0")
        fg2.Add(self.m_ctrlA, 1, wx.EXPAND)
        fg2.Add(wx.StaticText(self, label="Height B (Y):"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlB = wx.TextCtrl(self, value="10.0")
        fg2.Add(self.m_ctrlB, 1, wx.EXPAND)
        self.gb_rect.Add(fg2, 1, wx.EXPAND | wx.ALL, 4)
        left.Add(self.gb_rect, 0, wx.EXPAND | wx.ALL, 4)

        # --- Circle / sector dims panel ---
        self.gb_circ = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Circle / sector dimensions (mm)"),
            wx.VERTICAL)
        fg3 = wx.FlexGridSizer(2, 2, 4, 8)
        fg3.AddGrowableCol(1)
        fg3.Add(wx.StaticText(self, label="R outer:"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlRout = wx.TextCtrl(self, value="25.0")
        fg3.Add(self.m_ctrlRout, 1, wx.EXPAND)
        fg3.Add(wx.StaticText(self, label="R inner:"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlRin = wx.TextCtrl(self, value="15.0")
        fg3.Add(self.m_ctrlRin, 1, wx.EXPAND)
        self.gb_circ.Add(fg3, 1, wx.EXPAND | wx.ALL, 4)
        left.Add(self.gb_circ, 0, wx.EXPAND | wx.ALL, 4)

        # --- Connection angles (shared by rect + circle/sector) ---
        gb_angles = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Connection angles (°)"), wx.VERTICAL)
        fg_ang = wx.FlexGridSizer(2, 2, 4, 8)
        fg_ang.AddGrowableCol(1)
        self.lbl_start_angle = wx.StaticText(self, label="Start angle:")
        fg_ang.Add(self.lbl_start_angle, 0, wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlStartAngle = wx.TextCtrl(self, value="15.0")
        fg_ang.Add(self.m_ctrlStartAngle, 1, wx.EXPAND)
        self.lbl_end_angle = wx.StaticText(self, label="End angle:")
        fg_ang.Add(self.lbl_end_angle, 0, wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlEndAngle = wx.TextCtrl(self, value="345.0")
        fg_ang.Add(self.m_ctrlEndAngle, 1, wx.EXPAND)
        gb_angles.Add(fg_ang, 1, wx.EXPAND | wx.ALL, 4)
        left.Add(gb_angles, 0, wx.EXPAND | wx.ALL, 4)

        # --- Track rules ---
        gb_rules = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Track rules (mm)"), wx.VERTICAL)
        fg4 = wx.FlexGridSizer(2, 2, 4, 8)
        fg4.AddGrowableCol(1)
        fg4.Add(wx.StaticText(self, label="Track width:"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlTrackW = wx.TextCtrl(self, value="0.2")
        fg4.Add(self.m_ctrlTrackW, 1, wx.EXPAND)
        fg4.Add(wx.StaticText(self, label="Gap:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlGap = wx.TextCtrl(self, value="0.2")
        fg4.Add(self.m_ctrlGap, 1, wx.EXPAND)
        gb_rules.Add(fg4, 1, wx.EXPAND | wx.ALL, 4)
        left.Add(gb_rules, 0, wx.EXPAND | wx.ALL, 4)

        # --- Layer stack & via ---
        gb_layers = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Layer stack & via"), wx.VERTICAL)
        fg5 = wx.FlexGridSizer(5, 2, 4, 8)
        fg5.AddGrowableCol(1)
        fg5.Add(wx.StaticText(self, label="Layers:"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlLayers = wx.SpinCtrl(self, value="4", min=1, max=16)
        fg5.Add(self.m_ctrlLayers, 1, wx.EXPAND)
        fg5.Add(wx.StaticText(self, label="Via dia (mm):"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlViaDia = wx.TextCtrl(self, value="0.5")
        fg5.Add(self.m_ctrlViaDia, 1, wx.EXPAND)
        fg5.Add(wx.StaticText(self, label="Via drill (mm):"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlViaDrill = wx.TextCtrl(self, value="0.3")
        fg5.Add(self.m_ctrlViaDrill, 1, wx.EXPAND)
        fg5.Add(wx.StaticText(self, label="Via clearance (mm):"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlViaClearance = wx.TextCtrl(self, value="0.2")
        fg5.Add(self.m_ctrlViaClearance, 1, wx.EXPAND)
        fg5.Add(wx.StaticText(self, label="Start VIA offset (mm):"), 0,
                wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlStartViaOffset = wx.TextCtrl(self, value="0.8")
        self.m_ctrlStartViaOffset.SetToolTip(
            "Radial shift of VIA[0] away from the winding to avoid interlayer "
            "touches at the start terminal."
        )
        fg5.Add(self.m_ctrlStartViaOffset, 1, wx.EXPAND)
        gb_layers.Add(fg5, 1, wx.EXPAND | wx.ALL, 4)
        left.Add(gb_layers, 0, wx.EXPAND | wx.ALL, 4)

        # --- Coil centre ---
        gb_pos = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Coil centre (mm, board origin)"),
            wx.VERTICAL)
        fg6 = wx.FlexGridSizer(2, 2, 4, 8)
        fg6.AddGrowableCol(1)
        fg6.Add(wx.StaticText(self, label="CX:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlCX = wx.TextCtrl(self, value="150.0")
        fg6.Add(self.m_ctrlCX, 1, wx.EXPAND)
        fg6.Add(wx.StaticText(self, label="CY:"), 0, wx.ALIGN_CENTER_VERTICAL)
        self.m_ctrlCY = wx.TextCtrl(self, value="100.0")
        fg6.Add(self.m_ctrlCY, 1, wx.EXPAND)
        gb_pos.Add(fg6, 1, wx.EXPAND | wx.ALL, 4)
        left.Add(gb_pos, 0, wx.EXPAND | wx.ALL, 4)

        col_sizer.Add(left, 1, wx.EXPAND)

        # ── Right: results ──────────────────────────────────────────────────
        right = wx.BoxSizer(wx.VERTICAL)
        gb_stats = wx.StaticBoxSizer(
            wx.StaticBox(self, label="Results"), wx.VERTICAL)
        fg_s = wx.FlexGridSizer(3, 2, 6, 8)
        fg_s.AddGrowableCol(1)

        fg_s.Add(wx.StaticText(self, label="Turns:"), 0,
                 wx.ALIGN_CENTER_VERTICAL)
        self.m_lblTurns = wx.StaticText(self, label="—")
        fg_s.Add(self.m_lblTurns)

        fg_s.Add(wx.StaticText(self, label="Wire (mm):"), 0,
                 wx.ALIGN_CENTER_VERTICAL)
        self.m_lblLwire = wx.StaticText(self, label="—")
        fg_s.Add(self.m_lblLwire)

        fg_s.Add(wx.StaticText(self, label="Active (mm):"), 0,
                 wx.ALIGN_CENTER_VERTICAL)
        self.m_lblLactive = wx.StaticText(self, label="—")
        fg_s.Add(self.m_lblLactive)

        gb_stats.Add(fg_s, 1, wx.EXPAND | wx.ALL, 4)
        right.Add(gb_stats, 0, wx.EXPAND | wx.ALL, 4)

        col_sizer.Add(right, 0, wx.EXPAND)
        root_sizer.Add(col_sizer, 1, wx.EXPAND | wx.ALL, 4)

        # ── Silkscreen outline row ───────────────────────────────────────────
        silk_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.m_chkOutline = wx.CheckBox(self, label="Silkscreen outline")
        self.m_chkOutline.SetValue(True)
        silk_sizer.Add(self.m_chkOutline, 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 8)
        silk_sizer.Add(wx.StaticText(self, label="Width (mm):"), 0,
                       wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.m_ctrlOutlineW = wx.TextCtrl(self, value="0.12", size=(60, -1))
        silk_sizer.Add(self.m_ctrlOutlineW, 0, wx.ALIGN_CENTER_VERTICAL)
        root_sizer.Add(silk_sizer, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 8)

        # ── Preset bar ──────────────────────────────────────────────────────
        preset_sizer = wx.BoxSizer(wx.HORIZONTAL)
        preset_sizer.Add(wx.StaticText(self, label="Preset:"), 0,
                         wx.ALIGN_CENTER_VERTICAL | wx.RIGHT, 4)
        self.m_cbPreset = wx.ComboBox(self, style=wx.CB_DROPDOWN)
        preset_sizer.Add(self.m_cbPreset, 1, wx.EXPAND | wx.RIGHT, 4)
        self.btn_save_preset   = wx.Button(self, label="Save",   size=(50, -1))
        self.btn_load_preset   = wx.Button(self, label="Load",   size=(50, -1))
        self.btn_delete_preset = wx.Button(self, label="Delete", size=(55, -1))
        preset_sizer.Add(self.btn_save_preset,   0, wx.RIGHT, 4)
        preset_sizer.Add(self.btn_load_preset,   0, wx.RIGHT, 4)
        preset_sizer.Add(self.btn_delete_preset, 0)
        root_sizer.Add(preset_sizer, 0,
                       wx.EXPAND | wx.LEFT | wx.RIGHT | wx.BOTTOM, 4)

        # ── Bottom buttons ──────────────────────────────────────────────────
        btn_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.btn_generate = wx.Button(self, label="Generate")
        self.btn_clear    = wx.Button(self, label="Clear")
        self.btn_close    = wx.Button(self, id=wx.ID_CLOSE, label="Close")
        self.btn_clear.Enable(False)

        btn_sizer.Add(self.btn_generate, 0, wx.ALL, 4)
        btn_sizer.Add(self.btn_clear,    0, wx.ALL, 4)
        btn_sizer.AddStretchSpacer()
        btn_sizer.Add(self.btn_close,    0, wx.ALL, 4)
        root_sizer.Add(btn_sizer, 0, wx.EXPAND | wx.ALL, 4)

        self.SetSizer(root_sizer)
        self.Layout()

        # ── Bind events ─────────────────────────────────────────────────────
        self.m_cbMagnet.Bind(wx.EVT_COMBOBOX, self.on_magnet_selected)
        self.m_cbShape.Bind(wx.EVT_COMBOBOX, self.on_cb_shape)
        self.btn_generate.Bind(wx.EVT_BUTTON, self.on_btn_generate)
        self.btn_clear.Bind(wx.EVT_BUTTON, self.on_btn_clear)
        self.btn_close.Bind(wx.EVT_BUTTON, self.on_close)
        self.btn_save_preset.Bind(wx.EVT_BUTTON, self.on_preset_save)
        self.btn_load_preset.Bind(wx.EVT_BUTTON, self.on_preset_load)
        self.btn_delete_preset.Bind(wx.EVT_BUTTON, self.on_preset_delete)
        self.Bind(wx.EVT_CLOSE, self.on_close)

        # Initial state: Custom selected
        self.on_magnet_selected(None)

    # ── Derived shape accessor ────────────────────────────────────────────────

    def get_derived_shape(self) -> str:
        """
        Return the effective coil shape string ('rect', 'circle', 'sector').
        When a catalog magnet is selected the shape is derived from the magnet.
        In Custom mode it reads the manual shape combobox.
        """
        idx = self.m_cbMagnet.GetSelection()
        if idx > 0:
            return self._catalog[idx - 1]["coil_shape"]
        return self.m_cbShape.GetStringSelection()

    # ── Panel visibility helper ───────────────────────────────────────────────

    def _update_dim_panels(self, shape: str):
        """Show the dimension panel that matches the current shape.

        gb_rect / gb_circ are StaticBoxSizer objects; use ShowItems() to
        show/hide all child widgets inside them, then explicitly show/hide
        the static box itself.
        """
        is_rect   = shape == "rect"
        is_arc    = shape in ("circle", "sector")

        # StaticBoxSizer.ShowItems(bool) shows/hides every item in the sizer
        self.gb_rect.ShowItems(is_rect)
        self.gb_rect.GetStaticBox().Show(is_rect)

        self.gb_circ.ShowItems(is_arc)
        self.gb_circ.GetStaticBox().Show(is_arc)

        self.Layout()
        self.Fit()


    # ── Stub handlers (overridden in VCMCoilDialog) ──────────────────────────

    def on_magnet_selected(self, event):
        """Update info labels and dimension fields when magnet changes.

        Catalog magnet:
          - shape is derived from magnet (disc→circle, block→rect)
          - dimension fields are auto-filled from magnet geometry
          - fields remain EDITABLE so the user can adjust dimensions
          - m_cbShape (shape override) is disabled

        Custom:
          - m_cbShape is enabled; shape chosen manually
          - dimension fields are blank (user fills them)
        """
        idx = self.m_cbMagnet.GetSelection()
        is_custom = (idx == 0)

        # Shape override combobox: active only in Custom mode
        self.m_cbShape.Enable(is_custom)

        if is_custom:
            self.m_lblDerivedShape.SetLabel("—")
            self.m_lblBr.SetLabel("—")
            self.m_lblMagSize.SetLabel("—")
            shape = self.m_cbShape.GetStringSelection()
        else:
            mag = self._catalog[idx - 1]
            shape = mag["coil_shape"]
            self.m_lblDerivedShape.SetLabel(shape)
            self.m_lblBr.SetLabel(f"{mag['br']:.2f} T")
            if mag["mag_shape"] == "disc":
                self.m_lblMagSize.SetLabel(
                    f"\u00d8{mag['a_mm']:.0f}\u00d7{mag['h_mm']:.0f} mm")
            else:
                self.m_lblMagSize.SetLabel(
                    f"{mag['a_mm']:.0f}\u00d7{mag['b_mm']:.0f}"
                    f"\u00d7{mag['h_mm']:.0f} mm")

            # Auto-fill from magnet — fields stay editable for manual override
            if shape == "rect":
                self.m_ctrlA.SetValue(str(mag["a_mm"]))
                self.m_ctrlB.SetValue(str(mag["b_mm"]))
            else:  # circle
                self.m_ctrlRout.SetValue(str(mag["r_out_mm"]))
                self.m_ctrlRin.SetValue("2.0")   # default inner keep-out

        self._update_dim_panels(shape)
        if event:
            event.Skip()

    def on_cb_shape(self, event):
        """Only active in Custom mode — update panel visibility."""
        self._update_dim_panels(self.m_cbShape.GetStringSelection())
        if event:
            event.Skip()

    def on_btn_generate(self, event):
        event.Skip()

    def on_btn_clear(self, event):
        event.Skip()

    def on_close(self, event):
        self.Destroy()
