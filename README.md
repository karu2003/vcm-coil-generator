# VCM Coil Generator — KiCad Plugin

KiCad ActionPlugin for generating spiral PCB coils for a two-axis VCM actuator.

Architecture is based on [KiMotor](https://github.com/cooked/kimotor), adapted for rectangular/circular/sector geometry without rotational symmetry.

## Plugin Files

| File | Role |
|---|---|
| `__init__.py` | Registers `VCMCoilPlugin` in KiCad |
| `metadata.json` | Metadata (version, description) |
| `vcm_coil_gui.py` | wxPython dialog (parameters + buttons) |
| `vcm_coil_action.py` | `ActionPlugin` + track generation logic |
| `vcm_coil_solver.py` | Geometry engine (waypoints → nm) |
| `magnet_catalog.json` | NdFeB magnet catalog (copy from project root) |
| `vcm_coil_presets.json` | Saved user presets (created automatically) |

## Installation (KiCad 10, Windows)

```
cd C:\Users\xxxx\Documents\KiCad\10.0\scripting\plugins
mklink /J vcm_coil_generator "C:\Users\xxxx\Documents\Proj\Voice_Coil_Motor\kicad_plugin"
```

Then in KiCad: **Tools → External Plugins → Refresh Plugins**.  
A "VCM Coil Generator" button will appear in the toolbar.

## Dialog Parameters

### Magnet (from catalog)
Drop-down list of NdFeB magnets from `magnet_catalog.json`. When a magnet is selected:
- coil shape is derived automatically (disc → circle, block → rect)
- dimension fields are auto-filled from the catalog (remain editable)
- displayed: shape, Br (T), magnet size

Selecting **Custom** unlocks the manual shape selector.

### Axis / Direction / Start from / Shape override
| Parameter | Values | Description |
|---|---|---|
| **Axis** | X, Y | Net name: `coil_x` or `coil_y`. Run the plugin twice for both axes. |
| **Direction** | CW, CCW | Winding direction. All layers wind the same way → fields add up. |
| **Start from** | Outside→Inside, Inside→Outside | Direction the spiral begins winding. |
| **Shape** | rect, circle, sector | Custom mode only. |

### Rect coil dimensions (mm)
| Field | Description |
|---|---|
| **Width A (X)** | Outer coil dimension along X |
| **Height B (Y)** | Outer coil dimension along Y |

### Circle / sector dimensions (mm)
| Field | Description |
|---|---|
| **R outer** | Outer coil radius |
| **R inner** | Inner keep-out radius (no tracks) |

### Connection angles (°)
Shared by all shapes. Controls where the first and last VIA are placed.

| Field | Default | Description |
|---|---|---|
| **Start angle** | 0 | Position of the first VIA (input pad). |
| **End angle** | 360 | Position of the last VIA (output pad). |

**Rectangular coil angle convention:**  
VIAs are placed at the **centre of each edge**, not at corners.

| Angle | Edge | VIA position |
|-------|------|-------------|
| 0° | Right | mid-right, straight horizontal connection |
| 90° | Top | mid-top, straight vertical connection |
| 180° | Left | mid-left, straight horizontal connection |
| 270° | Bottom | mid-bottom, straight vertical connection |

Outer VIAs connect to the spiral with a **straight perpendicular track** (no L-bend).  
For **series connection**: set `End angle` of coil 1 = `Start angle` of coil 2.

### Track rules (mm)
| Field | Default | Description |
|---|---|---|
| **Track width** | 0.15 | Track width. JLCPCB Adv minimum: 0.09 mm |
| **Gap** | 0.15 | Gap between tracks. JLCPCB Adv minimum: 0.09 mm |

### Layer stack & via
| Field | Default | Description |
|---|---|---|
| **Layers** | 2 | Number of copper layers (1–16). F_Cu + inner + B_Cu. |
| **Via dia** | 0.5 | Via pad diameter. JLCPCB HDI: 0.25 mm |
| **Via drill** | 0.3 | Via drill diameter. JLCPCB HDI: 0.15 mm |
| **Via clearance** | 0.2 | Gap from coil boundary to via pad edge. Vias are placed **outside** the active spiral zone: beyond `r_out` or inside `r_in`. |
| **Start VIA offset** | 0.8 | Additional radial shift of `VIA[0]` (start terminal) away from winding copper to prevent interlayer touch at start. |
| **Silkscreen outline** | ✓ | Draw a coil outline on F_SilkS |
| *(field to the right)* | 0.12 | Silkscreen line width |

### Coil centre (mm)
| Field | Description |
|---|---|
| **CX / CY** | Coil centre in board coordinates |

### Preset
Bar above the action buttons. Saves and loads all dialog parameters to/from `vcm_coil_presets.json`.

| Button | Action |
|---|---|
| **Save** | Save current parameters under the entered name |
| **Load** | Load parameters from the selected preset |
| **Delete** | Delete preset (with confirmation) |

## Multi-Layer Winding Topology

All shapes (rect, circle, sector) use the **same topology**: N+1 vias for N layers.

```
Via[0] ──L0──> Via[1] ──L1──> Via[2] ── … ──L(N-1)──> Via[N]
  │                                                        │
Pad A (input)                                       Pad B (output)
```

- Even layers: outside→inside;  odd layers: inside→outside (or vice-versa)
- All layers wind in the **same rotational direction** → magnetic fields add up
- Every via is at a **unique XY position** in a safe zone (outside the coil boundary or inside the central void) → no collisions with tracks on any layer

### Rectangular coil — hybrid step + start corner

The winding direction (CCW) is the **same on every layer**, regardless of
inward/outward.  The transition between turns uses two different strategies
depending on radial direction:

- **Inward** (large→small): **concentrated step** — 3 clean sides, then a
  partial along the 4th side at the current turn's edge (outside all future
  turns), then a perpendicular step to the next turn.
- **Outward** (small→large): **distributed step** — the 3rd side transitions
  one coordinate, the 4th transitions the other.  The transition left is at
  the next turn's (larger) edge, which is outside the current turn.

Both strategies keep the transition segment **outside** the boundary of
subsequent turns, guaranteeing **zero T-junctions** and **zero crossings**.

Each layer starts at a different **corner** of the rectangle
(`start_corner` = 0 SW, 1 SE, 2 NE, 3 NW), analogous to `start_angle_deg`
for circular spirals.  The GUI `Start Angle` field maps to the nearest
corner: 0° → SW, 90° → SE, 180° → NE, 270° → NW.

| Layer | Radial | Start corner | End corner | Winding |
|---|---|---|---|---|
| 0 | Inward  | SW (0) | NW (3) | CCW |
| 1 | Outward | NW (3) | NE (2) | CCW |
| 2 | Inward  | NE (2) | SE (1) | CCW |
| 3 | Outward | SE (1) | SW (0) | CCW |

The natural progression (each via shifts by 3 corners ≈ 270°) distributes
vias across all 4 corners automatically.

### Via placement

**Outer VIAs** are at the centre of the coil edge, offset outward by
`d = w/2 + clr + vd/2`.  The spiral starts/ends at the edge midpoint
(half-edge segment), so the VIA connects with a **single straight
perpendicular track**.

| Edge | VIA position (j=0) | Spread (j>0) |
|------|-------------------|-------------|
| Right (0°) | (cx+xa+d, cy) | down (−Y) |
| Top (90°) | (cx, cy+xb+d) | right (+X) |
| Left (180°) | (cx−xa−d, cy) | up (+Y) |
| Bottom (270°) | (cx, cy−xb−d) | left (−X) |

**Inner VIAs** stay at rectangle corners with short L-stubs.

### End angle — series connection

The `End angle` parameter controls the last VIA's edge.  The last
layer's spiral is truncated so it naturally ends at the desired edge
midpoint (via `end_sides` + `end_mid_edge`).  No perimeter routing.

| Start | End | last es | Last turn |
|-------|-----|---------|-----------|
| 0° right | 180° left | 1 | 1 half-side |
| 0° right | 90° top | 2 | 1 full + 1 half |
| 0° right | 0° right | 4 | 3 full + 1 half |

### Inner void — optimal sizing

The inner void (spiral stop zone) is sized automatically to fit all inner
vias with proper clearance.  The algorithm:

```
max_n  = max vias at any single inner corner
spacing = vd + clr
void   = 2 × (d + (max_n − 1) × spacing)
```

| Layers | Inner vias | max_n | void (typical) |
|--------|-----------|-------|----------------|
| 2–4    | 1–2       | 1     | 1.2 mm         |
| 6–8    | 3–4       | 2     | 2.6 mm         |

The void is always square (`a_in = b_in`) because inner vias are at
diagonal corners and need equal clearance in both X and Y.  The void is
clamped to `coil_dim − 2×(w+s)` so that at least 2 turns always fit.

### 90° Manhattan stubs — no T-junctions

Each stub is an L-shaped 90° path.  Because the via is at the **same
corner** as the spiral start/end point, the stub is very short (~2d) and
routes entirely in the safe zone (outside the coil or inside the void).

| Stub type | Routing | Why no T-junction |
|---|---|---|
| Outer via → spiral start | Horiz first: through safe zone outside coil | Via and spiral start are at the same corner |
| Inner via → spiral start | Vert first: through void | Extends the spiral's first segment into void |
| Spiral end → any via | Horiz first: extends last segment | Routes away from coil then bends to via |

### Circular / sector stubs

- **Circle:** Vias are evenly spaced in angle → unique XY positions; a short radial stub connects each via to the spiral start/end point.
- **Sector:** Inner VIAs sit on the **sector bisector** inside the **hole enclosed by the innermost turn** (radius just below `r_in + (n_turns−1)·pitch`), not near `r_in` of the outermost turn (that region still carries upper turns’ copper). Stubs use a **two-segment** path (via → knee on bisector → anchor) when needed so the chord does not cut across inner arcs.

## Silkscreen Outline

| Shape | What is drawn |
|---|---|
| `circle` | Two circles: at `r_out` and `r_in` |
| `rect` | Rectangle `A × B` centred on CX/CY |
| `sector` | Two arcs + two radial lines |

The outline is added to the same group as the tracks and is removed by **Clear**.

## Connection to the Calculation Core

`vcm_coil_solver.py` uses the same logic as `pcb_coil_calc.py`:
- `rect_spiral` ↔ `calc_coil_geometry(shape="rect", ...)`
- Turn pitch: `pitch = w + s`
- Number of turns: `N = floor(r_span / pitch)`
- Archimedean spiral: 72 chord segments per turn, radius decreasing linearly with angle

## Dependencies

- KiCad 9 / 10 (Python 3.12+, wxPython, pcbnew)
- NumPy (included in KiCad Python)

## Status

| Feature | Status |
|---|---|
| Rectangular spiral | ✅ |
| Circular spiral (Archimedean) | ✅ |
| Sector spiral | ✅ |
| Multi-layer winding, N+1 vias | ✅ |
| Vias outside coil zone (clearance) | ✅ |
| Outer VIAs at edge midpoints (straight connection) | ✅ |
| Start / end angle for series connection | ✅ |
| Truncated last turn for custom end angle | ✅ |
| Silkscreen / outline | ✅ |
| Save/load presets | ✅ |
| Integration with magnet_catalog.json | ✅ |
| Start from Outside / Inside | ✅ |

## Support

If you find this project useful, consider supporting it:

[![Donate with PayPal](https://img.shields.io/badge/Donate-PayPal-blue?logo=paypal)](https://www.paypal.com/paypalme/BuckinAndrew)
