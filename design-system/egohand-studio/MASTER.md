# EgoHand Studio — Desktop Design System

Source skill: `<codex-home>/skills/ui-ux-pro-max/SKILL.md`.
Installed from <https://github.com/nextlevelbuilder/ui-ux-pro-max-skill> with the official Codex skill-installer helper.

## Applied guidance and fit

The `industrial scientific monitoring dashboard --design-system` search returned Minimalism & Swiss Style, dark slate surfaces, operational telemetry, and technical typography. These fit a computer-vision workstation.
The accompanying landing-page conversion pattern does not fit this desktop app and was rejected. A narrower instrument query incorrectly matched music/entertainment; its colors/type were rejected. The skill has no PySide6 stack. Native Qt implementation uses the verified general style plus the skill's stack-agnostic `quick-reference.md` rules. The progressive-disclosure domain query also returned an unrelated heading rule; tabs follow the explicit quick-reference guideline, not that database match.

## Visual language

- Graphite dark / slate light, mint primary action, amber right-hand identification. Semantic tokens live in `gui/theme.py`.
- Segoe UI with Microsoft YaHei fallback; Cascadia Code / Consolas for coordinates. No network fonts or new GUI dependencies.
- Consistent outline SVG icons; labels, keyboard shortcuts, hover/pressed/disabled states, accessible names and visible focus.
- Compact 4/8px spacing rhythm; restrained borders, no decorative animation, no extra marketing/navigation surfaces.
- Actual observations and source provenance only. Empty values are em dashes, never fake telemetry or mock success.

## Layout contract, incorporating user feedback

- Header and command strip remain compact. Four metrics share a 38px-high row, with detailed explanations in tooltips / Diagnostics.
- Viewports get the remaining vertical space. Image rendering uses KeepAspectRatio, never stretching or silent cropping.
- Automatic layout computes camera dimensions from the source image ratio. Wide/short windows place the stereo pair vertically at left and 3D at right. Tall windows can put two cameras above 3D.
- User can explicitly select stereo-at-left/stereo-at-top and drag separators. Resizing restores the selected layout's size allocation.
- Right configuration pane is collapsible. Acquisition, Detection, Recording, Coordinates and Diagnostics use separate tabs.
- Initial window size is bounded by the current screen's available area. Validated native sizes: 1500×960, 1280×820, 1100×720, 1280×1000.
- Camera image occupancy stays >=76% at tested sizes and usually >98% in automatic layout, without losing any input pixels.

## Rendering and themes

Qt Fusion provides consistent Windows widget styling. SVG rasterization uses explicit logical bounds for high DPI.
The GL scene uses opaque non-smoothed grid/line primitives and translucent point sprites to avoid the observed Windows/NVIDIA legacy line-smoothing artifact in light mode. Qt child labels provide the legend without QPainter calls inside GL item traversal.
Dark/light screenshots and actual OpenGL context validity are captured by `scripts/preview_ui.py`; no browser mockup replaces the native desktop app.
