"""Semantic design tokens shared by native desktop widgets and the GL canvas."""
THEMES = {
    "dark": {
        "background": "#0D1118", "surface": "#151B25", "elevated": "#1B2431",
        "canvas": "#0E141D", "border": "#303D4E", "divider": "#273140",
        "text": "#E8EDF5", "muted": "#A2B0C2", "faint": "#8697AD",
        "accent": "#5FE0C5", "on_accent": "#092C26", "accent_soft": "#173C36",
        "warning": "#F0BB76", "warning_soft": "#3C3021", "danger": "#FF939D",
        "danger_soft": "#42262E", "focus": "#85BFFF", "input": "#101620",
        "left_hand": "#5FE0C5", "right_hand": "#F0BB76",
    },
    "light": {
        "background": "#EFF3F8", "surface": "#FFFFFF", "elevated": "#E7EDF5",
        "canvas": "#F7F9FC", "border": "#A4B1C2", "divider": "#D6DFEA",
        "text": "#16243A", "muted": "#4E6078", "faint": "#5C6E85",
        "accent": "#087F6C", "on_accent": "#FFFFFF", "accent_soft": "#E0F4EE",
        "warning": "#8B5C1B", "warning_soft": "#FFF0D9", "danger": "#B62D43",
        "danger_soft": "#FDE9ED", "focus": "#1B67BF", "input": "#F7F9FC",
        "left_hand": "#087F6C", "right_hand": "#986013",
    },
}

def stylesheet(name):
    from pathlib import Path
    check="check-light.svg" if name=="light" else "check.svg"
    t={**THEMES[name],"check_icon":str(Path(__file__).parent/'assets'/check).replace('\\','/')}
    return """
    QWidget { color:%(text)s; font-family:'Segoe UI','Microsoft YaHei UI'; font-size:13px; }
    QMainWindow, QWidget#applicationRoot { background:%(background)s; }
    QFrame#appHeader, QFrame#commandBar, QFrame#inspector, QFrame#viewportCard, QFrame#statCard { background:%(surface)s; }
    QFrame#appHeader { border-bottom:1px solid %(divider)s; }
    QFrame#commandBar { border:1px solid %(divider)s; border-radius:10px; }
    QFrame#inspector, QFrame#viewportCard { border:1px solid %(divider)s; border-radius:10px; }
    QFrame#statCard { border:1px solid %(divider)s; border-radius:9px; }
    QLabel { background:transparent; border:0; }
    QLabel#brand { font-size:20px; font-weight:600; }
    QLabel#brandEdition { color:%(muted)s; font-size:11px; letter-spacing:2px; }
    QLabel#pageTitle { font-size:21px; font-weight:600; }
    QLabel#sectionTitle { font-size:14px; font-weight:600; }
    QLabel#sectionEyebrow { color:%(muted)s; font-size:11px; }
    QLabel[role="muted"], QLabel#statusBanner { color:%(muted)s; }
    QLabel[role="hint"] { color:%(muted)s; font-size:12px; }
    QLabel[role="mono"] { font-family:'Cascadia Code','Consolas'; font-size:12px; }
    QLabel#statValue { font-size:18px; font-weight:600; }
    QFrame#metricStrip { background:%(surface)s; border:1px solid %(divider)s; border-radius:6px; }
    QFrame#compactStat { background:transparent; border:0; }
    QLabel#statUnit { color:%(muted)s; font-size:12px; }
    QLabel#statusBanner[tone="error"] { color:%(danger)s; }
    QLabel#badge { background:%(elevated)s; color:%(muted)s; border:1px solid %(divider)s; border-radius:5px; padding:4px 8px; font-size:11px; font-weight:600; }
    QLabel#badge[tone="accent"] { background:%(accent_soft)s; color:%(accent)s; border:0; }
    QLabel#badge[tone="warning"] { background:%(warning_soft)s; color:%(warning)s; border:0; }
    QLabel#badge[tone="error"] { background:%(danger_soft)s; color:%(danger)s; border:0; }
    QPushButton, QToolButton { background:%(elevated)s; color:%(text)s; border:1px solid %(border)s; border-radius:6px; padding:7px 12px; min-height:22px; font-weight:500; }
    QPushButton:hover, QToolButton:hover { border-color:%(faint)s; background:%(divider)s; }
    QPushButton:pressed, QToolButton:pressed { background:%(input)s; }
    QPushButton:checked, QToolButton:checked { color:%(accent)s; background:%(accent_soft)s; border-color:%(accent)s; }
    QPushButton[role="primary"] { color:%(on_accent)s; background:%(accent)s; border-color:%(accent)s; font-weight:600; }
    QPushButton[role="primary"]:hover { border-color:%(focus)s; }
    QPushButton[role="ghost"], QToolButton[role="ghost"] { background:transparent; border-color:transparent; }
    QPushButton:disabled, QToolButton:disabled { color:%(faint)s; background:%(surface)s; border-color:%(divider)s; }
    QPushButton:focus, QToolButton:focus, QComboBox:focus, QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QCheckBox:focus { border:2px solid %(focus)s; }
    QComboBox, QLineEdit, QSpinBox, QDoubleSpinBox { background:%(input)s; color:%(text)s; border:1px solid %(border)s; border-radius:5px; min-height:24px; padding:6px 9px; selection-background-color:%(accent_soft)s; }
    QComboBox::drop-down { border:0; width:24px; }
    QComboBox QAbstractItemView { background:%(surface)s; color:%(text)s; selection-background-color:%(accent_soft)s; }
    QCheckBox { spacing:9px; min-height:26px; background:transparent; }
    QCheckBox::indicator { width:15px; height:15px; border:1px solid %(faint)s; border-radius:3px; background:%(input)s; }
    QCheckBox::indicator:checked { background:%(accent)s; border-color:%(accent)s; image:url(%(check_icon)s); }
    QTabWidget::pane { border:0; background:transparent; }
    QTabBar::tab { background:transparent; color:%(muted)s; border:0; border-bottom:2px solid transparent; padding:10px 9px; min-width:30px; font-weight:500; }
    QTabBar::tab:selected { color:%(accent)s; border-bottom:2px solid %(accent)s; }
    QTabBar::tab:hover { color:%(text)s; background:%(elevated)s; }
    QGroupBox { background:transparent; border:0; border-top:1px solid %(divider)s; margin-top:20px; padding-top:20px; font-weight:600; }
    QGroupBox::title { subcontrol-origin:margin; subcontrol-position:top left; padding:0; }
    QScrollArea { border:0; background:transparent; }
    QScrollArea > QWidget > QWidget { background:transparent; }
    QScrollBar:vertical { background:transparent; width:6px; margin:2px; }
    QScrollBar::handle:vertical { background:%(border)s; border-radius:2px; min-height:35px; }
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height:0; }
    QSplitter::handle { background:transparent; }
    QSplitter::handle:hover { background:%(divider)s; }
    QFrame#handReadout { background:%(input)s; border:1px solid %(divider)s; border-radius:6px; }
    QStatusBar { background:%(background)s; border-top:1px solid %(divider)s; color:%(muted)s; font-size:11px; }
    QStatusBar::item { border:0; }
    QToolTip, QMenu { background:%(elevated)s; color:%(text)s; border:1px solid %(border)s; padding:6px; }
    QMenu::item { padding:8px 20px; }
    QMenu::item:selected { background:%(accent_soft)s; }
    """ % t
