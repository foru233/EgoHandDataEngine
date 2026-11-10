"""Consistent 24px outline icons rendered by native Qt SVG."""
from PySide6.QtCore import QByteArray,Qt,QRectF
from PySide6.QtGui import QIcon,QPainter,QPixmap
from PySide6.QtSvg import QSvgRenderer

PATHS={
    "play":'<path d="m9 5 11 7-11 7z"/>',
    "pause":'<path d="M8 5v14M16 5v14"/>',
    "stop":'<rect x="6" y="6" width="12" height="12" rx="2"/>',
    "record":'<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="3"/>',
    "folder":'<path d="M3 8V5h6l3 3h9v12H3z"/>',
    "export":'<path d="M12 15V3m-4 4 4-4 4 4M4 14v6h16v-6"/>',
    "frame":'<path d="M8 3H3v5M16 3h5v5M21 16v5h-5M8 21H3v-5"/><circle cx="12" cy="12" r="3"/>',
    "reset":'<path d="M3 11a9 9 0 1 1 3 7M3 4v7h7"/>',
    "plus":'<path d="M12 5v14M5 12h14"/>',
    "minus":'<path d="M5 12h14"/>',
    "step":'<path d="m5 5 11 7-11 7zM19 5v14"/>',
    "seek":'<path d="M5 6v12l8-6zM17 5v14M21 5v14"/>',
    "layers":'<path d="m12 3 10 6-10 6L2 9zM2 14l10 6 10-6"/>',
    "model":'<rect x="6" y="6" width="12" height="12" rx="3"/><path d="M9 2v4m6-4v4M9 18v4m6-4v4M2 9h4m-4 6h4m12-6h4m-4 6h4"/>',
    "logo":'<path d="m4 17 8-12 8 12-8 3zM4 17l8-6 8 6M12 5v15"/><circle cx="12" cy="5" r="1"/>',
}

def icon(name,color="#A2B0C2",size=20):
    svg=f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="{color}" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round">{PATHS[name]}</svg>'
    pix=QPixmap(size*2,size*2);pix.fill(Qt.transparent);pix.setDevicePixelRatio(2)
    painter=QPainter(pix);QSvgRenderer(QByteArray(svg.encode())).render(painter,QRectF(0,0,size,size));painter.end()
    return QIcon(pix)
