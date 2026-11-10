from PySide6.QtCore import Qt,QSize
from PySide6.QtWidgets import QFrame,QLabel,QVBoxLayout,QHBoxLayout,QPushButton,QSizePolicy
from .icons import icon

def label(text,role=None,name=None):
    w=QLabel(text)
    if role:w.setProperty("role",role)
    if name:w.setObjectName(name)
    return w

def badge(text,tone=None):
    w=label(text,name="badge");w.setSizePolicy(QSizePolicy.Maximum,QSizePolicy.Fixed);w.setAlignment(Qt.AlignCenter)
    if tone:w.setProperty("tone",tone)
    return w

def button(text,callback,icon_name=None,role=None,tooltip=None):
    w=QPushButton(text);w.setCursor(Qt.PointingHandCursor);w.setMinimumHeight(36)
    if icon_name:w.setIcon(icon(icon_name));w.setIconSize(QSize(18,18));w.setProperty("icon_name",icon_name)
    if role:w.setProperty("role",role)
    if tooltip:w.setToolTip(tooltip)
    w.setAccessibleName(tooltip or text);w.clicked.connect(callback)
    return w

class StatCard(QFrame):
    def __init__(self,title,unit,description):
        super().__init__();self.setObjectName("compactStat")
        row=QHBoxLayout(self);row.setContentsMargins(12,5,12,5);row.setSpacing(7)
        row.addWidget(label(title,"muted"));row.addStretch()
        self.value=label("—",name="statValue");self.unit=label(unit,name="statUnit")
        row.addWidget(self.value);row.addWidget(self.unit)
        self.description=label(description,"hint");self.description.hide()
        self.setFixedHeight(38);self.setToolTip(description);self.setAccessibleName(title)
    def update_value(self,value,description=None):
        self.value.setText(value)
        if description is not None:self.description.setText(description);self.setToolTip(description);self.setAccessibleDescription(description)

class ViewportCard(QFrame):
    def __init__(self,title,tag,view,subtitle=None):
        super().__init__();self.setObjectName("viewportCard")
        self.body=QVBoxLayout(self);self.body.setContentsMargins(1,1,1,1);self.body.setSpacing(0)
        head=QFrame();self.header=QHBoxLayout(head);self.header.setContentsMargins(15,9,15,9);self.header.setSpacing(9)
        self.header.addWidget(label(title,name="sectionTitle"));self.tag=badge(tag);self.header.addWidget(self.tag);self.header.addStretch()
        self.meta=label(subtitle or "等待输入","hint");self.header.addWidget(self.meta)
        self.body.addWidget(head);self.body.addWidget(view,1)

class HandReadout(QFrame):
    def __init__(self,title):
        super().__init__();self.setObjectName("handReadout")
        v=QVBoxLayout(self);v.setContentsMargins(12,12,12,12);v.setSpacing(8)
        row=QHBoxLayout();self.title=label(title,name="sectionTitle");self.track=label("未关联","hint")
        row.addWidget(self.title);row.addStretch();row.addWidget(self.track);v.addLayout(row)
        self.valid=label("等待手部观测","hint");v.addWidget(self.valid)
        self.xyz=label("X   —\nY   —\nZ   —","mono");self.xyz.setMinimumHeight(55);v.addWidget(self.xyz)
    def update_hand(self,hand):
        if hand is None:
            self.track.setText("未关联");self.valid.setText("等待手部观测");self.xyz.setText("X   —\nY   —\nZ   —");return
        self.track.setText(f"#{hand.track_id:02d}" if hand.track_id>=0 else "逐帧" if hand.tracking_status=='frame_only' else "待关联")
        self.valid.setText(f"{hand.valid_joint_count} / 21 有效关节")
        wrist=hand.joints[0]
        self.xyz.setText("\n".join(f"{axis}  {value:+.4f} m" for axis,value in zip("XYZ",wrist.xyz_m)) if wrist.valid else "X   —\nY   —\nZ   —")
