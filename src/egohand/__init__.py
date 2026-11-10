"""EgoHand Data Engine. All public 3D coordinates are left optical metres."""
__version__ = "0.1.0"

def __getattr__(name):
    if name == "EgoHandEngine":
        from .core.pipeline import EgoHandEngine
        return EgoHandEngine
    raise AttributeError(name)
