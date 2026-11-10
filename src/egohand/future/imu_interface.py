from dataclasses import dataclass
from typing import Protocol,Iterator

@dataclass(frozen=True)
class IMUSample:
    sensor_timestamp_ns: int
    acceleration_m_s2: tuple[float,float,float]
    angular_velocity_rad_s: tuple[float,float,float]
    clock_domain: str
    hardware_sequence: int | None = None
    arrival_timestamp_ns: int | None = None
    temperature_c: float | None = None

class IMUProvider(Protocol):
    def samples(self) -> Iterator[IMUSample]: ...
