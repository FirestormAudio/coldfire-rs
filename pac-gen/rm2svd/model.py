from dataclasses import dataclass, field


@dataclass
class Field:
    name: str
    lsb: int
    width: int
    access: str
    one_to_clear: bool = False
    enum: dict[str, int] = field(default_factory=dict)


@dataclass
class Register:
    name: str
    offset: int
    width: int
    access: str
    reset: int | None
    fields: list[Field] = field(default_factory=list)


@dataclass
class Peripheral:
    name: str
    base: int
    group: str
    registers: list[Register] = field(default_factory=list)
    derived_from: str | None = None


@dataclass
class Interrupt:
    name: str
    value: int
    peripheral: str


@dataclass
class Device:
    name: str
    peripherals: list[Peripheral]
    interrupts: list[Interrupt]
    availability: dict[str, dict[str, int]]
