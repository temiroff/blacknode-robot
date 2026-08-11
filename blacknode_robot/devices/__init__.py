"""Connected-device contracts and safe provider lifecycle."""

from .contracts import (
    DeviceState,
    FaultState,
    JointState,
    MobileBaseCommand,
    MobileBaseProvider,
)
from .safety import SafetyGate, SafetyLimits
from .adapters import (
    ExistingRos2Config,
    ExistingRos2Monitor,
    I2CMecanumBase,
    I2CMecanumConfig,
    SerialJointConfig,
    SerialJointGroup,
    SerialJointMonitor,
    SerialJointSpec,
    probe_serial,
)
from .joint_group import JointGroupCommand, JointGroupProvider, JointGroupState
from .version import service_version

__version__ = service_version()

__all__ = [
    "DeviceState",
    "FaultState",
    "JointState",
    "MobileBaseCommand",
    "MobileBaseProvider",
    "SafetyGate",
    "SafetyLimits",
    "ExistingRos2Config",
    "ExistingRos2Monitor",
    "I2CMecanumBase",
    "I2CMecanumConfig",
    "SerialJointConfig",
    "SerialJointGroup",
    "SerialJointMonitor",
    "SerialJointSpec",
    "probe_serial",
    "JointGroupCommand",
    "JointGroupProvider",
    "JointGroupState",
    "__version__",
]
