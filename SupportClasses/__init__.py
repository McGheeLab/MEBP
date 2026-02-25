"""
SupportClasses - Hardware abstraction and control layer for the 3D bioprinter.

Module Overview:
    Processor        - Thread-safe command bus with pub/sub dispatch
    XYStage          - Prior ProScan III XY stage manager
    XYStageSimulator - Physics-based XY simulator
    ZPStage          - Marlin-based Z/pump stage manager
    ZPStageSimulator - Physics-based Z/pump simulator
    XboxController   - Multiprocessing Xbox gamepad polling
    StageController  - Top-level orchestrator (jog handlers, position polling)
    PrintManager     - Print job loading, execution, queue management
    WellPlate        - Well plate geometry and path generation
    Settings         - JSON-backed application settings
    SafetyLimits     - Software endstop management
    PositionLogger   - Timestamped position recording
    SerialUtils      - Serial port utilities and error handling
"""

from SupportClasses.Processor import Processor
from SupportClasses.XYStage import XYStage
from SupportClasses.XYStageSimulator import XYStageSimulator
from SupportClasses.ZPStage import ZPStage, AXIS_MAP, AXIS_MAP_REVERSE
from SupportClasses.ZPStageSimulator import ZPStageSimulator
from SupportClasses.XboxController import xbox_polling_worker
from SupportClasses.StageController import StageController
from SupportClasses.PrintManager import (
    PrintManager, PrintQueue, PrintJob, PrintCommand,
    PrintSettings, PrintState, CommandType,
    load_print_file, save_print_job, build_well_plate_job,
)
from SupportClasses.WellPlate import WellPlate, WellInfo, PLATE_DEFINITIONS
from SupportClasses.Settings import Settings
from SupportClasses.SafetyLimits import SafetyLimits
from SupportClasses.PositionLogger import PositionLogger, PositionRecord
from SupportClasses.SerialUtils import (
    SerialError, SerialDisconnectedError, SerialTimeoutError, SerialAccessError,
    retry_serial, safe_write, safe_readline, safe_read_all,
    check_port_health, list_serial_ports, friendly_error_message,
    ConnectionWatchdog,
)

__all__ = [
    # Core
    "Processor",
    "StageController",
    # Stages
    "XYStage", "XYStageSimulator",
    "ZPStage", "ZPStageSimulator",
    "AXIS_MAP", "AXIS_MAP_REVERSE",
    # Input
    "xbox_polling_worker",
    # Printing
    "PrintManager", "PrintQueue",
    "PrintJob", "PrintCommand", "PrintSettings", "PrintState", "CommandType",
    "load_print_file", "save_print_job", "build_well_plate_job",
    # Utilities
    "WellPlate", "WellInfo", "PLATE_DEFINITIONS",
    "Settings",
    "SafetyLimits",
    "PositionLogger", "PositionRecord",
    # Serial
    "SerialError", "SerialDisconnectedError", "SerialTimeoutError", "SerialAccessError",
    "retry_serial", "safe_write", "safe_readline", "safe_read_all",
    "check_port_health", "list_serial_ports", "friendly_error_message",
    "ConnectionWatchdog",
]
