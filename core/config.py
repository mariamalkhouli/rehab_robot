# =============================================================================
#  core/config.py
#
#  Loads config.yaml once at startup and exposes a global CFG object.
#  Every other module imports CFG from here — never reads the YAML directly.
#
#  Usage in any other module:
#      from core.config import CFG
#      port = CFG.serial.port
#      threshold = CFG.sensors.fsr_push_threshold
# =============================================================================

import yaml
import os
import logging
from types import SimpleNamespace

logger = logging.getLogger(__name__)

# Absolute path to config.yaml — always relative to project root
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_CONFIG_PATH  = os.path.join(_PROJECT_ROOT, "config.yaml")


def _dict_to_namespace(d):
    """
    Recursively converts a nested dictionary into a SimpleNamespace tree
    so values can be accessed with dot notation:
        CFG.serial.port  instead of  CFG['serial']['port']
    """
    if isinstance(d, dict):
        return SimpleNamespace(**{k: _dict_to_namespace(v) for k, v in d.items()})
    elif isinstance(d, list):
        return [_dict_to_namespace(i) for i in d]
    return d


def _load_config(path: str) -> SimpleNamespace:
    """
    Reads and parses the YAML file.
    Raises a clear error if the file is missing or malformed.
    """
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"config.yaml not found at: {path}\n"
            f"Make sure it exists at the project root: {_PROJECT_ROOT}"
        )

    with open(path, "r") as f:
        try:
            raw = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ValueError(f"config.yaml is malformed:\n{e}")

    if not isinstance(raw, dict):
        raise ValueError("config.yaml must be a YAML mapping at the top level.")

    return _dict_to_namespace(raw)


def _validate_config(cfg: SimpleNamespace):
    """
    Validates critical parameters to catch misconfiguration early.
    Raises ValueError with a clear message if anything is wrong.
    """
    errors = []

    # --- Serial ---
    if not hasattr(cfg, 'serial'):
        errors.append("Missing section: serial")
    else:
        if cfg.serial.baud_rate not in [9600, 19200, 38400, 57600, 115200, 230400]:
            errors.append(f"serial.baud_rate {cfg.serial.baud_rate} is unusual. Double-check.")
        if cfg.serial.send_rate_hz > cfg.serial.receive_rate_hz:
            errors.append("serial.send_rate_hz cannot exceed serial.receive_rate_hz")

    # --- Safety ---
    if not hasattr(cfg, 'safety'):
        errors.append("Missing section: safety")
    else:
        if cfg.safety.heartbeat_timeout_ms < cfg.safety.heartbeat_interval_ms:
            errors.append(
                "safety.heartbeat_timeout_ms must be >= heartbeat_interval_ms. "
                "Otherwise the system will E-stop constantly."
            )

    # --- Motors ---
    if not hasattr(cfg, 'motors'):
        errors.append("Missing section: motors")
    else:
        expected_steps = round(
            (cfg.motors.steps_per_rev * cfg.motors.microstepping * cfg.motors.gear_ratio) / 360,
            2
        )
        if abs(expected_steps - cfg.motors.steps_per_degree) > 0.5:
            errors.append(
                f"motors.steps_per_degree ({cfg.motors.steps_per_degree}) does not match "
                f"calculated value ({expected_steps}) from steps_per_rev × microstepping "
                f"× gear_ratio / 360. Check your values."
            )
        for axis in ['ax1_direction', 'ax2_direction', 'ax3_direction', 'ax4_direction']:
            val = getattr(cfg.motors, axis, None)
            if val not in [1, -1]:
                errors.append(f"motors.{axis} must be 1 or -1, got: {val}")

    # --- Joints ---
    if not hasattr(cfg, 'joints'):
        errors.append("Missing section: joints")
    else:
        for joint_name in ['hip_ab_ad', 'hip_flex_ext', 'knee_flex_ext', 'axis4']:
            joint = getattr(cfg.joints, joint_name, None)
            if joint is None:
                errors.append(f"joints.{joint_name} section is missing")
                continue
            if joint.min_deg >= joint.max_deg:
                errors.append(f"joints.{joint_name}: min_deg must be < max_deg")
            if joint.safe_min_deg < joint.min_deg:
                errors.append(f"joints.{joint_name}: safe_min_deg < min_deg — unsafe")
            if joint.safe_max_deg > joint.max_deg:
                errors.append(f"joints.{joint_name}: safe_max_deg > max_deg — unsafe")
            if not (joint.min_deg <= joint.home_deg <= joint.max_deg):
                errors.append(f"joints.{joint_name}: home_deg must be within [min_deg, max_deg]")

    # --- Sensors ---
    # Single FSR (A0) + single EMG (A1) on the leg cuff.
    if not hasattr(cfg, 'sensors'):
        errors.append("Missing section: sensors")
    else:
        for threshold_name in ['fsr_push_threshold', 'fsr_lift_threshold',
                                'emg_threshold']:
            val = getattr(cfg.sensors, threshold_name, None)
            if val is None:
                errors.append(f"sensors.{threshold_name} is missing")
            elif not (0 <= val <= 1023):
                errors.append(
                    f"sensors.{threshold_name} = {val} must be between 0–1023"
                )
        rest = getattr(cfg.sensors, 'fsr_rest_raw', None)
        if rest is None:
            errors.append("sensors.fsr_rest_raw is missing")
        elif not (0 <= rest <= 1023):
            errors.append(f"sensors.fsr_rest_raw = {rest} must be between 0–1023")

    # --- Flask ---
    if not hasattr(cfg, 'flask'):
        errors.append("Missing section: flask")
    else:
        if cfg.flask.secret_key == "CHANGE_THIS_TO_A_LONG_RANDOM_STRING_BEFORE_DEPLOYMENT":
            # Warn but don't block — developer may not have changed it yet
            logger.warning(
                "⚠️  flask.secret_key is still the default placeholder. "
                "Generate a real key with: python3 -c \"import secrets; print(secrets.token_hex(32))\""
            )

    # --- Raise all errors at once ---
    if errors:
        error_msg = "config.yaml validation failed:\n" + "\n".join(f"  • {e}" for e in errors)
        raise ValueError(error_msg)


def _setup_logging(cfg: SimpleNamespace):
    """
    Configures the root logger based on config.yaml settings.
    Called immediately after config is loaded.
    """
    log_level = getattr(logging, cfg.system.log_level.upper(), logging.INFO)

    # Ensure logs directory exists
    log_dir = os.path.join(_PROJECT_ROOT, os.path.dirname(cfg.system.log_file))
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(_PROJECT_ROOT, cfg.system.log_file)

    from logging.handlers import RotatingFileHandler

    formatter = logging.Formatter(
        fmt="%(asctime)s | %(levelname)-8s | %(name)-25s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)

    # Rotating file handler
    file_handler = RotatingFileHandler(
        log_path,
        maxBytes=cfg.system.log_max_bytes,
        backupCount=cfg.system.log_backup_count
    )
    file_handler.setFormatter(formatter)

    # Apply to root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    root_logger.addHandler(console_handler)
    root_logger.addHandler(file_handler)

    logger.info(f"Logging initialized. Level={cfg.system.log_level}, File={log_path}")


# =============================================================================
#  Derived / computed values
#  These are calculated from raw config values and attached to CFG
#  so every module can use them without re-computing.
# =============================================================================

def _attach_derived_values(cfg: SimpleNamespace):
    """
    Attaches computed constants to the CFG object.
    These are derived from the raw config and never need to be set manually.
    """
    import math

    # Steps per degree (re-derived cleanly for use in calculations)
    cfg.motors.computed_steps_per_deg = (
        cfg.motors.steps_per_rev *
        cfg.motors.microstepping *
        cfg.motors.gear_ratio
    ) / 360.0

    # Maximum steps for each joint (useful for range validation)
    cfg.joints.hip_ab_ad.max_steps = round(
        cfg.joints.hip_ab_ad.max_deg * cfg.motors.computed_steps_per_deg
    )
    cfg.joints.hip_flex_ext.max_steps = round(
        cfg.joints.hip_flex_ext.max_deg * cfg.motors.computed_steps_per_deg
    )
    cfg.joints.knee_flex_ext.max_steps = round(
        cfg.joints.knee_flex_ext.max_deg * cfg.motors.computed_steps_per_deg
    )
    cfg.joints.axis4.max_steps = round(
        cfg.joints.axis4.max_deg * cfg.motors.computed_steps_per_deg
    )

    # Serial loop periods (in seconds) — used for sleep() calls
    cfg.serial.send_period_sec    = 1.0 / cfg.serial.send_rate_hz
    cfg.serial.receive_period_sec = 1.0 / cfg.serial.receive_rate_hz

    # Sensor RMS window in seconds (for display/logging)
    cfg.sensors.rms_window_sec = (
        cfg.sensors.emg_rms_window_samples / cfg.sensors.sample_rate_hz
    )

    # WebSocket stream periods
    cfg.websocket.sensor_period_sec = 1.0 / cfg.websocket.sensor_stream_rate_hz
    cfg.websocket.angle_period_sec  = 1.0 / cfg.websocket.angle_stream_rate_hz

    logger.debug("Derived config values computed and attached.")


# =============================================================================
#  Module-level initialization — runs once on first import
# =============================================================================

def load(path: str = _CONFIG_PATH) -> SimpleNamespace:
    """
    Full load + validate + setup cycle.
    Called once from main.py at startup.
    Returns the CFG namespace.
    """
    cfg = _load_config(path)
    _setup_logging(cfg)
    logger.info(f"config.yaml loaded from: {path}")
    _validate_config(cfg)
    logger.info("config.yaml validation passed.")
    _attach_derived_values(cfg)
    logger.info(f"System: {cfg.system.device_name} — ready.")
    return cfg


# =============================================================================
#  Global CFG object
#  Initialized to None. main.py must call:
#      from core.config import load_config
#      CFG = load_config()
#  All other modules then do:
#      from core.config import CFG
# =============================================================================

CFG: SimpleNamespace = None   # type: ignore


def load_config(path: str = _CONFIG_PATH) -> SimpleNamespace:
    """
    Public entry point called from main.py.
    Loads config, sets the global CFG, and returns it.

    Example in main.py:
        from core.config import load_config
        load_config()

    Then in any other module:
        from core.config import CFG
        print(CFG.serial.port)
    """
    global CFG
    CFG = load(path)
    return CFG