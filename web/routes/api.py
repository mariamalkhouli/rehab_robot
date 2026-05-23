# =============================================================================
#  web/routes/api.py
#  /home/pi/rehab_robot/web/routes/api.py
#
#  All REST API endpoints for the Rehab Robot dashboard.
#
#  ENDPOINT GROUPS:
#    /api/system/*       → System status, E-stop, calibration
#    /api/therapists/*   → Therapist CRUD + PIN auth
#    /api/patients/*     → Patient CRUD
#    /api/sessions/*     → Session lifecycle management
#    /api/therapy/*      → Therapy mode control (start CPM, active, mimicry)
#    /api/keyframes/*    → Mimicry mode path management
#    /api/analytics/*    → Progress data for charts
#    /api/media/*        → Video file management
#
#  ALL responses follow this structure:
#    Success: { "ok": true,  "data": {...} }
#    Error:   { "ok": false, "error": "message" }
# =============================================================================

import os
import logging
from flask import Blueprint, request, jsonify, current_app

from core.database import (
    # Therapists
    create_therapist, get_all_therapists, verify_therapist_pin,
    # Patients
    create_patient, get_patient, get_all_patients,
    update_patient,
    update_patient_notes, archive_patient,
    # Sessions
    create_session, end_session, get_session,
    get_patient_sessions, increment_estop_count,
    # ROM
    save_rom_assessment, get_patient_rom_history,
    # Paths & Keyframes
    create_therapy_path, get_patient_paths,
    save_keyframe, get_keyframes_for_path, delete_path,
    # E-stop
    log_estop_event, clear_estop, get_recent_estops,
    # Calibration
    save_calibration, get_current_calibration,
    # System
    log_system_event, get_system_events,
    # Analytics
    get_effort_trend, get_rom_trend,
)

logger = logging.getLogger(__name__)

api_bp = Blueprint("api", __name__, url_prefix="/api")

# ---------------------------------------------------------------------------
#  Helper — get hardware services safely
#  Returns None if hardware not connected (allows UI to work without hardware)
# ---------------------------------------------------------------------------

def _mgr():
    from web.app import get_session_manager
    return get_session_manager()

def _serial():
    from web.app import get_serial_comm
    return get_serial_comm()

def _safety():
    from web.app import get_safety_monitor
    return get_safety_monitor()

def _sensor():
    from web.app import get_sensor_hub
    return get_sensor_hub()

def ok(data=None):
    return jsonify({"ok": True,  "data": data or {}})

def err(message, code=400):
    return jsonify({"ok": False, "error": message}), code


# =============================================================================
#  SYSTEM
# =============================================================================

@api_bp.route("/system/status")
def system_status():
    """Full system status — hardware connections, safety state, sensor values."""
    serial  = _serial()
    safety  = _safety()
    sensor  = _sensor()

    serial_status = serial.get_stats()    if serial  else {"is_open": False}
    safety_status = safety.get_status()   if safety  else {"state": "DISARMED"}
    sensor_status = sensor.get_full_status() if sensor else {}

    return ok({
        "serial"  : serial_status,
        "safety"  : safety_status,
        "sensors" : sensor_status,
    })


@api_bp.route("/system/ping")
def system_ping():
    """Quick liveness check — used by dashboard to confirm server is alive."""
    return ok({"alive": True})


@api_bp.route("/system/estop/clear", methods=["POST"])
def estop_clear():
    """
    Clear the E-stop state and re-arm the system.
    Must only be called after therapist confirms it is safe.
    """
    data       = request.get_json(silent=True) or {}
    cleared_by = data.get("cleared_by", "therapist")
    estop_id   = data.get("estop_id")

    safety = _safety()
    serial = _serial()

    if safety:
        success = safety.clear_estop(cleared_by=cleared_by)
        if not success:
            return err(
                "Cannot clear E-stop: system is not in ESTOPPED state "
                "or ESP32 heartbeat is not restored yet."
            )

    if serial:
        serial.enable_motors()

    if estop_id:
        clear_estop(estop_id, cleared_by=cleared_by)

    log_system_event(
        "estop", "info",
        f"E-stop cleared by '{cleared_by}'.",
        "api"
    )
    return ok({"cleared": True})


@api_bp.route("/system/estop/log", methods=["POST"])
def estop_log():
    """Log an E-stop event manually (called from dashboard E-stop button)."""
    serial = _serial()
    safety = _safety()

    angles     = serial.get_current_angles() if serial else [None]*4
    session_id = serial.get_active_session_id() if serial else None

    estop_id = log_estop_event(
        session_id     = session_id,
        trigger_source = "manual_dashboard",
        trigger_reason = "Therapist pressed E-stop button on dashboard.",
        ax1=angles[0], ax2=angles[1], ax3=angles[2], ax4=angles[3]
    )

    if serial:
        serial.send_halt()
    if safety and hasattr(safety, '_trigger_estop'):
        pass   # Safety monitor state updated via callback chain

    return ok({"estop_id": estop_id})


@api_bp.route("/system/estops/recent")
def recent_estops():
    rows = get_recent_estops(limit=20)
    return ok([dict(r) for r in rows])


@api_bp.route("/system/events")
def system_events():
    severity = request.args.get("severity")
    limit    = int(request.args.get("limit", 50))
    rows     = get_system_events(severity=severity, limit=limit)
    return ok([dict(r) for r in rows])


@api_bp.route("/system/calibration/current")
def calibration_current():
    row = get_current_calibration()
    return ok(dict(row) if row else None)


@api_bp.route("/system/calibration/save", methods=["POST"])
def calibration_save():
    data = request.get_json(silent=True) or {}
    cal_id = save_calibration(
        calibrated_by = data.get("therapist_id"),
        ax1_home      = data.get("ax1_home", 0.0),
        ax2_home      = data.get("ax2_home", 0.0),
        ax3_home      = data.get("ax3_home", 0.0),
        ax4_home      = data.get("ax4_home", 0.0),
        ax1_offset    = data.get("ax1_offset", 0.0),
        ax2_offset    = data.get("ax2_offset", 0.0),
        ax3_offset    = data.get("ax3_offset", 0.0),
        ax4_offset    = data.get("ax4_offset", 0.0),
    )
    return ok({"calibration_id": cal_id})


@api_bp.route("/system/fsr/calibrate", methods=["POST"])
def fsr_calibrate():
    """Trigger FSR rest baseline calibration."""
    sensor = _sensor()
    if not sensor:
        return err("Sensor hub not available.")
    rest_value = sensor.calibrate_fsr_rest()
    return ok({"fsr_rest_raw": rest_value})


# =============================================================================
#  THERAPISTS
# =============================================================================

@api_bp.route("/therapists", methods=["GET"])
def therapists_list():
    rows = get_all_therapists()
    return ok([dict(r) for r in rows])


@api_bp.route("/therapists", methods=["POST"])
def therapists_create():
    data = request.get_json(silent=True) or {}
    required = ["first_name", "last_name", "pin"]
    for field in required:
        if not data.get(field):
            return err(f"Missing required field: {field}")

    t_id = create_therapist(
        first_name = data["first_name"],
        last_name  = data["last_name"],
        pin        = data["pin"],
        role       = data.get("role", "therapist")
    )
    return ok({"id": t_id}), 201


@api_bp.route("/therapists/<int:therapist_id>/verify", methods=["POST"])
def therapist_verify(therapist_id):
    data = request.get_json(silent=True) or {}
    pin  = data.get("pin", "")
    valid = verify_therapist_pin(therapist_id, pin)
    if not valid:
        return err("Invalid PIN.", code=401)
    return ok({"verified": True})


# =============================================================================
#  PATIENTS
# =============================================================================

@api_bp.route("/patients", methods=["GET"])
def patients_list():
    active_only = request.args.get("active", "true").lower() == "true"
    rows        = get_all_patients(active_only=active_only)
    return ok([dict(r) for r in rows])


@api_bp.route("/patients/<int:patient_id>", methods=["GET"])
def patient_get(patient_id):
    row = get_patient(patient_id)
    if not row:
        return err("Patient not found.", code=404)
    return ok(dict(row))


@api_bp.route("/patients", methods=["POST"])
def patient_create():
    data     = request.get_json(silent=True) or {}
    required = ["first_name", "last_name", "date_of_birth",
                "affected_side", "diagnosis"]
    for field in required:
        if not data.get(field):
            return err(f"Missing required field: {field}")

    p_id = create_patient(
        first_name    = data["first_name"],
        last_name     = data["last_name"],
        date_of_birth = data["date_of_birth"],
        gender        = data.get("gender"),
        affected_side = data["affected_side"],
        weight_kg     = data.get("weight_kg"),
        height_cm     = data.get("height_cm"),
        diagnosis     = data["diagnosis"],
        contact_phone = data.get("contact_phone"),
        notes         = data.get("notes"),
        therapist_id  = data.get("therapist_id"),
    )
    return ok({"id": p_id}), 201


@api_bp.route("/patients/<int:patient_id>/notes", methods=["PATCH"])
def patient_update_notes(patient_id):
    data  = request.get_json(silent=True) or {}
    notes = data.get("notes", "")
    update_patient_notes(patient_id, notes)
    return ok({"updated": True})


@api_bp.route("/patients/<int:patient_id>", methods=["PATCH"])
def patient_update(patient_id):
    """
    Update all editable patient fields.
    Called by the Edit Patient modal on the patients page.
    """
    data = request.get_json(silent=True) or {}
 
    required = ["first_name", "last_name", "date_of_birth",
                "affected_side", "diagnosis"]
    for field in required:
        if not data.get(field):
            return err(f"Missing required field: {field}")
 
    update_patient(
        patient_id    = patient_id,
        first_name    = data["first_name"],
        last_name     = data["last_name"],
        date_of_birth = data["date_of_birth"],
        gender        = data.get("gender"),
        affected_side = data["affected_side"],
        weight_kg     = data.get("weight_kg"),
        height_cm     = data.get("height_cm"),
        diagnosis     = data["diagnosis"],
        contact_phone = data.get("contact_phone"),
        notes         = data.get("notes"),
    )
 
    # Return the updated patient record so the UI can refresh
    row = get_patient(patient_id)
    return ok(dict(row) if row else {"updated": True})


@api_bp.route("/patients/<int:patient_id>/archive", methods=["POST"])
def patient_archive(patient_id):
    archive_patient(patient_id)
    return ok({"archived": True})


@api_bp.route("/patients/<int:patient_id>/sessions", methods=["GET"])
def patient_sessions(patient_id):
    rows = get_patient_sessions(patient_id)
    return ok([dict(r) for r in rows])


@api_bp.route("/patients/<int:patient_id>/rom", methods=["GET"])
def patient_rom(patient_id):
    joint = request.args.get("joint")
    rows  = get_patient_rom_history(patient_id, joint_name=joint)
    return ok([dict(r) for r in rows])


# =============================================================================
#  SESSIONS
# =============================================================================

@api_bp.route("/sessions/start", methods=["POST"])
def session_start():
    """
    MASTER LINK: This function connects the 'Start' button to the motors.
    """
    data = request.get_json(silent=True) or {}

    # 1. Validation
    required = ["patient_id", "mode"]
    for field in required:
        if not data.get(field):
            return err(f"Missing required field: {field}")

    # 2. Get the Manager (Robot Brain)
    # This was wired correctly in your app.py grep!
    mgr = _mgr() 
    if not mgr:
        logger.error("API Error: SessionManager is None. Check main.py startup.")
        return err("Robot Manager not initialized", 500)

    # 3. Trigger the actual Robot Session
    # This call travels from API -> Manager -> PassiveMode -> SerialComm -> Arduino
    try:
        result = mgr.start_session(
            patient_id        = int(data["patient_id"]),
            mode              = data["mode"],
            exercise   = data.get("exercise", "hip_knee_flex"),
            leg               = data.get("leg", "right"),
            hip_flex_max      = float(data.get("hip_flex_max", 90.0)),
            knee_flex_max     = float(data.get("knee_flex_max", 90.0)),
            ab_ad_max         = float(data.get("ab_ad_max", 25.0)),
            speed_deg_per_sec = float(data.get("speed", 5.0)),
            reps_target       = int(data.get("reps_target", 10))
        )
        
        if result.get("ok"):
            logger.info(f"SUCCESS: Session #{result['session_id']} started via Web.")
            return ok(result), 201
        else:
            return err(result.get("error", "Failed to start session"))

    except Exception as e:
        import traceback
        traceback.print_exc() # Shows the mismatch in your terminal
        return err(f"Internal Logic Error: {str(e)}", 500)


@api_bp.route("/sessions/<int:session_id>/end", methods=["POST"])
def session_end(session_id):
    data = request.get_json(silent=True) or {}

    end_session(
        session_id         = session_id,
        status             = data.get("status", "completed"),
        hip_ab_achieved    = data.get("hip_ab_achieved"),
        hip_flex_achieved  = data.get("hip_flex_achieved"),
        knee_flex_achieved = data.get("knee_flex_achieved"),
        reps_done          = data.get("reps_done"),
        notes              = data.get("notes"),
    )

    serial = _serial()
    if serial:
        serial.set_active_session_id(None)

    return ok({"ended": True})


@api_bp.route("/sessions/<int:session_id>", methods=["GET"])
def session_get(session_id):
    row = get_session(session_id)
    if not row:
        return err("Session not found.", code=404)
    return ok(dict(row))


@api_bp.route("/sessions/<int:session_id>/rom", methods=["POST"])
def session_save_rom(session_id):
    data = request.get_json(silent=True) or {}
    rom_id = save_rom_assessment(
        session_id  = session_id,
        patient_id  = data.get("patient_id"),
        joint_name  = data.get("joint_name"),
        passive_rom = data.get("passive_rom"),
        active_rom  = data.get("active_rom"),
        pain_score  = data.get("pain_score"),
        notes       = data.get("notes"),
    )
    return ok({"rom_id": rom_id}), 201


# =============================================================================
#  THERAPY MODE CONTROL
# =============================================================================

@api_bp.route("/therapy/mode", methods=["POST"])
def therapy_set_mode():
    """
    Switch therapy mode during an active session.
    Modes: passive | active | mimicry | idle
    """
    data   = request.get_json(silent=True) or {}
    mode   = data.get("mode")
    serial = _serial()

    if not mode:
        return err("Missing field: mode")

    if serial and serial.is_halted():
        return err("Cannot change mode: system is halted. Clear E-stop first.")

    # Actual mode switching is handled by therapy engine modules.
    # This endpoint signals intent — therapy modules pick it up.
    log_system_event(
        "startup", "info",
        f"Therapy mode change requested: {mode}",
        "api"
    )
    return ok({"mode": mode})


@api_bp.route("/therapy/angles", methods=["POST"])
def therapy_send_angles():
    """
    Send target angles directly — used by mimicry mode jogging from UI.
    """
    data   = request.get_json(silent=True) or {}
    angles = data.get("angles", [])
    serial = _serial()

    if len(angles) != 4:
        return err("angles must be a list of exactly 4 floats.")

    if not serial:
        return err("Serial not connected.")

    if serial.is_halted():
        return err("System is halted.")

    success = serial.send_angles(angles)
    return ok({"sent": success})


@api_bp.route("/therapy/jog", methods=["POST"])
def therapy_jog():
    """Jog one axis by steps — mimicry mode manual positioning."""
    data   = request.get_json(silent=True) or {}
    axis   = data.get("axis")
    steps  = data.get("steps")
    serial = _serial()

    if axis is None or steps is None:
        return err("Missing fields: axis, steps")

    if not serial:
        return err("Serial not connected.")

    success = serial.send_jog(int(axis), int(steps))
    return ok({"sent": success})


@api_bp.route("/therapy/halt", methods=["POST"])
def therapy_halt():
    """Manual HALT from dashboard E-stop button."""
    serial = _serial()
    if serial:
        serial.send_halt()
    return ok({"halted": True})


@api_bp.route("/therapy/enable", methods=["POST"])
def therapy_enable():
    """Re-enable motors after HALT — called after E-stop cleared."""
    serial = _serial()
    if serial:
        serial.enable_motors()
    return ok({"enabled": True})


@api_bp.route("/therapy/home", methods=["POST"])
def therapy_home():
    """Trigger full homing sequence."""
    serial = _serial()
    if not serial:
        return err("Serial not connected.")
    serial.send_home_all()
    return ok({"homing": True})


# =============================================================================
#  THERAPY PATHS & KEYFRAMES (Mimicry Mode)
# =============================================================================

@api_bp.route("/paths", methods=["GET"])
def paths_list():
    patient_id = request.args.get("patient_id", type=int)
    if not patient_id:
        return err("Missing query param: patient_id")
    rows = get_patient_paths(patient_id)
    return ok([dict(r) for r in rows])


@api_bp.route("/paths", methods=["POST"])
def path_create():
    data = request.get_json(silent=True) or {}
    path_id = create_therapy_path(
        patient_id     = data.get("patient_id"),
        created_by     = data.get("therapist_id"),
        name           = data.get("name", "Untitled Path"),
        target_joints  = data.get("target_joints", ""),
        speed          = data.get("speed", 5.0),
        loop_count     = data.get("loop_count", 1),
    )
    return ok({"path_id": path_id}), 201


@api_bp.route("/paths/<int:path_id>", methods=["DELETE"])
def path_delete(path_id):
    delete_path(path_id)
    return ok({"deleted": True})


@api_bp.route("/paths/<int:path_id>/keyframes", methods=["GET"])
def keyframes_list(path_id):
    rows = get_keyframes_for_path(path_id)
    return ok([dict(r) for r in rows])


@api_bp.route("/paths/<int:path_id>/keyframes", methods=["POST"])
def keyframe_save(path_id):
    """
    Save current motor position as a keyframe.
    Automatically uses current angles from serial_comm.
    """
    data   = request.get_json(silent=True) or {}
    serial = _serial()

    if serial:
        angles = serial.get_current_angles()
    else:
        angles = [
            data.get("ax1", 0.0),
            data.get("ax2", 0.0),
            data.get("ax3", 0.0),
            data.get("ax4", 0.0),
        ]

    # Get next order index
    existing = get_keyframes_for_path(path_id)
    order    = len(existing)

    kf_id = save_keyframe(
        path_id      = path_id,
        order_index  = order,
        ax1          = angles[0],
        ax2          = angles[1],
        ax3          = angles[2],
        ax4          = angles[3],
        hold_time_ms = data.get("hold_time_ms", 0.0),
        interp_speed = data.get("interp_speed"),
    )
    return ok({"keyframe_id": kf_id, "angles": angles}), 201

@api_bp.route("/therapy/mimicry/keyframe", methods=["POST"])
def mimicry_add_keyframe():
    mgr = _mgr()
    if not mgr or mgr._mode != 'mimicry':
        return err("No active Mimicry session.")

    if mgr._mode != 'mimicry':
        return err(f"System is in '{mgr._mode}' mode. You must click the Green START button while in Mimicry tab first!")
    
    # Access the MimicryMode object inside the SessionManager
    mimicry_obj = mgr._mode_obj 
    if not mimicry_obj:
        return err("Mimicry engine not ready.")

    data = request.get_json()
    # This calls the add_keyframe method in your mimicry_mode.py file!
    result = mimicry_obj.add_keyframe(
        hold_time_ms = data.get("hold_time_ms", 0.0)
    )
    return jsonify(result)

@api_bp.route("/therapy/mimicry/replay", methods=["POST"])
def mimicry_start_replay():
    mgr = _mgr()
    
    # 1. Check if the session manager exists
    if not mgr:
        return err("Robot Manager not initialized.")
        
    # 2. Check if the Green START button was clicked (mgr._mode must be 'mimicry')
    if mgr._mode != 'mimicry':
        return err("System not in Mimicry mode. Click the Green START button first!")

    # 3. Check if the Mimicry object is alive
    if not mgr._mode_obj:
        return err("Mimicry engine not ready.")

    # 4. Start the replay thread
    result = mgr._mode_obj.start_replay()
    
    if result.get('ok'):
        return ok(result)
    else:
        return err(result.get('error', 'Replay failed to start'))


# =============================================================================
#  ANALYTICS
# =============================================================================

@api_bp.route("/analytics/<int:patient_id>/effort")
def analytics_effort(patient_id):
    n    = request.args.get("sessions", 10, type=int)
    rows = get_effort_trend(patient_id, last_n_sessions=n)
    return ok([dict(r) for r in rows])


@api_bp.route("/analytics/<int:patient_id>/rom")
def analytics_rom(patient_id):
    joint = request.args.get("joint", "knee_flex_ext")
    rows  = get_rom_trend(patient_id, joint_name=joint)
    return ok([dict(r) for r in rows])


# =============================================================================
#  MEDIA — Videos for patient entertainment during sessions
# =============================================================================

MEDIA_DIR = os.path.join(
    os.path.dirname(__file__), "..", "..", "web", "static", "media"
)

@api_bp.route("/media/videos", methods=["GET"])
def media_videos_list():
    """List all video files stored locally on the Pi."""
    try:
        os.makedirs(MEDIA_DIR, exist_ok=True)
        video_extensions = {".mp4", ".webm", ".ogg", ".mkv"}
        files = [
            f for f in os.listdir(MEDIA_DIR)
            if os.path.splitext(f)[1].lower() in video_extensions
        ]
        videos = [
            {
                "name"    : f,
                "url"     : f"/static/media/{f}",
                "type"    : "local",
            }
            for f in sorted(files)
        ]
        return ok(videos)
    except Exception as e:
        logger.error(f"Error listing media: {e}")
        return err("Could not list media files.")