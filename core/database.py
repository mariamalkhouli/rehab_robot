# core/database.py
import sqlite3
import hashlib
import os
from datetime import datetime
from contextlib import contextmanager
#from core.config import CFG
import logging

logger = logging.getLogger(__name__)

DB_PATH = os.path.join(os.path.dirname(__file__), '..', 'rehab.db')

# ─────────────────────────────────────────────
# Connection context manager
# ─────────────────────────────────────────────

@contextmanager
def get_db():
    # Increase timeout and use isolation_level=None for better concurrency
    conn = sqlite3.connect(DB_PATH, timeout=30, detect_types=sqlite3.PARSE_DECLTYPES)
    conn.row_factory = sqlite3.Row
    
    # Force WAL mode every time to ensure Pi 5 performance
    try:
        conn.execute("PRAGMA journal_mode = WAL")
        conn.execute("PRAGMA synchronous = NORMAL")
        conn.execute("PRAGMA foreign_keys = ON")
        yield conn
        conn.commit()
    except Exception as e:
        conn.rollback()
        raise e
    finally:
        conn.close()


# ─────────────────────────────────────────────
# Schema creation
# ─────────────────────────────────────────────

def init_db():
    with get_db() as conn:
        conn.executescript("""

        -- ── THERAPISTS ───────────────────────────────────────────────────────
        CREATE TABLE IF NOT EXISTS therapists (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            first_name      TEXT    NOT NULL,
            last_name       TEXT    NOT NULL,
            pin_hash        TEXT    NOT NULL,          -- SHA-256 of 4-digit PIN
            role            TEXT    NOT NULL DEFAULT 'therapist',  -- 'admin' | 'therapist'
            created_at      TEXT    NOT NULL DEFAULT (datetime('now'))
        );

        -- ── PATIENTS ─────────────────────────────────────────────────────────
        CREATE TABLE IF NOT EXISTS patients (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            therapist_id    INTEGER REFERENCES therapists(id) ON DELETE SET NULL,
            first_name      TEXT    NOT NULL,
            last_name       TEXT    NOT NULL,
            date_of_birth   TEXT    NOT NULL,          -- ISO-8601: YYYY-MM-DD
            gender          TEXT,                      -- 'M' | 'F' | 'other'
            affected_side   TEXT    NOT NULL,          -- 'left' | 'right' | 'bilateral'
            weight_kg       REAL,
            height_cm       REAL,
            diagnosis       TEXT    NOT NULL,
            contact_phone   TEXT,
            notes           TEXT,
            created_at      TEXT    NOT NULL DEFAULT (datetime('now')),
            is_active       INTEGER NOT NULL DEFAULT 1  -- 1=active, 0=archived
        );

        -- ── SESSIONS ─────────────────────────────────────────────────────────
        CREATE TABLE IF NOT EXISTS sessions (
            id                      INTEGER PRIMARY KEY AUTOINCREMENT,
            patient_id              INTEGER NOT NULL REFERENCES patients(id),
            therapist_id            INTEGER REFERENCES therapists(id) ON DELETE SET NULL,
            start_time              TEXT    NOT NULL DEFAULT (datetime('now')),
            end_time                TEXT,
            mode                    TEXT    NOT NULL,  -- 'passive'|'active'|'mimicry'
            status                  TEXT    NOT NULL DEFAULT 'running',
                                                        -- 'running'|'completed'|'interrupted'|'estopped'
            estop_count             INTEGER NOT NULL DEFAULT 0,

            -- ROM targets set by therapist before session
            hip_ab_target_min       REAL,
            hip_ab_target_max       REAL,
            hip_flex_target_min     REAL,
            hip_flex_target_max     REAL,
            knee_flex_target_min    REAL,
            knee_flex_target_max    REAL,

            -- Actual ROM achieved (written at session end)
            hip_ab_achieved         REAL,
            hip_flex_achieved       REAL,
            knee_flex_achieved      REAL,

            speed_deg_per_sec       REAL,
            repetitions_target      INTEGER,
            repetitions_done        INTEGER NOT NULL DEFAULT 0,
            session_notes           TEXT
        );

        -- ── SESSION LOGS ─────────────────────────────────────────────────────
        -- High-frequency data. Written at ~10 Hz during active therapy.
        CREATE TABLE IF NOT EXISTS session_logs (
            id                  INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id          INTEGER NOT NULL REFERENCES sessions(id),
            timestamp_ms        REAL    NOT NULL,   -- ms since session start_time
            fsr_hip             REAL,               -- raw ADC 0–1023
            fsr_knee            REAL,
            emg_hip             REAL,               -- raw ADC 0–1023
            emg_knee            REAL,
            emg_hip_rms         REAL,               -- computed RMS over window
            emg_knee_rms        REAL,
            ax1_angle           REAL,               -- degrees, hip Ab/Ad
            ax2_angle           REAL,               -- degrees, hip Flex/Ext
            ax3_angle           REAL,               -- degrees, knee Flex/Ext
            ax4_angle           REAL,               -- degrees, 4th axis
            ax1_speed           REAL,               -- deg/sec
            ax2_speed           REAL,
            ax3_speed           REAL,
            ax4_speed           REAL,
            patient_effort_pct  REAL,               -- 0.0–100.0
            robot_effort_pct    REAL,               -- 0.0–100.0
            assist_state        TEXT                -- 'assisting'|'holding'|'idle'
        );
        CREATE INDEX IF NOT EXISTS idx_session_logs_session_id
            ON session_logs(session_id);

        -- ── ROM ASSESSMENTS ───────────────────────────────────────────────────
        -- Discrete clinical measurements, separate from continuous logs.
        CREATE TABLE IF NOT EXISTS rom_assessments (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id      INTEGER NOT NULL REFERENCES sessions(id),
            patient_id      INTEGER NOT NULL REFERENCES patients(id),
            joint_name      TEXT    NOT NULL,  -- 'hip_ab_ad'|'hip_flex_ext'|'knee_flex_ext'
            passive_rom_deg REAL,              -- ROM achieved with robot (passive)
            active_rom_deg  REAL,              -- ROM patient achieves unassisted
            pain_score      REAL,              -- 0–10 VAS scale
            assessed_at     TEXT    NOT NULL DEFAULT (datetime('now')),
            notes           TEXT
        );

        -- ── THERAPY PATHS ─────────────────────────────────────────────────────
        -- Named collections of keyframes created in Mimicry mode.
        CREATE TABLE IF NOT EXISTS therapy_paths (
            id                  INTEGER PRIMARY KEY AUTOINCREMENT,
            patient_id          INTEGER NOT NULL REFERENCES patients(id),
            created_by          INTEGER REFERENCES therapists(id) ON DELETE SET NULL,
            name                TEXT    NOT NULL,
            target_joints       TEXT    NOT NULL,  -- e.g. 'hip_flex,knee_flex'
            speed_deg_per_sec   REAL    NOT NULL DEFAULT 5.0,
            loop_count          INTEGER NOT NULL DEFAULT 1,  -- 0 = loop forever
            created_at          TEXT    NOT NULL DEFAULT (datetime('now')),
            is_active           INTEGER NOT NULL DEFAULT 1
        );

        -- ── KEYFRAMES ─────────────────────────────────────────────────────────
        CREATE TABLE IF NOT EXISTS keyframes (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            path_id         INTEGER NOT NULL REFERENCES therapy_paths(id) ON DELETE CASCADE,
            order_index     INTEGER NOT NULL,
            ax1_angle       REAL    NOT NULL DEFAULT 0.0,
            ax2_angle       REAL    NOT NULL DEFAULT 0.0,
            ax3_angle       REAL    NOT NULL DEFAULT 0.0,
            ax4_angle       REAL    NOT NULL DEFAULT 0.0,
            hold_time_ms    REAL    NOT NULL DEFAULT 0.0,   -- pause at this frame
            interp_speed    REAL                            -- override path speed for this segment
        );

        -- ── ESTOP EVENTS ──────────────────────────────────────────────────────
        -- Every E-stop must be logged. Required for clinical safety audit.
        CREATE TABLE IF NOT EXISTS estop_events (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id      INTEGER REFERENCES sessions(id) ON DELETE SET NULL,
            triggered_at    TEXT    NOT NULL DEFAULT (datetime('now')),
            trigger_source  TEXT    NOT NULL,  -- 'wireless_button'|'heartbeat_loss'|'limit_switch'|'software'
            trigger_reason  TEXT,              -- human-readable description
            ax1_at_stop     REAL,              -- motor positions at moment of stop
            ax2_at_stop     REAL,
            ax3_at_stop     REAL,
            ax4_at_stop     REAL,
            cleared_at      TEXT,
            cleared_by      TEXT               -- therapist name or 'auto'
        );

        -- ── MOTOR CALIBRATION ─────────────────────────────────────────────────
        -- Home position and zero offsets. One row is marked is_current = 1.
        CREATE TABLE IF NOT EXISTS motor_calibration (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            calibrated_at   TEXT    NOT NULL DEFAULT (datetime('now')),
            calibrated_by   INTEGER REFERENCES therapists(id) ON DELETE SET NULL,
            ax1_home_steps  REAL    NOT NULL DEFAULT 0.0,
            ax2_home_steps  REAL    NOT NULL DEFAULT 0.0,
            ax3_home_steps  REAL    NOT NULL DEFAULT 0.0,
            ax4_home_steps  REAL    NOT NULL DEFAULT 0.0,
            ax1_zero_offset REAL    NOT NULL DEFAULT 0.0,  -- mechanical alignment correction
            ax2_zero_offset REAL    NOT NULL DEFAULT 0.0,
            ax3_zero_offset REAL    NOT NULL DEFAULT 0.0,
            ax4_zero_offset REAL    NOT NULL DEFAULT 0.0,
            is_current      INTEGER NOT NULL DEFAULT 0      -- only one row = 1 at a time
        );

        -- ── SYSTEM EVENTS ─────────────────────────────────────────────────────
        -- Startup, shutdown, serial errors, Wi-Fi drops — full audit trail.
        CREATE TABLE IF NOT EXISTS system_events (
            id              INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp       TEXT    NOT NULL DEFAULT (datetime('now')),
            event_type      TEXT    NOT NULL,  -- 'startup'|'shutdown'|'serial_error'|
                                               -- 'wifi_lost'|'calibration'|'config_change'
            severity        TEXT    NOT NULL DEFAULT 'info',  -- 'info'|'warning'|'error'|'critical'
            message         TEXT    NOT NULL,
            source_module   TEXT               -- e.g. 'serial_comm'|'safety_monitor'
        );

        """)
        logger.info("Database schema initialized.")


# ─────────────────────────────────────────────
# THERAPISTS
# ─────────────────────────────────────────────

def hash_pin(pin: str) -> str:
    return hashlib.sha256(pin.encode()).hexdigest()

def create_therapist(first_name, last_name, pin, role='therapist'):
    with get_db() as conn:
        cur = conn.execute(
            "INSERT INTO therapists (first_name, last_name, pin_hash, role) VALUES (?,?,?,?)",
            (first_name, last_name, hash_pin(pin), role)
        )
        return cur.lastrowid

def verify_therapist_pin(therapist_id, pin):
    with get_db() as conn:
        row = conn.execute(
            "SELECT pin_hash FROM therapists WHERE id=?", (therapist_id,)
        ).fetchone()
        return row and row['pin_hash'] == hash_pin(pin)

def get_all_therapists():
    with get_db() as conn:
        return conn.execute(
            "SELECT id, first_name, last_name, role FROM therapists"
        ).fetchall()


# ─────────────────────────────────────────────
# PATIENTS
# ─────────────────────────────────────────────

def create_patient(first_name, last_name, date_of_birth, gender,
                   affected_side, weight_kg, height_cm,
                   diagnosis, contact_phone=None, notes=None, therapist_id=None):
    with get_db() as conn:
        cur = conn.execute("""
            INSERT INTO patients
                (therapist_id, first_name, last_name, date_of_birth, gender,
                 affected_side, weight_kg, height_cm, diagnosis, contact_phone, notes)
            VALUES (?,?,?,?,?,?,?,?,?,?,?)
        """, (therapist_id, first_name, last_name, date_of_birth, gender,
              affected_side, weight_kg, height_cm, diagnosis, contact_phone, notes))
        return cur.lastrowid

def get_patient(patient_id):
    with get_db() as conn:
        return conn.execute(
            "SELECT * FROM patients WHERE id=?", (patient_id,)
        ).fetchone()

def get_all_patients(active_only=True):
    with get_db() as conn:
        q = "SELECT * FROM patients"
        q += " WHERE is_active=1" if active_only else ""
        return conn.execute(q).fetchall()

def update_patient_notes(patient_id, notes):
    with get_db() as conn:
        conn.execute(
            "UPDATE patients SET notes=? WHERE id=?", (notes, patient_id)
        )


def update_patient(patient_id, first_name, last_name, date_of_birth,
                   gender, affected_side, weight_kg, height_cm,
                   diagnosis, contact_phone=None, notes=None):
    """Update all editable fields of a patient record."""
    with get_db() as conn:
        conn.execute("""
            UPDATE patients SET
                first_name    = ?,
                last_name     = ?,
                date_of_birth = ?,
                gender        = ?,
                affected_side = ?,
                weight_kg     = ?,
                height_cm     = ?,
                diagnosis     = ?,
                contact_phone = ?,
                notes         = ?
            WHERE id = ?
        """, (first_name, last_name, date_of_birth, gender, affected_side,
              weight_kg, height_cm, diagnosis, contact_phone, notes,
              patient_id))

def archive_patient(patient_id):
    with get_db() as conn:
        conn.execute(
            "UPDATE patients SET is_active=0 WHERE id=?", (patient_id,)
        )


# ─────────────────────────────────────────────
# SESSIONS
# ─────────────────────────────────────────────

def create_session(patient_id, therapist_id, mode,
                  hip_ab_min=None, hip_ab_max=None,
                  hip_flex_min=None, hip_flex_max=None,
                  knee_flex_min=None, knee_flex_max=None,
                  speed=None, reps_target=None):
    with get_db() as conn:
        cur = conn.execute("""
            INSERT INTO sessions
                (patient_id, therapist_id, mode,
                 hip_ab_target_min, hip_ab_target_max,
                 hip_flex_target_min, hip_flex_target_max,
                 knee_flex_target_min, knee_flex_target_max,
                 speed_deg_per_sec, repetitions_target)
            VALUES (?,?,?,?,?,?,?,?,?,?,?)
        """, (patient_id, therapist_id, mode,
              hip_ab_min, hip_ab_max,
              hip_flex_min, hip_flex_max,
              knee_flex_min, knee_flex_max,
              speed, reps_target))
        log_system_event('startup', 'info',
            f"Session {cur.lastrowid} started. Patient {patient_id}, mode={mode}",
            'session_manager')
        return cur.lastrowid

def end_session(session_id, status='completed',
                hip_ab_achieved=None, hip_flex_achieved=None,
                knee_flex_achieved=None, reps_done=None, notes=None):
    with get_db() as conn:
        conn.execute("""
            UPDATE sessions SET
                end_time=datetime('now'),
                status=?,
                hip_ab_achieved=?,
                hip_flex_achieved=?,
                knee_flex_achieved=?,
                repetitions_done=COALESCE(?, repetitions_done),
                session_notes=COALESCE(?, session_notes)
            WHERE id=?
        """, (status, hip_ab_achieved, hip_flex_achieved,
              knee_flex_achieved, reps_done, notes, session_id))

def get_session(session_id):
    with get_db() as conn:
        return conn.execute(
            "SELECT * FROM sessions WHERE id=?", (session_id,)
        ).fetchone()

def get_patient_sessions(patient_id):
    with get_db() as conn:
        return conn.execute(
            "SELECT * FROM sessions WHERE patient_id=? ORDER BY start_time DESC",
            (patient_id,)
        ).fetchall()

def increment_estop_count(session_id):
    with get_db() as conn:
        conn.execute(
            "UPDATE sessions SET estop_count = estop_count + 1 WHERE id=?",
            (session_id,)
        )


# ─────────────────────────────────────────────
# SESSION LOGS
# ─────────────────────────────────────────────

def log_sensor_sample(session_id, timestamp_ms,
                      fsr_hip, fsr_knee, emg_hip, emg_knee,
                      emg_hip_rms, emg_knee_rms,
                      ax1, ax2, ax3, ax4,
                      ax1_spd, ax2_spd, ax3_spd, ax4_spd,
                      patient_pct, robot_pct, assist_state):
    with get_db() as conn:
        conn.execute("""
            INSERT INTO session_logs
                (session_id, timestamp_ms,
                 fsr_hip, fsr_knee, emg_hip, emg_knee,
                 emg_hip_rms, emg_knee_rms,
                 ax1_angle, ax2_angle, ax3_angle, ax4_angle,
                 ax1_speed, ax2_speed, ax3_speed, ax4_speed,
                 patient_effort_pct, robot_effort_pct, assist_state)
            VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
        """, (session_id, timestamp_ms,
              fsr_hip, fsr_knee, emg_hip, emg_knee,
              emg_hip_rms, emg_knee_rms,
              ax1, ax2, ax3, ax4,
              ax1_spd, ax2_spd, ax3_spd, ax4_spd,
              patient_pct, robot_pct, assist_state))

def get_session_logs(session_id):
    with get_db() as conn:
        return conn.execute(
            "SELECT * FROM session_logs WHERE session_id=? ORDER BY timestamp_ms",
            (session_id,)
        ).fetchall()

def bulk_log_samples(rows: list[tuple]):
    """Insert multiple log rows in one transaction. Use this during live sessions."""
    with get_db() as conn:
        conn.executemany("""
            INSERT INTO session_logs
                (session_id, timestamp_ms,
                 fsr_hip, fsr_knee, emg_hip, emg_knee,
                 emg_hip_rms, emg_knee_rms,
                 ax1_angle, ax2_angle, ax3_angle, ax4_angle,
                 ax1_speed, ax2_speed, ax3_speed, ax4_speed,
                 patient_effort_pct, robot_effort_pct, assist_state)
            VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
        """, rows)


# ─────────────────────────────────────────────
# ROM ASSESSMENTS
# ─────────────────────────────────────────────

def save_rom_assessment(session_id, patient_id, joint_name,
                        passive_rom=None, active_rom=None,
                        pain_score=None, notes=None):
    with get_db() as conn:
        cur = conn.execute("""
            INSERT INTO rom_assessments
                (session_id, patient_id, joint_name,
                 passive_rom_deg, active_rom_deg, pain_score, notes)
            VALUES (?,?,?,?,?,?,?)
        """, (session_id, patient_id, joint_name,
              passive_rom, active_rom, pain_score, notes))
        return cur.lastrowid

def get_patient_rom_history(patient_id, joint_name=None):
    """Return all ROM assessments for a patient, optionally filtered by joint."""
    with get_db() as conn:
        if joint_name:
            return conn.execute("""
                SELECT r.*, s.start_time FROM rom_assessments r
                JOIN sessions s ON r.session_id = s.id
                WHERE r.patient_id=? AND r.joint_name=?
                ORDER BY r.assessed_at
            """, (patient_id, joint_name)).fetchall()
        return conn.execute("""
            SELECT r.*, s.start_time FROM rom_assessments r
            JOIN sessions s ON r.session_id = s.id
            WHERE r.patient_id=?
            ORDER BY r.assessed_at
        """, (patient_id,)).fetchall()


# ─────────────────────────────────────────────
# THERAPY PATHS & KEYFRAMES
# ─────────────────────────────────────────────

def create_therapy_path(patient_id, created_by, name,
                        target_joints, speed=5.0, loop_count=1):
    with get_db() as conn:
        cur = conn.execute("""
            INSERT INTO therapy_paths
                (patient_id, created_by, name, target_joints,
                 speed_deg_per_sec, loop_count)
            VALUES (?,?,?,?,?,?)
        """, (patient_id, created_by, name, target_joints, speed, loop_count))
        return cur.lastrowid

def get_patient_paths(patient_id, active_only=True):
    with get_db() as conn:
        q = "SELECT * FROM therapy_paths WHERE patient_id=?"
        params = [patient_id]
        if active_only:
            q += " AND is_active=1"
        return conn.execute(q, params).fetchall()

def save_keyframe(path_id, order_index, ax1, ax2, ax3, ax4,
                  hold_time_ms=0.0, interp_speed=None):
    with get_db() as conn:
        cur = conn.execute("""
            INSERT INTO keyframes
                (path_id, order_index, ax1_angle, ax2_angle,
                 ax3_angle, ax4_angle, hold_time_ms, interp_speed)
            VALUES (?,?,?,?,?,?,?,?)
        """, (path_id, order_index, ax1, ax2, ax3, ax4,
              hold_time_ms, interp_speed))
        return cur.lastrowid

def get_keyframes_for_path(path_id):
    with get_db() as conn:
        return conn.execute(
            "SELECT * FROM keyframes WHERE path_id=? ORDER BY order_index",
            (path_id,)
        ).fetchall()

def delete_path(path_id):
    """Soft-delete the path. Keyframes are CASCADE deleted."""
    with get_db() as conn:
        conn.execute(
            "UPDATE therapy_paths SET is_active=0 WHERE id=?", (path_id,)
        )


# ─────────────────────────────────────────────
# ESTOP EVENTS
# ─────────────────────────────────────────────

def log_estop_event(session_id, trigger_source, trigger_reason=None,
              ax1=None, ax2=None, ax3=None, ax4=None):
    with get_db() as conn:
        cur = conn.execute("""
            INSERT INTO estop_events
                (session_id, trigger_source, trigger_reason,
                 ax1_at_stop, ax2_at_stop, ax3_at_stop, ax4_at_stop)
            VALUES (?,?,?,?,?,?,?)
        """, (session_id, trigger_source, trigger_reason, ax1, ax2, ax3, ax4))
        if session_id:
            increment_estop_count(session_id)
        log_system_event('estop', 'critical',
            f"E-stop: source={trigger_source} reason={trigger_reason}",
            'safety_monitor')
        return cur.lastrowid

def clear_estop(estop_id, cleared_by='therapist'):
    with get_db() as conn:
        conn.execute("""
            UPDATE estop_events
            SET cleared_at=datetime('now'), cleared_by=?
            WHERE id=?
        """, (cleared_by, estop_id))

def get_recent_estops(limit=20):
    with get_db() as conn:
        return conn.execute(
            "SELECT * FROM estop_events ORDER BY triggered_at DESC LIMIT ?",
            (limit,)
        ).fetchall()


# ─────────────────────────────────────────────
# MOTOR CALIBRATION
# ─────────────────────────────────────────────

def save_calibration(calibrated_by,
                     ax1_home, ax2_home, ax3_home, ax4_home,
                     ax1_offset=0.0, ax2_offset=0.0,
                     ax3_offset=0.0, ax4_offset=0.0):
    with get_db() as conn:
        # Clear previous current calibration
        conn.execute("UPDATE motor_calibration SET is_current=0")
        cur = conn.execute("""
            INSERT INTO motor_calibration
                (calibrated_by,
                 ax1_home_steps, ax2_home_steps, ax3_home_steps, ax4_home_steps,
                 ax1_zero_offset, ax2_zero_offset, ax3_zero_offset, ax4_zero_offset,
                 is_current)
            VALUES (?,?,?,?,?,?,?,?,?,1)
        """, (calibrated_by, ax1_home, ax2_home, ax3_home, ax4_home,
              ax1_offset, ax2_offset, ax3_offset, ax4_offset))
        log_system_event('calibration', 'info',
            "Motor calibration saved.", 'motor_calibration')
        return cur.lastrowid

def get_current_calibration():
    with get_db() as conn:
        return conn.execute(
            "SELECT * FROM motor_calibration WHERE is_current=1"
        ).fetchone()


# ─────────────────────────────────────────────
# SYSTEM EVENTS
# ─────────────────────────────────────────────

def log_system_event(event_type, severity, message, source_module=None):
    try:
        with get_db() as conn:
            conn.execute("""
                INSERT INTO system_events
                    (event_type, severity, message, source_module)
                VALUES (?,?,?,?)
            """, (event_type, severity, message, source_module))
    except Exception as e:
        # System event logging must never crash the caller
        logger.error(f"Failed to log system event: {e}")

def get_system_events(severity=None, limit=100):
    with get_db() as conn:
        if severity:
            return conn.execute("""
                SELECT * FROM system_events WHERE severity=?
                ORDER BY timestamp DESC LIMIT ?
            """, (severity, limit)).fetchall()
        return conn.execute(
            "SELECT * FROM system_events ORDER BY timestamp DESC LIMIT ?",
            (limit,)
        ).fetchall()


# ─────────────────────────────────────────────
# ANALYTICS QUERIES
# ─────────────────────────────────────────────

def get_effort_trend(patient_id, last_n_sessions=10):
    """Return avg patient/robot effort % per session for progress graphing."""
    with get_db() as conn:
        return conn.execute("""
            SELECT
                s.id            AS session_id,
                s.start_time,
                s.mode,
                AVG(l.patient_effort_pct) AS avg_patient_effort,
                AVG(l.robot_effort_pct)   AS avg_robot_effort,
                MAX(l.ax2_angle)          AS max_hip_flex,
                MAX(l.ax3_angle)          AS max_knee_flex
            FROM sessions s
            JOIN session_logs l ON l.session_id = s.id
            WHERE s.patient_id=?
            GROUP BY s.id
            ORDER BY s.start_time DESC
            LIMIT ?
        """, (patient_id, last_n_sessions)).fetchall()

def get_rom_trend(patient_id, joint_name):
    """Return ROM progression over time for a specific joint."""
    with get_db() as conn:
        return conn.execute("""
            SELECT assessed_at, passive_rom_deg, active_rom_deg, pain_score
            FROM rom_assessments
            WHERE patient_id=? AND joint_name=?
            ORDER BY assessed_at
        """, (patient_id, joint_name)).fetchall()