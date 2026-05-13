# =============================================================================
#  web/routes/pages.py
#  /home/pi/rehab_robot/web/routes/pages.py
#
#  Page routes — serve HTML templates.
#  Each route renders a Jinja2 template from web/templates/.
# =============================================================================

from flask import Blueprint, render_template

pages_bp = Blueprint("pages", __name__)


@pages_bp.route("/")
def dashboard():
    return render_template("dashboard.html")


@pages_bp.route("/patients")
@pages_bp.route("/patients/<int:patient_id>")
def patients(patient_id=None):
    return render_template("patients.html", patient_id=patient_id)


@pages_bp.route("/therapy")
def therapy():
    return render_template("therapy.html")


@pages_bp.route("/analytics")
def analytics():
    return render_template("analytics.html")


@pages_bp.route("/settings")
def settings():
    return render_template("settings.html")