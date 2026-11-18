from flask import Flask, request, jsonify
import sqlite3
import math

app = Flask(__name__)
DB = "potholes.db"
RADIUS_M = 10        # reports closer than this = same pothole
RIDES_NEEDED = 3     # rides needed to confirm a pothole
NEARBY_M = 300       # how far ahead to warn the rider

def db():
    conn = sqlite3.connect(DB)
    conn.row_factory = sqlite3.Row
    return conn

with db() as conn:
    conn.execute("""
        CREATE TABLE IF NOT EXISTS reports (
            event_id TEXT PRIMARY KEY,
            ride_id TEXT,
            timestamp TEXT,
            lat REAL,
            lon REAL,
            gps_fix INTEGER,
            satellites INTEGER,
            evidence TEXT,
            range_delta_cm REAL,
            vision_confidence REAL
        )
    """)

# distance in meters between two GPS points
def distance_m(lat1, lon1, lat2, lon2):
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2 * R * math.asin(math.sqrt(a))

# group reports into potholes
def build_hazards():
    with db() as conn:
        rows = conn.execute(
            "SELECT * FROM reports WHERE gps_fix = 1 ORDER BY timestamp"
        ).fetchall()
    groups = []
    for r in rows:
        for g in groups:
            if distance_m(r["lat"], r["lon"], g["lat"], g["lon"]) <= RADIUS_M:
                g["reports"].append(r)
                break
        else:
            groups.append({"lat": r["lat"], "lon": r["lon"], "reports": [r]})

    hazards = []
    for i, g in enumerate(groups, start=1):
        rides = {r["ride_id"] for r in g["reports"]}
        hazards.append({
            "hazard_id": i,
            "lat": g["lat"],
            "lon": g["lon"],
            "ride_count": len(rides),
            "report_count": len(g["reports"]),
            "corroborated": len(rides) >= RIDES_NEEDED,
            "evidence": sorted({r["evidence"] for r in g["reports"]}),
            "last_seen": max(r["timestamp"] for r in g["reports"]),
        })
    return hazards

# confirmed potholes near a position
def nearby_hazards(lat, lon):
    return [h for h in build_hazards()
            if h["corroborated"]
            and distance_m(lat, lon, h["lat"], h["lon"]) <= NEARBY_M]

# the sensing TTGO sends a report here
@app.post("/reports")
def add_report():
    r = request.get_json()
    with db() as conn:
        cur = conn.execute(
            "INSERT OR IGNORE INTO reports VALUES (?,?,?,?,?,?,?,?,?,?)",
            (r["event_id"], r.get("ride_id"), r.get("timestamp"),
             r["lat"], r["lon"], r.get("gps_fix"), r.get("satellites"),
             r.get("evidence"), r.get("range_delta_cm"),
             r.get("vision_confidence")),
        )
        duplicate = cur.rowcount == 0
    return jsonify({
        "acked": [r["event_id"]],
        "duplicate": duplicate,
        "hazards": nearby_hazards(r["lat"], r["lon"]),
    })

# see every saved report
@app.get("/reports")
def list_reports():
    with db() as conn:
        rows = conn.execute("SELECT * FROM reports").fetchall()
    return jsonify([dict(row) for row in rows])

# see all potholes (or only nearby confirmed ones with ?lat=..&lon=..)
@app.get("/hazards")
def get_hazards():
    lat = request.args.get("lat", type=float)
    lon = request.args.get("lon", type=float)
    if lat is not None and lon is not None:
        return jsonify(nearby_hazards(lat, lon))
    return jsonify(build_hazards())

if __name__ == "__main__":
    app.run(debug=True, port=8000)

