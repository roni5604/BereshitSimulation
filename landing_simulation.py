#!/usr/bin/env python3
"""
2D Lunar Landing Simulation – Dual PID for Vertical & Horizontal Control
----------------------------------------------------------------------------
This simulation models a lunar landing with two PID controllers:
  - One for vertical speed (vs) controlling the engine throttle.
  - One for horizontal speed (hs) controlling the spacecraft's angle.
  
The simulation starts from a high altitude and far left (negative horizontal position)
and must decelerate its horizontal speed to land safely above the target (x=0).

Outputs:
  - A live simulation window.
  - A CSV file (and an Excel file) containing full simulation data.
  
Dependencies: numpy, matplotlib, pandas, openpyxl
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from matplotlib.patches import Polygon, Circle
import os
import pandas as pd

# ===================== GLOBAL CONSTANTS =====================
# Spacecraft physical parameters
WEIGHT_EMP    = 165.0         # kg (empty weight)
FUEL_INIT     = 120.0         # liters (initial fuel)
MAIN_ENG_F    = 430.0         # Main engine thrust (N)
SECOND_ENG_F  = 25.0          # Secondary engine thrust (N)
MAIN_BURN     = 0.15          # Main engine fuel burn rate (L/s)
SECOND_BURN   = 0.009         # Secondary engine fuel burn rate (L/s)
ALL_BURN      = MAIN_BURN + 8 * SECOND_BURN

MOON_ACC      = 1.622         # m/s² (lunar gravity)

DT            = 1.0           # seconds (time step)

# Drawing scales (convert from meters to drawing units)
SCALE_X       = 1/10000.0     # horizontal scale
SCALE_Y       = 1/1000.0      # vertical scale

# ===================== PID CONTROLLER CLASS =====================
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, integral_limit=100):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit

    def update(self, current, dt):
        error = current - self.setpoint
        if abs(error) < 0.01:  # סף קרוב לאפס - לא נזיז יותר
            return 0.0
        self.integral += error * dt
        # Anti-windup: clamp the integral term
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

# ===================== DESIRED SPEED FUNCTIONS =====================
def desired_vs(alt, initial_alt):
    """
    Desired vertical speed (m/s). We linearly scale from 25 m/s at the initial altitude
    to a low value (3 m/s) near the ground.
    """
    vs_max = 25.0
    vs_min = 3.0 ############
    ratio = alt / initial_alt  # ratio: 1 at start, 0 at ground
    return vs_min + (vs_max - vs_min) * ratio

def desired_hs(alt, initial_alt, hs_initial=932.0):
    """
    Desired horizontal speed (m/s). We linearly reduce from hs_initial at the initial altitude
    to 0 m/s when alt <= 500 m.
    """
    if alt <= 1500:
        return 0.0
    return hs_initial * (alt / initial_alt)

# ===================== SPACECRAFT CLASS =====================
class Spacecraft:
    def __init__(self):
        # Starting state:
        # Horizontal distance: far left (negative, in meters). Target is x=0.
        self.dist = -181000.0    # m
        self.alt  = 13748.0      # m altitude
        self.vs   = 24.8         # m/s vertical speed (descent)
        self.hs   = 932.0        # m/s horizontal speed (to the right)
        self.ang  = 58.3         # degrees; 0° means vertical thrust (for pure deceleration)
        self.fuel = FUEL_INIT    # liters
        self.weight = WEIGHT_EMP + self.fuel
        self.NN   = 0.7          # throttle (0 to 1)
        self.acc  = 0.0          # engine acceleration magnitude (m/s²)
        self.landed = False
        self.time = 0
        self.initial_alt = self.alt

    def update_physics(self):
        """
        Update spacecraft physics:
          - Vertical speed (vs) increases due to gravity and is reduced by engine thrust.
          - Horizontal speed (hs) is decreased by the horizontal component of engine thrust.
          - Altitude and horizontal distance are updated accordingly.
        """
        self.weight = WEIGHT_EMP + self.fuel
        thrust_total = MAIN_ENG_F + 8 * SECOND_ENG_F
        acc_max = thrust_total / self.weight if self.weight > 0 else 0.0
        engine_acc = self.NN * acc_max

        # Convert angle to radians (angle measured from vertical upward)
        ang_rad = math.radians(self.ang)
        # Thrust components:
        a_thrust_vertical = engine_acc * math.cos(ang_rad)
        a_thrust_horizontal = engine_acc * math.sin(ang_rad)

        # Update vertical speed: gravity increases descent speed, engine thrust counteracts it.
        self.vs = max(0, self.vs + (MOON_ACC - a_thrust_vertical) * DT)
        # Update horizontal speed: subtract horizontal component of thrust.
        # נוסיף כפול (לדוגמה 1.5) אם hs גדול מהמטרה, כדי להאיץ את הבלימה.
        factor = 1.9 if self.hs > desired_hs(self.alt, self.initial_alt) else 1.0
        self.hs = max(0, self.hs - factor * a_thrust_horizontal * DT)

        # Update positions:
        self.alt = max(0, self.alt - self.vs * DT)
        self.dist += self.hs * DT

        # Fuel consumption:
        fuel_used = ALL_BURN * self.NN * DT
        self.fuel = max(0, self.fuel - fuel_used)

        self.time += DT

        if self.alt <= 0:
            self.alt = 0
            self.vs = 0
            self.hs = 0
            self.landed = True

    def get_state(self):
        return {
            "time": self.time,
            "alt": self.alt,        # Is it alt or initial alt ??
            "vs": self.vs,
            "hs": self.hs,
            "dist": self.dist,
            "ang": self.ang,
            "fuel": self.fuel,
            "NN": self.NN,
            "acc": self.acc,
            "landed": self.landed
        }

def update_angle_by_hs(ship):
    """
    משנה את הזווית של החללית ב־5 שלבים לפי מהירות אופקית (hs)
    בכל שלב מקטין את הזווית יותר עד שמגיע ל-0
    """
    hs = ship.hs
    vs = ship.vs
    alt = ship.alt
    # הגדרת יעד זווית לפי מהירות אופקית
    target_ang = 58.3

    if hs > 0.1:
        target_ang = 50.5
    else:
        target_ang = 0

    # שינוי הדרגתי של הזווית כלפי יעד
    angle_change_rate = 1.0  # מעלות לשנייה
    ang_diff = target_ang - ship.ang
    max_delta = angle_change_rate * DT
    # הגבלה שלא נחרוג מקצב השינוי
    if abs(ang_diff) > max_delta:
        ang_diff = math.copysign(max_delta, ang_diff)

    ship.ang += ang_diff
    # נוודא שלא עוברים את הגבולות
    ship.ang = max(-60, min(ship.ang, 60))




# ===================== GLOBAL VARIABLES & DATA STORAGE =====================
ship = Spacecraft()
# PID for vertical speed control (affects throttle)
vs_pid = PID(Kp=0.2, Ki=0.01, Kd=0.002, integral_limit=100)  # PID(Kp=0.2, Ki=0.01, Kd=0.8)
# PID for horizontal speed control (affects angle)
hs_pid = PID(Kp=0.005, Ki=0.0, Kd=0.01, integral_limit=50) # hs_pid = PID(Kp=0.005, Ki=0.0, Kd=0.01)


running = False

time_data = []
alt_data  = []
vs_data   = []
hs_data   = []
ang_data  = []
fuel_data = []
nn_data   = []

def reset_data():
    time_data.clear()
    alt_data.clear()
    vs_data.clear()
    hs_data.clear()
    ang_data.clear()
    fuel_data.clear()
    nn_data.clear()

# ===================== PLOTTING & VISUALIZATION SETUP =====================
fig_sim = plt.figure("Lunar Landing Simulation", figsize=(12,8))
ax_sim = fig_sim.add_subplot(111)
ax_sim.axis("off")
ax_sim.set_xlim(-20,20)
ax_sim.set_ylim(-5,35)

# Load background image if available
if os.path.exists("space.jpg"):
    bg = plt.imread("space.jpg")
    ax_sim.imshow(bg, extent=[-20,20,-5,35], aspect="auto", zorder=-1)
else:
    print("Background image 'space.jpg' not found.")

# Draw Moon as a circle at bottom center
moon_center = (0, -10)
moon_radius = 10
moon_patch = Circle(moon_center, moon_radius, color="dimgray", zorder=0)
ax_sim.add_patch(moon_patch)

# Draw landing flag (only left flag)
ax_sim.plot([-3, -3], [0, 1.5], color="white", lw=3, zorder=1)
ax_sim.plot([-3, -1.5], [1.5, 1.2], color="red", lw=3, zorder=1)

# ===================== DRAWING THE SPACECRAFT =====================
body_width = 0.5
body_height = 2.5
body_coords = np.array([
    [-body_width/2, -body_height/2],
    [ body_width/2, -body_height/2],
    [ body_width/2,  body_height/2],
    [-body_width/2,  body_height/2]
])
left_leg_local = np.array([
    [-body_width/2, -body_height/2],
    [-body_width, -body_height/2 - 0.7]
])
right_leg_local = np.array([
    [ body_width/2, -body_height/2],
    [ body_width, -body_height/2 - 0.7]
])
engine_radius = 0.1
left_engine_center = np.array([-body_width/2, -body_height/2])
right_engine_center = np.array([body_width/2, -body_height/2])
flame_local = np.array([
    [-0.15, -body_height/2],
    [ 0.0,  -body_height/2 - 0.8],
    [ 0.15, -body_height/2]
])

ship_body_patch = Polygon(body_coords, closed=True, fc="silver", ec="white", lw=2, zorder=10)
ax_sim.add_patch(ship_body_patch)
left_leg_line, = ax_sim.plot([], [], color="white", lw=2, zorder=9)
right_leg_line, = ax_sim.plot([], [], color="white", lw=2, zorder=9)
left_engine_patch = Circle((0,0), engine_radius, fc="gray", ec="white", lw=1.5, zorder=11)
right_engine_patch = Circle((0,0), engine_radius, fc="gray", ec="white", lw=1.5, zorder=11)
ax_sim.add_patch(left_engine_patch)
ax_sim.add_patch(right_engine_patch)
flame_patch = Polygon(flame_local, closed=True, fc="orange", ec="red", lw=1.0, alpha=0.0, zorder=8)
ax_sim.add_patch(flame_patch)

# Flight path line
path_line, = ax_sim.plot([], [], "cyan", linestyle="--", lw=2, zorder=4)
path_x = []
path_y = []

# Info text overlay
info_text = ax_sim.text(0.02, 0.95, "", transform=ax_sim.transAxes, fontsize=12,
                        color="white", verticalalignment="top",
                        bbox=dict(boxstyle="round", facecolor="black", alpha=0.5))

# ===================== START/RESET BUTTON =====================
start_ax = fig_sim.add_axes([0.4, 0.02, 0.2, 0.06])
start_button = Button(start_ax, "Start/Reset", color="limegreen", hovercolor="green")

def start_sim(event):
    global running, ship
    running = True
    ship.__init__()
    vs_pid.reset()
    hs_pid.reset()
    reset_data()
    path_x.clear()
    path_y.clear()

start_button.on_clicked(start_sim)

# ===================== UTILITY: TRANSFORM POINTS =====================
def transform_points(points, angle_deg, translation):
    theta = -math.radians(angle_deg)
    rot_matrix = np.array([[math.cos(theta), -math.sin(theta)],
                           [math.sin(theta),  math.cos(theta)]])
    return (points @ rot_matrix.T) + translation

# ===================== MAIN UPDATE FUNCTION =====================
def update(frame):
    global running
    if not running:
        return (ship_body_patch, left_leg_line, right_leg_line,
                left_engine_patch, right_engine_patch, flame_patch, path_line, info_text)

    state = ship.get_state()
    # Stop simulation if landed
    if state["landed"]:
        return (ship_body_patch, left_leg_line, right_leg_line,
                left_engine_patch, right_engine_patch, flame_patch, path_line, info_text)

    # ========= DYNAMIC CONTROL LOGIC =========
    # Compute desired speeds based on current altitude:
    # vs_target = desired_vs(state["alt"], ship.initial_alt)
    # hs_target = desired_hs(state["alt"], ship.initial_alt, hs_initial=932.0)
    vs_target = desired_vs(ship.alt, ship.initial_alt)
    hs_target = desired_hs(ship.alt, ship.initial_alt, hs_initial=932.0)
    # ======= New desired speed logic =======

    # --- Vertical PID: adjust throttle to reduce vs ---
    vs_pid.setpoint = vs_target
    out_vs = vs_pid.update(state["vs"], DT)
    ship.NN = max(0, min(ship.NN + out_vs, 1))

    # --- Horizontal PID: adjust angle to reduce hs ---
    hs_pid.setpoint = hs_target
    out_hs = hs_pid.update(state["hs"], DT)

    # זווית
    update_angle_by_hs(ship)

    if ship.hs < 1.0:
        # אנחנו בשלב האחרון של הנחיתה
        landing_pid = PID(0.3, 0.0, 0.5, setpoint=10.0, integral_limit=50)
        vs_target = 10 if ship.alt > 80 else 0.5  # יעד למהירות הנחיתה
        landing_pid.setpoint = vs_target
        out_vs = landing_pid.update(ship.vs, DT)




    # ========= UPDATE PHYSICS =========
    ship.update_physics()
    st = ship.get_state()

    # Log simulation data
    time_data.append(st["time"])
    alt_data.append(st["alt"])
    vs_data.append(st["vs"])
    hs_data.append(st["hs"])
    ang_data.append(st["ang"])
    fuel_data.append(st["fuel"])
    nn_data.append(st["NN"])

    # ========= DRAWING =========
    draw_x = st["dist"] * SCALE_X
    draw_y = st["alt"] * SCALE_Y
    draw_x = max(-20, min(draw_x, 20))
    draw_y = max(0, min(draw_y, 35))
    path_x.append(draw_x)
    path_y.append(draw_y)
    path_line.set_data(path_x, path_y)

    new_body_coords = transform_points(body_coords, ship.ang, np.array([draw_x, draw_y]))
    ship_body_patch.set_xy(new_body_coords)
    left_leg_coords = transform_points(left_leg_local, ship.ang, np.array([draw_x, draw_y]))
    right_leg_coords = transform_points(right_leg_local, ship.ang, np.array([draw_x, draw_y]))
    left_leg_line.set_data(left_leg_coords[:, 0], left_leg_coords[:, 1])
    right_leg_line.set_data(right_leg_coords[:, 0], right_leg_coords[:, 1])
    le_center = transform_points(np.array([left_engine_center]), ship.ang, np.array([draw_x, draw_y]))[0]
    re_center = transform_points(np.array([right_engine_center]), ship.ang, np.array([draw_x, draw_y]))[0]
    left_engine_patch.center = (le_center[0], le_center[1])
    right_engine_patch.center = (re_center[0], re_center[1])
    flame_scaled = flame_local.copy()
    flame_scaled[1, 1] = flame_local[1, 1] * (1 + ship.NN)
    new_flame = transform_points(flame_scaled, ship.ang, np.array([draw_x, draw_y]))
    flame_patch.set_xy(new_flame)
    flame_patch.set_alpha(0.8 if ship.NN > 0.1 else 0.0)

    # ========= UPDATE INFO TEXT =========
    info_str = (f"Time: {st['time']:.1f} s\n"
                f"Altitude: {st['alt']:.2f} m\n"
                f"VS: {st['vs']:.2f} m/s (target: {vs_target:.1f})\n"
                f"HS: {st['hs']:.2f} m/s (target: {hs_target:.1f})\n"
                f"Angle: {st['ang']:.1f}°\n"
                f"Throttle: {st['NN']:.2f}\n"
                f"Fuel: {st['fuel']:.2f} L\n"
                f"Distance: {st['dist']:.1f} m")
    if st["fuel"] < 50:
        info_text.set_color("red")
    else:
        info_text.set_color("white")
    info_text.set_text(info_str)

    # ========= LANDING CHECK =========
    if st["alt"] <= 0.5:
        running = False
        if st["vs"] < 2.5 and st["hs"] < 2.5 and abs(st["ang"]) < 5:
            info_text.set_text(info_text.get_text() + "\n*** LANDING SUCCESS ***")
        else:
            info_text.set_text(info_text.get_text() + "\n*** CRASH ***")
        plt.pause(1)
        show_full_data_table()

    return (ship_body_patch, left_leg_line, right_leg_line,
            left_engine_patch, right_engine_patch, flame_patch, path_line, info_text)

# ===================== DATA OUTPUT: CSV & Excel =====================
def show_full_data_table():
    dhs = [0]
    dvs = [0]
    for i in range(1, len(hs_data)):
        dhs.append(hs_data[i] - hs_data[i-1])
        dvs.append(vs_data[i] - vs_data[i-1])
    df = pd.DataFrame({
        "Time (s)": time_data,
        "Altitude (m)": alt_data,
        "Vertical Speed (m/s)": vs_data,
        "Horizontal Speed (m/s)": hs_data,
        "Angle (°)": ang_data,
        "Fuel (L)": fuel_data,
        "Throttle": nn_data,
        "dhs": dhs,
        "dvs": dvs
    })
    # Create output directory if it doesn't exist
    if not os.path.exists("output"):
        os.makedirs("output")
    # Save both Excel and CSV
    df.to_excel("output/full_sim_data.xlsx", index=False)
    df.to_csv("output/full_sim_data.csv", index=False)

    fig_tbl = plt.figure("Full Simulation Data", figsize=(12,8))
    ax_tbl = fig_tbl.add_subplot(111)
    ax_tbl.axis("off")
    table_str = df.to_string(index=False)
    ax_tbl.text(0.01, 0.99, table_str, transform=ax_tbl.transAxes,
                fontsize=8, verticalalignment="top", family="monospace")
    plt.show()

# ===================== ANIMATION SETUP =====================
ani = animation.FuncAnimation(fig_sim, update, interval=150, blit=True)
plt.show()


# Close the figure when done