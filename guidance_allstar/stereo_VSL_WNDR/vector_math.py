import math
import guidance_config as cfg
def vector_cross_product(ax, ay, az, bx, by, bz):
    """Returns C = A x B."""
    return (ay * bz - az * by,
            az * bx - ax * bz,
            ax * by - ay * bx)

def accel_to_vel(ax,ay,az,vx_prev,vy_prev,vz_prev,dt):
    """Convert acceleration to velocity using Euler integration."""
    vx = vx_prev + ax * dt
    vy = vy_prev + ay * dt
    vz = vz_prev + az * dt
    return vx, vy, vz

def accel_to_euler_ef(ax, ay, az, yaw_cmd, vx, vy):
    """
    Convert a desired NED acceleration vector into Earth-Frame 
    roll, pitch, and yaw-rate commands.
    
    Parameters
    ----------
    ax, ay, az : float
        Desired NED acceleration (m/s^2).
    yaw_cmd : float
        Current commanded yaw heading (radians).
    vx, vy : float
        Current horizontal NED velocity (m/s).
        
    Returns
    -------
    roll_ef, pitch_ef, yaw_rate_ef, thrust_norm
    """
    # 1. Total specific force required in NED (add gravity to Z axis)
    # NED gravity points DOWN (+Z), thrust must counter it (-Z)
    fx = ax
    fy = ay
    fz = az - 9.81  # Net required acceleration pushes us UP (negative in NED)

    # Required thrust magnitude (m/s^2)
    thrust_req = math.sqrt(fx**2 + fy**2 + fz**2)
    
    # Thrust normalized (0 to 1 range, assuming max physical accel is e.g. 20m/s^2)
    # This scaling will depend on the drone's actual thrust-to-weight.
    # We will pass raw thrust_req and let the sender normalize it.
    
    # 2. Desired Body Z axis (points opposite to thrust)
    # Since thrust f is the force the copter MUST exert, and thrust always comes 
    # out the bottom (Body +Z), the copter's Body +Z axis must point along -f.
    if thrust_req < 1e-3:
        z_bx, z_by, z_bz = 0.0, 0.0, 1.0
    else:
        z_bx = -fx / thrust_req
        z_by = -fy / thrust_req
        z_bz = -fz / thrust_req

    # 3. Horizontal heading vector V_h
    vh_x = math.cos(yaw_cmd)
    vh_y = math.sin(yaw_cmd)
    vh_z = 0.0

    # 4. Desired Right Wing vector (Y_body) = Z_body x V_h
    # This ensures the right wing stays horizontal (coordinated turn).
    y_bx, y_by, y_bz = vector_cross_product(z_bx, z_by, z_bz, vh_x, vh_y, vh_z)
    y_mag = math.sqrt(y_bx**2 + y_by**2 + y_bz**2)
    
    if y_mag < 1e-3: # Hover logic if heading perfectly matches thrust
        y_bx, y_by, y_bz = -vh_y, vh_x, 0.0
    else:
        y_bx /= y_mag
        y_by /= y_mag
        y_bz /= y_mag

    # 5. Desired Forward vector (X_body) = Y_body x Z_body
    x_bx, x_by, x_bz = vector_cross_product(y_bx, y_by, y_bz, z_bx, z_by, z_bz)

    # 6. Extract Earth-Frame Roll and Pitch from the Rotation Matrix
    # R = [X_b, Y_b, Z_b]
    # R31 = -sin(theta)  -> theta = asin(-R31) = asin(-x_bz)
    # R32 = sin(phi)cos(theta), R33 = cos(phi)cos(theta) -> phi = atan2(y_bz, z_bz)
    
    pitch_ef = math.asin(max(-1.0, min(1.0, -x_bz)))
    roll_ef  = math.atan2(y_bz, z_bz)

    # 7. Calculate Yaw Rate (turn rate of velocity vector)
    # Use a small epsilon for stability
    v_horiz_sq = vx**2 + vy**2
    if v_horiz_sq < 0.5: # Increased threshold for reliable yaw rate calculation
        yaw_rate_ef = 0.0
    else:
        # Standard coordinated turn rate: yaw_rate = (vx*ay - vy*ax) / (vx^2 + vy^2)
        yaw_rate_ef = (vx * ay - vy * ax) / v_horiz_sq
        
    # Clamp yaw rate to a reasonable physical limit (e.g., 60 deg/s)
    max_yaw_rate = getattr(cfg, 'MAX_OMEGA', 2.0) 
    yaw_rate_ef = max(-max_yaw_rate, min(max_yaw_rate, yaw_rate_ef))
    
    # Map Earth-Frame yaw rate (psi_dot) to Body-Frame yaw rate (r)
    # r = psi_dot * cos(roll) * cos(pitch)
    yaw_rate_body = yaw_rate_ef * math.cos(roll_ef) * math.cos(pitch_ef)
        
    return roll_ef, pitch_ef, yaw_rate_body, thrust_req
    
def vector_dot_product(ax, ay, az, bx, by, bz):
    return ax * bx + ay * by + az * bz


def body_to_ned(roll, pitch, yaw, bx, by, bz):
    """Rotate a body-frame vector to NED using 3-2-1 Euler angles."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    nx = cp*cy*bx + (sr*sp*cy - cr*sy)*by + (cr*sp*cy + sr*sy)*bz
    ny = cp*sy*bx + (sr*sp*sy + cr*cy)*by + (cr*sp*sy - sr*cy)*bz
    nz = -sp*bx   + sr*cp*by              + cr*cp*bz
    return nx, ny, nz


def calculate_distance_vector(tx, ty, tz, px, py, pz):
    """Returns distance and difference vector from pursuer to target."""
    rx = tx - px
    ry = ty - py
    rz = tz - pz
    d = math.sqrt(rx**2 + ry**2 + rz**2)
    return d, rx, ry, rz


def global_to_ned(lat, lon, alt_m, home_lat, home_lon, home_alt, earth_radius=6378137.0):
    """
    Convert a global (lat, lon, alt) to NED metres relative to a home.
    Uses a flat-earth approximation which is accurate to ~1 m per km.
    """
    d_lat = math.radians(lat - home_lat)
    d_lon = math.radians(lon - home_lon)
    cos_home = math.cos(math.radians(home_lat))

    north = d_lat * earth_radius                  # metres North
    east  = d_lon * earth_radius * cos_home        # metres East
    down  = -(alt_m - home_alt)                    # NED: positive = below home
    return (north, east, down)
