import math

# Earth radius (meters)
R_EARTH = 6378137.0

def get_target_location(lat, lon, dx, dy, heading_deg):
    """
    Adds dx (right/left) and dy (forward/backward) offsets to the given (lat, lon) position, taking
    into account the heading angle, and returns the new coordinate GPS.

    :param lat: Leader's Latitude (Latitude) :param lon: Leader's Longitude (Longitude) :param dx:
    Offset to the right (meters) - If to the left minus (-) :param dy: Forward offset (meters) - If
    backwards minus (-) :param heading_deg: Compass angle of the leader (North 0, East 90) :return:
    (new_lat, new_lon)
    """
    # Convert heading angle to radians
    heading_rad = math.radians(heading_deg)

    # We convert dx and dy to vectors on the Earth axes (North/East) X axis north (0 degrees), Y axis east
    # (90 degrees)
    d_north = dy * math.cos(heading_rad) - dx * math.sin(heading_rad)
    d_east = dy * math.sin(heading_rad) + dx * math.cos(heading_rad)

    # Convert coordinate offsets to Degrees (Haversine approximation)
    d_lat = (d_north / R_EARTH) * (180.0 / math.pi)
    d_lon = (d_east / (R_EARTH * math.cos(math.pi * lat / 180.0))) * (180.0 / math.pi)

    # Add to leader's location and find new coordinates GPS
    new_lat = lat + d_lat
    new_lon = lon + d_lon

    return new_lat, new_lon
