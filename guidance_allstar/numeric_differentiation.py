from collections import deque

class VelocityDifferentiator:
    """
    Computes numerical differentiation of a continuous 3D velocity stream.
    Maintains a rolling history of (time, vx, vy, vz) samples to apply
    higher-order differentiation stencils.
    """
    def __init__(self, max_history=5):
        # Store tuples of (t, vx, vy, vz)
        self.history = deque(maxlen=max_history)

    def update(self, t, vx, vy, vz):
        """Add a new velocity sample to the history."""
        self.history.append((t, vx, vy, vz))

    def get_acceleration(self, method="backwards"):
        """
        Calculates acceleration (ax, ay, az).
        Falls back to simpler methods if not enough history exists.
        
        methods: "backwards" (O(h^2)), "central", or "first_order"
        """
        n = len(self.history)
        if n < 2:
            return (0.0, 0.0, 0.0)

        t_curr = self.history[-1][0]
        t_prev = self.history[-2][0]
        dt = t_curr - t_prev
        
        # Guard against zero or extremely small dt to avoid division by zero/noise spikes
        if dt < 0.001:
            return (0.0, 0.0, 0.0)

        # Fallback to simple first-order difference if we don't have enough data
        if method == "first_order" or (method == "backwards" and n < 3) or (method == "central" and n < 5):
            # (v[i] - v[i-1]) / dt
            v_curr = self.history[-1]
            v_prev = self.history[-2]
            return tuple((v_curr[i] - v_prev[i]) / dt for i in range(1, 4))
            
        elif method == "backwards":
            # Backwards difference O(h^2), relies on i, i-1, i-2
            # Formula: (3*v[i] - 4*v[i-1] + v[i-2]) / (2*dt)
            v0 = self.history[-1]
            v1 = self.history[-2]
            v2 = self.history[-3]
            
            return tuple(
                (3 * v0[i] - 4 * v1[i] + v2[i]) / (2 * dt)
                for i in range(1, 4)
            )
            
        elif method == "central":
            # Central difference, relies on i, i-1, (skips i-2 which is the center), i-3, i-4
            # Formula: (-v[i] + 8*v[i-1] - 8*v[i-3] + v[i-4]) / (12*dt)
            # NOTE: this calculates the derivative at time i-2, introducing a 2-tick latency
            v0 = self.history[-1]  # i
            v1 = self.history[-2]  # i-1
            # v2 = self.history[-3] would be i-2, the point we are evaluating at
            v3 = self.history[-4]  # i-3
            v4 = self.history[-5]  # i-4
            
            return tuple(
                (-v0[i] + 8 * v1[i] - 8 * v3[i] + v4[i]) / (12 * dt)
                for i in range(1, 4)
            )
            
        else:
            # Safe fallback
            return (0.0, 0.0, 0.0)