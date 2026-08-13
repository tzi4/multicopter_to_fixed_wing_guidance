import time
import datetime
from pathlib import Path
import guidance_config as cfg

class TelemetryLogger:
    def __init__(self, filepath=None):
        if filepath is None:
            filepath = str(Path(__file__).resolve().parents[1] /
                           "logs" / "drone_telemetry.log")
        # Change .csv to .log if passed explicitly
        if filepath.endswith('.csv'):
            filepath = filepath.replace('.csv', '.log')

        Path(filepath).expanduser().parent.mkdir(parents=True, exist_ok=True)
        self.filepath = filepath
        self.file = open(self.filepath, mode='w')
        self.step_count = 0

        # Print all configuration variables as the first line
        config_items = [f"{k}={v}" for k, v in vars(cfg).items() if not k.startswith('__')]
        config_str = "CONFIG: " + " | ".join(config_items)
        self.file.write(config_str + "\n\n")
        self.file.flush()

    def log_step(self, data_dict):
        self.step_count += 1

        # Format values safely
        def f(val, width=6, prec=2):
            if val is None or val == "":
                return " "*width
            if isinstance(val, (int, float)):
                return f"{val:{width}.{prec}f}"
            return f"{str(val):>{width}}"

        # Extract values
        ts = data_dict.get("timestamp", time.time())
        dt_str = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        
        mode = str(data_dict.get("mode", ""))[:4].ljust(4)
        
        rng = f(data_dict.get("range"), 7, 2)
        vc = f(data_dict.get("closing_velocity"), 6, 2)

        px, py, pz = f(data_dict.get("px")), f(data_dict.get("py")), f(data_dict.get("pz"))
        pvx, pvy, pvz = f(data_dict.get("pvx")), f(data_dict.get("pvy")), f(data_dict.get("pvz"))
        
        tx, ty, tz = f(data_dict.get("tx")), f(data_dict.get("ty")), f(data_dict.get("tz"))
        tvx, tvy, tvz = f(data_dict.get("tvx")), f(data_dict.get("tvy")), f(data_dict.get("tvz"))

        tax, tay, taz = f(data_dict.get("tax")), f(data_dict.get("tay")), f(data_dict.get("taz"))
        cmd_ax, cmd_ay, cmd_az = f(data_dict.get("cmd_acx")), f(data_dict.get("cmd_acy")), f(data_dict.get("cmd_acz"))
        
        # New attitude command telemetry (converting rad/s -> deg/s or rad -> deg for readability)
        cmd_roll = f(data_dict.get("cmd_roll", 0.0) * 57.2958)
        cmd_pitch = f(data_dict.get("cmd_pitch", 0.0) * 57.2958)
        cmd_yaw = f(data_dict.get("cmd_yaw", 0.0) * 57.2958)
        cmd_yaw_rate = f(data_dict.get("cmd_yaw_rate", 0.0) * 57.2958)
        raw_thrust_req = f(data_dict.get("raw_thrust_req"))
        thrust_req = f(data_dict.get("thrust_req"))
        thrust_xy_scale = f(data_dict.get("thrust_xy_scale"), 5, 2)
        thrust_lift = f(data_dict.get("thrust_lift"), 5, 2)

        log_str = (
            f"{dt_str} | {mode} | RNG:{rng}m | VC:{vc}m/s | "
            f"P_POS:[{px}, {py}, {pz}] | P_VEL:[{pvx}, {pvy}, {pvz}] | "
            f"T_POS:[{tx}, {ty}, {tz}] | T_VEL:[{tvx}, {tvy}, {tvz}] | "
            f"T_ACC:[{tax}, {tay}, {taz}] | CMD_A:[{cmd_ax}, {cmd_ay}, {cmd_az}] | "
            f"CMD_ATT:[{cmd_roll}, {cmd_pitch}, {cmd_yaw}, {cmd_yaw_rate}] | "
            f"THR:[{raw_thrust_req}, {thrust_req}, {thrust_xy_scale}, {thrust_lift}]\n"
        )
        self.file.write(log_str)
        self.file.flush()

    def close(self):
        self.file.close()

if __name__ == "__main__":
    logger = TelemetryLogger()
    logger.log_step({"timestamp": time.time(), "mode": "TEST", "px": 1.0, "py": 2.0})
    logger.close()
    print("Test log written.")
