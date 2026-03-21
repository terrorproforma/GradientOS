def move_line(x, y, z, v=None, a=None, closed=False):
    parts = [str(x), str(y), str(z)]
    if v is not None:
        parts.append(str(v))
    if a is not None:
        parts.append(str(a))
    if v is not None or a is not None:
        parts.append(str(closed).lower())
    return "MOVE_LINE," + ",".join(parts)


def move_line_relative(dx, dy, dz, speed=None, closed=False):
    parts = [str(dx), str(dy), str(dz)]
    if speed is not None:
        parts.append(str(speed))
        parts.append(str(closed).lower())
    return "MOVE_LINE_RELATIVE," + ",".join(parts)


def set_orientation(roll, pitch, yaw):
    return f"SET_ORIENTATION,{roll},{pitch},{yaw}"


def run_trajectory(name, use_cache=False, loop=False):
    return f"RUN_TRAJECTORY,{name},{str(use_cache).lower()},{str(loop).lower()}"


def set_gripper(angle_deg):
    return f"SET_GRIPPER,{int(angle_deg)}"


