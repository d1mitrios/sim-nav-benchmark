"""Read a world manifest (runs/world_<seed>.csv) written by isaac/generate_world.py.

Rows are type,name,x,y,param1,param2,yaw. Boxes carry their size in param1/param2,
cylinders their radius in param1, doors and gates their clear width in param1.
"""
import os

RUNS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "runs")


def load(seed):
    rows = []
    for line in open(os.path.join(RUNS_DIR, f"world_{seed}.csv")):
        p = line.strip().split(",")
        if len(p) >= 4 and p[0] in ("box", "cyl", "door", "gate", "person"):
            rows.append(p)
    num = lambda v: float(v)
    boxes = [(num(p[2]), num(p[3]), num(p[4]), num(p[5]), num(p[6]))
             for p in rows if p[0] == "box" and p[1].startswith("rnd_box")]
    partitions = [(num(p[2]), num(p[3]), num(p[4]), num(p[5]))
                  for p in rows if p[0] == "box" and p[1].startswith("partition")]
    cyls = [(num(p[2]), num(p[3]), num(p[4])) for p in rows if p[0] == "cyl"]
    doors = [dict(name=p[1], x=num(p[2]), y=num(p[3]), w=num(p[4]), axis=p[5])
             for p in rows if p[0] == "door"]
    persons = [(num(p[2]), num(p[3])) for p in rows if p[0] == "person"]
    walls = {p[1]: (num(p[2]), num(p[3])) for p in rows if p[0] == "box" and p[1].startswith("gate_")}
    gates = [(num(p[2]), num(p[3]), num(p[4]), num(p[5]), num(p[6]),
              walls[p[1] + "_a"], walls[p[1] + "_b"]) for p in rows if p[0] == "gate"]
    return dict(boxes=boxes, partitions=partitions, cyls=cyls, doors=doors,
                persons=persons, gates=gates)
