def combine(dmp_p):
    traj = []
    for i, p in enumerate(dmp_p):
        pose = p.tolist() + [0.536, 3.094, 0]
        traj.append(pose)
    return traj