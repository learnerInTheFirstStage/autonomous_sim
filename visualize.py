import matplotlib.pyplot as plt


def load_points(filename):
    xs, ys = [], []
    with open(filename) as f:
        for line in f:
            x, y = map(float, line.strip().split())
            xs.append(x)
            ys.append(y)
    return xs, ys

path_x, path_y = load_points('build/path.txt')
traj_x, traj_y = load_points('build/trajectory.txt')

plt.figure(figsize=(6,6))

plt.plot(path_x, path_y, 'g-o', label="Planned Path")
plt.plot(traj_x, traj_y, 'r-o', label="Vehicle Trajectory")

plt.scatter(path_x[0], path_y[0], c='blue', label="Start")
plt.scatter(path_x[-1], path_y[-1], c='black', label="Goal")

plt.title("Autonomous Driving Simulation")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.legend()
plt.grid(True)

plt.show()