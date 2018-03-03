import matplotlib.pyplot as plt

P = 0; I = 1; D = 2
LEFT = 0
RIGHT = 1

lin_pid = []
ang_pid = []
lin_acc_saturation = []
ang_acc_saturation = []
global_saturation = [[], []]
pos = []
target = []

with open("output.txt", "r") as f:
    for line in f:

        if line.find("[PID_INFO]") == -1: continue

        line_f = [x for x in map(float, line.split(':')[-1].split(' ')[1:]) if x != '']

        if line.find("[LINEAR]") != -1:
            p, i, d, sat = line_f
            lin_pid.append((p, i, d))
            lin_acc_saturation.append(sat)

        elif line.find("[ANGULAR]") != -1:
            p, i, d, sat = line_f
            ang_pid.append((p, i, d))
            ang_acc_saturation.append(sat)

        elif line.find("[GLOBAL]") != -1:
            l_sat, r_sat = line_f
            global_saturation[LEFT].append(l_sat)
            global_saturation[RIGHT].append(r_sat)

        elif line.find("[POSITION]") != -1:
            target.append(line_f[0])
            pos.append(line_f[1])

    plt.plot(zip(*lin_pid)[0], label="lin_p")
    plt.plot(zip(*lin_pid)[1], label="lin_i")
    plt.plot(zip(*lin_pid)[2], label="lin_d")
    plt.plot(zip(*ang_pid)[0], label="ang_p")
    plt.plot(zip(*ang_pid)[1], label="ang_i")
    plt.plot(zip(*ang_pid)[2], label="ang_d")
    plt.title("PID weight repartition")
    plt.legend()

    plt.figure()
    plt.plot(pos, label="position")
    plt.plot(target, label="target")
    plt.legend()
    plt.title("Position and target")

    plt.figure()
    plt.plot(lin_acc_saturation, label="lin_acc / max_lin_acc")
    plt.plot(ang_acc_saturation, label="ang_acc / max_ang_acc")
    plt.plot(l_sat, label="left saturation") # = command / MAX_PWM
    plt.plot(r_sat, label="right saturation")
    plt.legend()
    plt.title("Saturation")

    plt.show()
