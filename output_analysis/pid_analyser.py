import matplotlib.pyplot as plt
from sys import argv

P = 0; I = 1; D = 2
LEFT = 0
RIGHT = 1

def reset():
    global lin_pid, ang_pid, lin_acc_saturation, ang_acc_saturation
    global global_saturation, pos, target

    lin_pid = []
    ang_pid = []
    lin_acc_saturation = []
    ang_acc_saturation = []
    global_saturation = [[], []]
    pos = []
    target = []

reset()

with open(argv[1], "r") as f:
    for line in f:

        #The JLinkRTTServer has a buffer which saves the previous output until
        #it is read by the client.In practice we are not interested in what
        #happened before the init
        if line.find("Init OK") != -1:
            reset()
            print "Init found, reset done"
            print line

        if line.find("[P_I]") == -1: continue

        try:
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

        except Exception as e:
            print e

    # can fail if the lists are empty, ie the measurements of these metrics
    # has not been done
    try:
        plt.plot(zip(*lin_pid)[0], label="lin_p")
        plt.plot(zip(*lin_pid)[1], label="lin_i")
        plt.plot(zip(*lin_pid)[2], label="lin_d")
        plt.plot(zip(*ang_pid)[0], label="ang_p")
        plt.plot(zip(*ang_pid)[1], label="ang_i")
        plt.plot(zip(*ang_pid)[2], label="ang_d")
        plt.title("PID weight repartition")
        plt.legend()
    except IndexError:
        pass

    try:
        plt.figure()
        plt.plot(pos, label="position")
        plt.plot(target, label="target")
        plt.legend()
        plt.title("Position and target")
    except IndexError:
        pass

    try:
        plt.figure()
        plt.plot(lin_acc_saturation, label="lin_acc / max_lin_acc")
        plt.plot(ang_acc_saturation, label="ang_acc / max_ang_acc")
        plt.plot(global_saturation[LEFT], label="left saturation") # = command / MAX_PWM
        plt.plot(global_saturation[RIGHT], label="right saturation")
        plt.legend()
        plt.title("Saturation")
    except IndexError:
        pass

    plt.show()
