import matplotlib.pyplot as plt
from sys import argv

P = 0; I = 1; D = 2
LEFT = 0
RIGHT = 1

def reset():
    global lin_pid, ang_pid, lin_acc_saturation, ang_acc_saturation
    global global_saturation, pos, target, heading, target_heading, c_w_heading, imu_heading

    lin_pid = []
    ang_pid = []
    lin_acc_saturation = []
    ang_acc_saturation = []
    global_saturation = [[], []]
    pos = []
    target = []
    heading = []
    target_heading = []
    c_w_heading = []
    imu_heading = []

def angle_conversion(x):
    # if x is an angle between 0 and 360 * 16, it returns the corresponding
    # value in range ]-180, +180] degrees.
    if x < 0 or x > 360 * 16:
        print "ERROR: impossible to convert x = ", x, "in degrees"
        return 666

    if x <= 360 * 16 / 2: return x / 16.
    return (x - 360 * 16) / 16.

def mean(data):
    return sum(data) / float(len(data))

def standard_deviation(data):
    m = mean(data)
    return sum((x - m) ** 2 for x in data) ** .5

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
                target_heading.append(angle_conversion(line_f[2]))
                heading.append(angle_conversion(line_f[3]))
                c_w_heading.append(angle_conversion(line_f[4]))
                imu_heading.append(angle_conversion(line_f[5]))

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
        plt.plot(global_saturation[LEFT], label="left_motor_command / max_command") # = command / MAX_PWM
        plt.plot(global_saturation[RIGHT], label="right_motor_command / max_command")
        plt.legend()
        plt.title("Saturation (per mille)")
    except IndexError:
        pass

    plt.figure()
    plt.plot(target_heading, label="target heading")
    plt.plot(heading, label="estimated heading")
    plt.plot(c_w_heading, label="heading according coding wheels")
    plt.plot(imu_heading, label="heading according IMU")
    plt.legend()
    plt.title("Orientation")
    #print "Orientation: Average and standard_deviation:"
    #print "\tCoding wheels: ", mean(c_w_heading), standard_deviation(c_w_heading)
    #print "\tIMU: ", mean(imu_heading), standard_deviation(imu_heading)

    plt.show()
