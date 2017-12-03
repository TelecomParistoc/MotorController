from matplotlib.pyplot import *
from subprocess import call, check_output
from sys import argv

temp_file = "data_test/targets.txt"
print("#---------- CHECK TARGET COMPUTATION ---------- #")

if len(argv) != 5:
    print("Wrong number of parameters")
    print("Usage : python this_file.py a_m a_d v_c x_f")
    exit()

call("make target_computation", shell=True)
with open(temp_file, "w") as f:
    call("./target_computation " + " ".join(argv[1:]), stdout=f, shell=True)

with open(temp_file, "r") as f:

    #attention, il faut virer le \n final
    l_x = map(float, f.readline().split(',')[:-1])
    l_y = map(float, f.readline().split(',')[:-1])

    plot(l_x, l_y)
    show()
