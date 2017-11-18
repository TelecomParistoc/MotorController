from matplotlib.pyplot import *

with open("data_test/data.txt", "r") as f:

    #attention, il faut virer le \n final
    l_x = map(float, f.readline().split(',')[:-1])
    l_y = map(float, f.readline().split(',')[:-1])

    plot(l_x, l_y)
    show()
