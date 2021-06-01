# PK WK RW
# Pracownia Badawczo-Problemowa
# 2020/2021
# Rysowanie wykresów na podstawie symulacji wykonanej w C++ (Visual Studio)


import matplotlib.pyplot as plt


with open('results.txt') as f:
    lines = f.readlines()
    time = [float(line.split()[0]) for line in lines]
    x = [float(line.split()[1]) for line in lines]
    y = [float(line.split()[2]) for line in lines]
    xr = [float(line.split()[3]) for line in lines]
    yr = [float(line.split()[4]) for line in lines]
    u1 = [float(line.split()[5]) for line in lines]
    u2 = [float(line.split()[6]) for line in lines]
    dalfa1 = [float(line.split()[7]) for line in lines]
    dalfa2 = [float(line.split()[8]) for line in lines]
    z1 = [float(line.split()[9]) for line in lines]
    z2 = [float(line.split()[10]) for line in lines]

# print(time[-2])
d_time = time[-1]-time[-2]
full_time = int(time[-1]+d_time)
print(full_time)
plt.figure(1)
plt.plot(x, y, 'k')
plt.plot(z1, z2, 'r')
plt.xlim([-1.8, 0.3])
plt.ylim([-0.75, 1.35])
plt.grid()
plt.legend(["(g_x, g_y)", "(z_x, z_y)"])
plt.savefig("Wykres_1_g_i_z.svg")

plt.figure(2)
plt.plot(time, xr, 'k')
plt.plot(time,yr,'r')
plt.xlim([0, full_time])
plt.legend(["eta_3","eta_4"])
plt.grid()
plt.savefig("Wykres_2_eta3_eta4.svg")

plt.figure(3)
plt.plot(time, u1, 'k')
plt.xlim([0, full_time])
plt.legend(["u_1"])
plt.grid()
plt.savefig("Wykres_3_u1.svg")

plt.figure(4)
plt.plot(time, u2, 'k')
plt.xlim([0, full_time])
plt.legend(["u_2"])
plt.grid()
plt.savefig("Wykres_4_u2.svg")

plt.figure(5)
plt.plot(time, dalfa1, 'k')
plt.plot(time,dalfa2,'r')
plt.xlim([0, full_time])
plt.legend(["dalfa1","dalfa2"])
plt.grid()
plt.savefig("Wykres_5_dalfa1_dalfa2.svg")

plt.show()



