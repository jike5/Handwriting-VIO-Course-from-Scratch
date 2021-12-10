import matplotlib.pyplot as plt

f = open("./Lambda.txt", "r")
str = f.read().split()
Lambda = [float(x) for x in str]

plt.plot(range(len(Lambda)), Lambda)
plt.show()
