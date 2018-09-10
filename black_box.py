import numpy as np
import time

# flight data recorder

class BlackBox:
    def __init__(self, size = (1000,3)):
        self.data = np.zeros(size)
        self.i = 0
        self.j = False

    def record(self, v):
        for n in range(len(v)):
            self.data[self.i][n] = v[n]
        self.i += 1
        self.i %= len(self.data)

    def write(self):
        f = open('data-' + str(time.time()) + '.csv','w')
        for m in range(len(self.data)):
            for n in range(len(self.data[m])):
                f.write(str(self.data[m][n]))
                if n < len(self.data[m]) - 1:
                    f.write(',')
            f.write('\n')
        f.close()

b = BlackBox(size=(1000,6))
print(b.data)
b.record([1,2,3,4,5,6])
b.record([1,2,3,4,5,6])
b.record([1,2,3,4,5,6])
b.record([1,2,3,4,5,6])
b.write()
