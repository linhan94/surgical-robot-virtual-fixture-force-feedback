import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate(frameno):
    x = mu + sigma * np.random.randn(N)
    n, _ = np.histogram(x, bins, normed=True)
    for rect, h in zip(patches, n):
        rect.set_height(h)
    return patches

N, mu, sigma = 10000, 100, 15
fig, ax = plt.subplots()
x = mu + sigma * np.random.randn(N)
n, bins, patches = plt.hist(x, 50, normed=1, facecolor='green', alpha=0.75)

frames = 100
ani = animation.FuncAnimation(fig, animate, blit=True, interval=0,
                              frames=frames,
                              repeat=False)
plt.show()