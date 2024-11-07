import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.interpolate import interp1d

# Creazione di una traiettoria X e Y a forma di U
x_original = np.array([0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600])
y_original = np.array([0, 25, 50, 100, 150, 250, 300, 350, 400, 450, 500, 550, 600])

# Interpolazione delle traiettorie X e Y
interp_points = 200  # Numero di punti di interpolazione
t = np.linspace(0, 1, len(x_original))
t_new = np.linspace(0, 1, interp_points)

interp_x = interp1d(t, x_original, kind='cubic')
interp_y = interp1d(t, y_original, kind='cubic')

x_interpolated = interp_x(t_new)
y_interpolated = interp_y(t_new)

# Creazione della figura e dell'asse
fig, ax = plt.subplots()
ax.set_xlim(0, max(x_original))
ax.set_ylim(0, max(y_original))
ax.set_xlabel("Posizione Asse X (mm)")
ax.set_ylabel("Posizione Asse Y (mm)")
ax.set_title("Animazione Traiettoria X-Y")
ax.plot(x_interpolated, y_interpolated, color='gray', linestyle='-', label="Traiettoria Interpolata")
ax.legend()

# Punto iniziale per l'animazione
point, = ax.plot([], [], 'ro', label="Posizione Attuale")
ax.legend()

# Funzione di inizializzazione
def init():
    point.set_data([], [])
    return point,

# Funzione di animazione
def animate(i):
    point.set_data(x_interpolated[i], y_interpolated[i])
    return point,

# Creazione dell'animazione
ani = FuncAnimation(fig, animate, frames=len(x_interpolated), init_func=init, blit=True, interval=25)

plt.grid(True)
plt.show()
