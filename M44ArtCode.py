import numpy as np
import matplotlib.pyplot as plt

def densidade_do_ar(altitude):
    """
    Calcula a densidade do ar com base na altitude (até 11 km).

    Args:
        altitude (float): Altitude em metros.

    Returns:
        float: Densidade do ar em kg/m^3.
    """
    rho_0 = 1.225  # Densidade do ar ao nível do mar (kg/m^3)
    if altitude > 11000:
        return 0  # Aproximação para altitudes acima de 11 km
    return rho_0 * (1 - altitude / 44300) ** 4.256

def calcular_trajetoria(massa, velocidade, angulo, diametro, gravidade=9.81, C_d=0.5):
    """
    Calcula a trajetória de um projétil considerando resistência do ar.

    Args:
        massa (float): Massa do projétil (kg).
        velocidade (float): Velocidade inicial (m/s).
        angulo (float): Ângulo de lançamento (graus).
        diametro (float): Diâmetro do projétil (m).
        gravidade (float): Aceleração da gravidade (m/s^2).
        C_d (float): Coeficiente de arrasto.

    Returns:
        tuple: Vetores com posições x e y.
    """
    # Verificando limites do ângulo
    angulo = max(min(angulo, 63), -5)

    # Convertendo o ângulo para radianos
    angulo_rad = np.radians(angulo)

    # Componentes da velocidade inicial
    vx = velocidade * np.cos(angulo_rad)
    vy = velocidade * np.sin(angulo_rad)

    # Área da seção transversal
    A = np.pi * (diametro / 2) ** 2

    # Passo de tempo para a simulação
    dt = 0.01

    # Listas para armazenar posições
    x, y = [0], [0]

    # Velocidades iniciais
    v_x, v_y = vx, vy

    while y[-1] >= 0:
        # Velocidade total
        v = np.sqrt(v_x**2 + v_y**2)

        # Altitude atual
        altitude = y[-1]

        # Densidade do ar
        rho = densidade_do_ar(altitude)

        # Força de arrasto
        F_d = 0.5 * rho * C_d * A * v**2

        # Acelerações devido à gravidade e ao arrasto
        a_x = -F_d * (v_x / v) / massa
        a_y = -gravidade - (F_d * (v_y / v) / massa)

        # Atualização das velocidades
        v_x += a_x * dt
        v_y += a_y * dt

        # Atualização das posições
        x.append(x[-1] + v_x * dt)
        y.append(y[-1] + v_y * dt)

    return np.array(x), np.array(y)

# Parâmetros do projétil
massa = 43  # kg
velocidade = 564  # m/s
angulo = 30  # graus
diametro = 0.155  # m (155 mm)

# Calculando a trajetória
x, y = calcular_trajetoria(massa, velocidade, angulo, diametro)

# Filtrando os pontos onde y >= 0
x = x[y >= 0]
y = y[y >= 0]

# Ajustando o alcance máximo se necessário
alcance_maximo = 14955  # metros
x = x[x <= alcance_maximo]
y = y[:len(x)]

# Plotando a trajetória
plt.figure(figsize=(10, 6))
plt.plot(x, y, label=f"Ângulo: {angulo}°")
plt.title("Trajetória do Projétil de Artilharia")
plt.xlabel("Distância Horizontal (m)")
plt.ylabel("Altura (m)")
plt.legend()
plt.grid()
plt.show()
