# 🎯 Control System Simulation with PID and Proportional Navigation (PN)

## 📌 Introduction

This project illustrates the importance of **error management** and **control systems** in autonomous systems.

An object (rocket) moves freely, while a pursuer (missile) applies a control law to intercept it, while avoiding obstacles.

---

## ⚙️ Theory: PID vs PN

### 🔹 PID (Proportional, Integral, Derivative)

A PID continuously corrects the **error** between the desired trajectory and the actual state.

![pid](https://github.com/user-attachments/assets/8e60ed9e-f773-4d31-b787-b3afd132d338)


## $$u(t) = K_p \cdot e(t) + K_i \cdot \int e(t) \, dt + K_d \cdot \frac{de(t)}{dt}$$

**Breakdown:**
- **u(t)**: Control signal/controller output at time t
- **e(t)**: Error at time t = setpoint - measurement
- **Kp, Ki, Kd**: Gains (tuning constants)

**The three terms:**

1. **Proportional Term**: `Kp · e(t)`
   - **Immediate** reaction to current error
   - The larger the error, the stronger the correction

2. **Integral Term**: `Ki · ∫ e(t) dt`
   - Corrects **persistent errors** (accumulation over time)
   - Eliminates steady-state error (permanent offset)

3. **Derivative Term**: `Kd · de(t)/dt`
   - Anticipates **future variations** of the error
   - Improves stability, reduces oscillations

👉 **Used everywhere**: drones, motors, industrial control, etc.

---

### 🔹 Proportional Navigation (PN)

PN adjusts the trajectory not by following the target directly, but by **correcting based on the variation of the line of sight (LOS)**.

<img width="1024" height="585" alt="unnamed (1)" src="https://github.com/user-attachments/assets/46ffe3aa-0856-4c41-9edc-0cea53cef4e4" />


## $$a_m = N \cdot V_m \cdot \dot{\lambda}$$

**Breakdown:**
- **am**: Missile lateral acceleration (perpendicular to its velocity)
- **N**: Navigation constant (typically 3-5)
- **Vm**: Missile velocity
- **λ̇**: Line of sight angular rate (LOS rate)

**Physical principle:**

**Line of Sight (LOS)**: imaginary line connecting the missile to its target

**λ̇ (lambda dot)**: How fast this line rotates in space
- If λ̇ = 0 → the line of sight doesn't rotate → direct collision course
- If λ̇ ≠ 0 → trajectory correction needed

**Operation:**
1. The missile measures how fast the line of sight is rotating
2. It applies lateral acceleration proportional to this rotation
3. This correction tends to maintain λ̇ = 0, thus heading straight toward the target

👉 **Advantage**: Instead of constantly pointing toward the target (inefficient), PN predicts where the target will be and optimizes the interception trajectory.

### 🎯 Practical Comparison

| Aspect | PID | Proportional Navigation |
|--------|-----|------------------------|
| **Usage** | Position/velocity control | Interception guidance |
| **Input** | Position error | LOS angular rate |
| **Output** | Motor command | Lateral acceleration |
| **Objective** | Follow a setpoint | Intercept a moving target |

---

## 🕹️ Simulation

![Vidéo sans titre ‐ Réalisée avec Clipchamp](https://github.com/user-attachments/assets/82a1b5a4-3021-4e36-9738-8183d873e31d)


- **Rocket**: inertia + friction, controlled by mouse drag
- **Missile**: PN + automatic obstacle avoidance
- **Obstacles**: random circles and rectangles

👉 **Rule**: If collision → explosion + object destruction.

---

## 🚀 Installation and Execution

1. **Clone the project**:
```bash
git clone https://github.com/your-profile/your-project.git
cd your-project
```

2. **Install dependencies**:
```bash
pip install -r requirements.txt
```

3. **Launch the simulation**:
```bash
python missile_pn_drag_launch.py
```

---

## 📖 Usage

- **Left click + drag**: move the rocket
- **Space**: restart the simulation
- **Escape**: quit

---

## 🔧 Configurable Parameters

In `config.py`:
- `N`: proportional navigation constant
- `Kp`, `Ki`, `Kd`: PID controller gains
- Number and size of obstacles

---

## 📚 References

- [Guidance and Control Theory](https://example.com)
- [PID Control Systems](https://example.com)

---

## 📝 License

MIT License - see `LICENSE` file for more details.
