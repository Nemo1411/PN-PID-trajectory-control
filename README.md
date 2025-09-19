# 🎯 Simulation d’asservissement avec PID et Navigation Proportionnelle (PN)

## 📌 Introduction
Ce projet illustre l’importance de la **gestion d’erreur** et de l’**asservissement** dans les systèmes autonomes.  
Un objet (fusée) se déplace librement, tandis qu’un poursuivant (missile) applique une loi de contrôle pour l’intercepter, tout en évitant des obstacles.  

---

## ⚙️ Théorie : PID vs PN

### 🔹 PID (Proportionnel, Intégral, Dérivé)

Un PID corrige en continu l’**erreur** entre trajectoire idéale et état réel.  

![Schéma PID](assets/pid_diagram.png)  

\[
u(t) = K_p \cdot e(t) + K_i \cdot \int e(t)\,dt + K_d \cdot \frac{de(t)}{dt}
\]

👉 Utilisé partout : drones, moteurs, contrôle industriel, etc.

---

### 🔹 Navigation Proportionnelle (PN)

La PN ajuste la trajectoire non pas en suivant directement la cible, mais en **corrigeant en fonction de la variation de la ligne de visée (LOS)**.  

![Schéma PN](assets/pn_guidance.png)  

\[
a_m = N \cdot V_m \cdot \dot{\lambda}
\]

👉 Avantage : trajectoire d’interception plus efficace et stable.

---

## 🕹️ Simulation

![Aperçu Simulation](assets/simulation_demo.png)  

- **Fusée** : inertie + frottement, contrôlée par drag de souris.  
- **Missile** : PN + évitement automatique d’obstacles.  
- **Obstacles** : cercles et rectangles aléatoires.  

👉 Si collision : explosion + destruction de l’objet.  

---

## 🚀 Installation et exécution
1. Cloner le projet :
```bash
git clone https://github.com/ton-profil/ton-projet.git
cd ton-projet

