# Laboratorio de Simulación Robótica con Docker, ROS, PyBullet, LIDAR y SLAM
## Descripción General

Este repositorio contiene el desarrollo completo del laboratorio dividido en tres puntos principales:

Investigación y análisis de herramientas robóticas (ROS, MoveIt, Gazebo y tecnologías IoT).

Simulación de un cuadrúpedo (UR5) con PyBullet, desplegado mediante Docker.

Simulación de un robot móvil TurtleBot3 con LIDAR y SLAM (gmapping) en Docker, visualizando el mapa en tiempo real con Rviz y Gazebo.

## 1️⃣ Punto 1 — Investigación de Herramientas
### Objetivo

Investigar las herramientas ROS, MoveIt y Gazebo, su relación con el Internet de las Cosas (IoT), y su aplicación en la robótica moderna.

### Contenido

ROS: middleware para robótica y su integración con IoT mediante ROS–MQTT y FOGROS2.

MoveIt: planificación de movimiento, control cinemático y manipulación inteligente.

Gazebo: simulación física 3D para validación de robots y gemelos digitales.

El documento fue desarrollado en Overleaf con citas y referencias, abordando los alcances de cada herramienta en el campo IoT.

## 2️⃣ Punto 2 — Simulación de un Cuadrúpedo (UR5) con PyBullet
### Objetivo

Controlar y visualizar un cuadrúpedo UR5 usando PyBullet, desplegándolo dentro de un contenedor Docker.

### Pasos
#### Crear el Dockerfile

imagen

#### Construir y ejecutar la imagen

Imagen

### Resultados

El cuadrúpedo se visualiza correctamente en la simulación PyBullet.

El despliegue se ejecuta desde Docker (docker run --rm).

Se validó el control básico de movimiento.

## 3️⃣ Punto 3 — Simulación TurtleBot3 con LIDAR y SLAM (Gmapping)
### Objetivo

Implementar un robot TurtleBot3 con sensor LIDAR, usando ROS Noetic y SLAM (gmapping) para crear un mapa en tiempo real dentro de Docker.

### Estructura del Dockerfile
imagen

### Instalar TurtleBot3 y SLAM

imagen 

### Configurar entorno

Imagen 

### Script de inicio

Imagen

🔹 Ejecución del Sistema
🧩 Terminal 1 — Simulación (Gazebo)
```
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --network host \
    --name slam-bot \
    turtlebot3-slam
```
🧠 Terminal 2 — SLAM (Dentro del contenedor)
```
docker exec -it slam-bot bash -c \
"source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping"
```
🎮 Terminal 3 — Teleoperación
```
docker exec -it slam-bot bash -c \
"source /opt/ros/noetic/setup.bash && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"
```
🗺️ Terminal 4 — Visualización del mapa (RViz)
```
docker exec -it slam-bot bash -c \
"source /opt/ros/noetic/setup.bash && rosrun rviz rviz -d /opt/ros/noetic/share/turtlebot3_slam/rviz/turtlebot3_gmapping.rviz"
```
### Resultados 
