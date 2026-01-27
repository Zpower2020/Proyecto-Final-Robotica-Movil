# Navegación Autónoma con Deep Q-Network (DQN) en ROS2

Este proyecto implementa un agente de Aprendizaje por Refuerzo (DQN) para la navegación autónoma de un robot móvil en el simulador Stage. El sistema utiliza ROS2 Humble y procesa lecturas de LiDAR 2D para navegar hacia objetivos aleatorios evitando obstáculos.

## 1. Instalación y Obtención del Proyecto

Dirígete a la carpeta `src` de tu workspace:

```bash
cd ~/tucarpeta_ws/src
```

Clona el repositorio:

```bash
git clone [https://github.com/Zpower2020/Proyecto-Final-Robotica-Movil.git](https://github.com/Zpower2020/Proyecto-Final-Robotica-Movil.git)
```

**Renombrar la carpeta (OBLIGATORIO):**
El sistema espera que el paquete se llame "dqn_robot_nav".

```bash
mv Proyecto-Final-Robotica-Movil dqn_robot_nav
```

Compilar el proyecto:

```bash
cd ~/tucarpeta_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. Instrucciones de Ejecución (2 Terminales)

El sistema requiere dos terminales abiertas simultáneamente: una para el entorno y otra para el cerebro.

### TERMINAL 1: El Entorno
**(Mantener esta terminal siempre abierta)**

Lanza el simulador Stage y el sistema de corrección de odometría.

```bash
ros2 launch dqn_robot_nav stage_env.launch.py
```

### TERMINAL 2: El Cerebro (Elige una opción)

En esta segunda terminal, ejecuta **SOLO UNO** de los siguientes comandos según lo que quieras hacer:

#### Opción A: ENTRENAR (Aprender desde cero)
El robot comenzará a explorar y realizara 500 episodios, guardará modelos en `saved_models/` cada 10 episodios y generará logs en `training_log.csv`.

```bash
ros2 run dqn_robot_nav train_node
```

#### Opción B: EVALUAR (Test Final)
Para probar un modelo ya entrenado, debes seleccionar cuál usar:

1. Si realizaste un entrenamiento reciente, ve a la carpeta `saved_models` y elige el archivo de modelo que desees(En general el ultimo es el mejor, como `dqn_model_490.pkl`); copialo y ve a la carpeta `saved_models`, pega el archivo y renombralo modificando el numero final (`..._490.pkl`) por un numero que corresponda al numero de modelo mas reciente en la carpeta (Por ejemplo `dqn_model_1.pkl` o `dqn_model_2.pkl`). Estos otros archivos de modelo son de anteriores entrenamientos con el objetivo de evaluar cada uno y determinar el mejor rendimiento.
2. Abre el archivo `src/dqn_robot_nav/dqn_robot_nav/test_node.py`.
3. Busca la línea que dice `model_path =` y cambia el numero final del nombre del archivo por el del modelo que desees evaluar (El archivo `dqn_model_1.pkl` es el modelo presente en la carpeta con mayor porcentaje de exito):

```python
model_path = os.path.join(os.getcwd(), 'saved_models', 'dqn_model_1.pkl')
```


4. Guarda los cambios, realiza `colcon build` si corresponde y ejecuta el test:

```bash
ros2 run dqn_robot_nav test_node
```

#### Opción C: GENERAR GRÁFICAS 
Genera las curvas de aprendizaje y la reconstrucción histórica basándose en los modelos guardados.

```bash
ros2 run dqn_robot_nav reconstruct_history
```

## 3. Estructura del Paquete

* `launch/stage_env.launch.py`: Launcher del Entorno.
* `dqn_robot_nav/dqn_agent.py`: Red Neuronal y Lógica DQN.
* `dqn_robot_nav/environment.py`: Interfaz ROS2-Stage.
* `dqn_robot_nav/train_node.py`: Nodo de Entrenamiento.
* `dqn_robot_nav/test_node.py`: Nodo de Test.
* `dqn_robot_nav/reconstruct_history.py`: Generador de Gráficas.
