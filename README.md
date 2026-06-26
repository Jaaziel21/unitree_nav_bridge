# unitree_nav_bridge

Puente de navegación entre **[Nav2](https://navigation.ros.org/)** y la **API de movimiento propietaria de Unitree** para el cuadrúpedo **Go2**.

El SDK de Unitree expone la primitiva de bajo nivel `SportClient::Move(vx, vy, vyaw)`, pero **no** incluye la integración con el stack estándar de ROS 2 ni la lógica de seguridad necesaria para una navegación autónoma. Este paquete aporta esa capa intermedia: convierte los comandos de velocidad estándar (`geometry_msgs/Twist` en `/cmd_vel`) en peticiones de la API de Unitree, añadiendo *watchdog*, verificación de modo y saturación de velocidades.

---

## 🎥 Demo

<!-- Sustituye el enlace por tu vídeo (puedes arrastrar un .mp4 a un issue/PR de GitHub y pegar la URL que se genera, o enlazar a YouTube). -->

> _Vídeo del Go2 navegando de forma autónoma con Nav2 (próximamente)._

<!--
[![Demo de navegación](docs/thumbnail.png)](https://www.youtube.com/watch?v=TU_VIDEO_ID)
-->

---

## Arquitectura

```
        ┌──────────────┐   /cmd_vel (Twist)   ┌─────────────────────┐   /api/sport/request   ┌────────────┐
        │     Nav2     │ ───────────────────► │  unitree_nav_bridge │ ─────────────────────► │  Go2 (API) │
        │ (controller) │                      │  (este paquete)     │     unitree_api/Req    │  SportMode │
        └──────────────┘                      └─────────────────────┘                        └────────────┘
               ▲                                        ▲                                            │
               │ map → odom → base_footprint            │ lf/sportmodestate (modo actual)            │
               │                                        └────────────────────────────────────────────┘
        ┌──────┴───────┐      ┌──────────────┐      ┌──────────────────────────┐
        │ slam_toolbox │◄─────│   robot_     │◄─────│  Go2: /utlidar/scan,     │
        │   (mapping)  │      │ localization │      │  /utlidar/robot_odom     │
        └──────────────┘      │    (EKF)     │      └──────────────────────────┘
                              └──────────────┘
```

**Flujo de datos:**
1. El LiDAR del Go2 (`/utlidar/robot_odom`) alimenta un **EKF** (`robot_localization`) que publica la TF `odom → base_footprint`.
2. **slam_toolbox** construye el mapa a partir de `/utlidar/scan` y publica `map → odom`.
3. **Nav2** planifica y publica velocidades en `/cmd_vel`.
4. **`unitree_nav_bridge`** traduce esas velocidades a la API de Unitree, con la lógica de seguridad descrita abajo.

---

## Características

- **Traducción `/cmd_vel` → `SportClient::Move`** — conecta cualquier fuente de `Twist` (Nav2, teleop, etc.) con el Go2.
- **Watchdog de seguridad** — si dejan de llegar comandos durante `cmd_timeout` segundos, el robot se detiene automáticamente.
- **Verificación de modo de locomoción** — solo se envían comandos si el cuadrúpedo está en un modo seguro (de pie / caminando); evita forzar el hardware cuando los motores están en *damping*.
- **Saturación de velocidades** — recorta `vx`, `vy`, `vyaw` a los límites físicos del robot.
- **Loop de control a frecuencia fija** (50 Hz por defecto), desacoplado de la llegada de comandos mediante un *timer*.
- **Totalmente parametrizable** vía YAML — tópicos, frecuencia, límites y modos válidos sin recompilar.
- Ejecución con `MultiThreadedExecutor` para no serializar el loop de control tras un callback lento.

---

## Dependencias

- ROS 2 (probado en **Humble**)
- [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2) (provee `unitree_api` y `unitree_go`)
- `nav2`, `slam_toolbox`, `robot_localization`, `pointcloud_to_laserscan` (para el stack de navegación completo)

---

## Compilación

```bash
cd ~/ros2_ws
colcon build --packages-select unitree_nav_bridge
source install/setup.bash
```

## Uso

Lanzar solo el puente (con los parámetros por defecto de `config/bridge_params.yaml`):

```bash
ros2 launch unitree_nav_bridge bridge.launch.py
```

Con un fichero de parámetros propio:

```bash
ros2 launch unitree_nav_bridge bridge.launch.py params_file:=/ruta/a/mis_params.yaml
```

---

## Configuración

Todos los valores operativos están en [`config/bridge_params.yaml`](config/bridge_params.yaml):

| Parámetro | Tipo | Por defecto | Descripción |
|---|---|---|---|
| `cmd_vel_topic` | string | `/cmd_vel` | Tópico de entrada de velocidades (Nav2). |
| `state_topic` | string | `lf/sportmodestate` | Tópico de estado/modo del robot. |
| `control_frequency` | double | `50.0` | Frecuencia (Hz) de envío de `Move()`. |
| `cmd_timeout` | double | `0.3` | Segundos sin `cmd_vel` antes de detener (watchdog). |
| `valid_locomotion_modes` | int[] | `[0, 7, 9]` | Modos en los que es seguro mover el robot. |
| `max_vx` / `min_vx` | double | `1.5` / `-1.0` | Límites de velocidad de avance (m/s). |
| `max_vy` | double | `0.5` | Límite de desplazamiento lateral, simétrico (m/s). |
| `max_vyaw` | double | `1.0` | Límite de velocidad de giro, simétrico (rad/s). |

> ⚠️ **Nota sobre los modos:** los valores de `mode` en `SportModeState` (7 = de pie, 9 = caminando, 12 = damping…) pueden variar según la versión de firmware del robot. Verifica los tuyos antes de operar.

En `config/` también se incluyen ejemplos de configuración del stack de navegación:
- `ekf.yaml` — fusión de odometría con `robot_localization`.
- `mapper.yaml` — mapeo con `slam_toolbox`.
- `qos.yaml` — `pointcloud_to_laserscan` para generar `/utlidar/scan`.

---

## Tópicos

| Dirección | Tópico | Tipo |
|---|---|---|
| Sub | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Sub | `lf/sportmodestate` | `unitree_go/msg/SportModeState` |
| Pub | `/api/sport/request` | `unitree_api/msg/Request` |

---

## Estructura del paquete

```
unitree_nav_bridge/
├── src/
│   ├── unitree_bridge.cpp        # nodo del puente (Nav2 -> Unitree)
│   └── ros2_sport_client.cpp     # cliente de la API Sport de Unitree
├── include/unitree_nav_bridge/
│   ├── ros2_sport_client.h
│   └── patch.hpp
├── config/
│   ├── bridge_params.yaml        # parámetros del puente
│   ├── ekf.yaml                  # robot_localization (EKF)
│   ├── mapper.yaml               # slam_toolbox
│   └── qos.yaml                  # pointcloud_to_laserscan
├── launch/
│   └── bridge.launch.py
├── CMakeLists.txt
└── package.xml
```

---

## Licencia

[MIT](LICENSE)

> `ros2_sport_client.{h,cpp}` y `patch.hpp` provienen de los ejemplos de Unitree Robotics y conservan su copyright original.
