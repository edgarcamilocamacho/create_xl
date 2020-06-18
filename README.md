# Create XL

Conjunto de paquetes de ROS para puesta en marcha del robot Create XL (Create 2 con plataformas adicionales).

# Requisitos

Probado con los siguientes computadores a bordo:

* Up Squared - 4GB de RAM - Intel(R) Atom(TM) Processor E3940 @ 1.60GHz

Sistema Operativo:

* Ubuntu 18.04
* Mint 

Versiones de ROS:

* ROS Melodic

Paquetes de ROS necesarios:

* rtabmap
* rtabmap-ros

*Importante*: Los [drivers del Create 2](https://github.com/AutonomyLab/create_autonomy) no funcionaron con el kernel 5.0.0 (original del Ubuntu 18.04 y derivados), por lo cual se bajó a 4.14.0.

# Configuración

Convención para los computadores involucrados (conectados a la misma red):

* *Robot*: Computador a bordo
* *PC*: Computador externo

Variables de entorno en el Robot:
``` bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<ROBOT_IP>
```

Variables de entorno en el PC:
``` bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<PC_IP>
```

# Instalación

## Instalación en el PC

[PC] Cree el *workspace* y clone el repositorio:
``` bash
mkdir -p ~/ros/create_ws/src/
cd ~/ros/create_ws/src/
git clone git@github.com:edgarcamilocamacho/create_xl.git
```

[PC] Compile el *workspace*:
``` bash
cd ~/ros/create_ws/
catkin_make
```

## Instalación en el robot

[Robot] Cree el workspace en el robot:
``` bash
mkdir -p ~/ros/create_ws/src/create_xl/
```

Defina las variables `ROBOT_IP`, `ROBOT_USER` y `ROBOT_PASS` en el archivo [push2robot.py](https://github.com/edgarcamilocamacho/create_xl/blob/master/push2robot.py).

Debido a que se conectan dos cámaras *Realsense* (*D435* y *T256*), es necesario identificarlas para que cada nodo acceda a la correspondiente:
* Ingrese el serial de la *Realsense D435* [aquí](https://github.com/edgarcamilocamacho/create_xl/blob/master/create_camera/launch/create_camera_d435.launch#L6).
* Ingrese el serial de la *Realsense T265* [aquí](https://github.com/edgarcamilocamacho/create_xl/blob/master/create_camera/launch/create_camera_t265.launch#L8).


[PC] Ejecute el script [push2robot.py](https://github.com/edgarcamilocamacho/create_xl/blob/master/push2robot.py) para actualizar los paquetes en el robot por primera vez.

``` bash
python2 ~/ros/create_ws/src/create_xl/push2robot.py force_all
```

[Robot] Compile el *workspace*:
``` bash
cd ~/ros/create_ws/
catkin_make
```

[Robot] Conceda permisos a su usuario, para acceder al puerto USB:
``` bash
sudo adduser $USER dialout
```

Asegurarse que el puerto serial correspondiente al robot es asignado a `/dev/ttyUSB0`, si no, modificar [aquí](https://github.com/edgarcamilocamacho/create_xl/blob/master/create_autonomy/ca_driver/config/default.yaml#L2).

## Edición de paquetes

[PC] Luego de realizar alguna modificación en el contenido de algún paquete, al ejecutar el script [push2robot.py](https://github.com/edgarcamilocamacho/create_xl/blob/master/push2robot.py) sin la bandera `force_all`, éste actualizará sólo los paquetes modificados.

``` bash
python2 ~/ros/create_ws/src/create_xl/push2robot.py
```

[Robot] Vuelva a compilar el *workspace* si es necesario:
``` bash
cd ~/ros/create_ws/
catkin_make
```

# Ejecución

[PC] Para las siguientes sesiones, se debe ejecutar `roscore` en el PC:
``` bash
roscore
```

[PC y Robot] Además, necesitan que se haya configurado el *workspace* (considerar agregar la línea al archivo `~/.bashrc`), tanto en el PC como en el robot:
``` bash
source ~/ros/create_ws/devel/setup.bash
```

## Movimiento manual del robot

[Robot] Lanzar driver:
``` bash
roslaunch create_robot create_movement.launch
```

Debe mostrar algo como:
```
...
[ INFO] [1592511326.425346392]: [CREATE] "CREATE_2" selected
[ INFO] [1592511327.590941295]: [CREATE] Connection established.
[ INFO] [1592511327.591350194]: [CREATE] Battery level 91.44 %
[ INFO] [1592511328.040531149]: [CREATE] Ready.
...
```

[Robot o PC] Lanzar `teleop`:
``` bash
roslaunch create_robot create_teleop_key.launch
```

Si no se mueve el robot al enviar comandos de movimiento, finalice y vuelva a iniciar el driver.

## Prueba de cámara de profundidad (*Intel Realsense D435*)

[Robot] Ejecute el nodo de la cámara de profundidad:
``` bash
roslaunch create_camera create_camera_d435.launch
```

Debe mostrar algo como:
```
...
[ INFO] [1592511895.209749833]: insert Infrared to Stereo Module
[ INFO] [1592511895.482399631]: SELECTED BASE:Depth, 0
 18/06 15:24:55,599 WARNING [140671377729280] (messenger-libusb.cpp:42) control_transfer returned error, index: 768, error: No data available, number: 61
 ...
```

Si muestra un INFO sin contenido, y no avanza, o dice que no encontró la cámara, reinicie el nodo, o desconecte y vuelva a conectar la cámara.

[PC] Inicie *RViz* para confirmar la imagen de profundidad:

``` bash
roslaunch create_rviz create_rviz_view_d435.launch
```

Algunas veces no muestra la nube de puntos, si esto ocurre, desactive y vuelva a activar el ítem *DepthCloud*.

La visualización debe presentarse así:

![rviz_d435.png](https://github.com/edgarcamilocamacho/create_xl/blob/master/doc/img/rviz_d435.png?raw=true)

*Nota:* Esta visualización está referenciada al frame *base_link*, así que si el robot se mueve, dicho desplazamiento no se verá reflejado en *RViz*.

## Prueba de cámara de seguimiento (*Intel Realsense T256*) y odometría visual

Finalice RViz, si se encuentra abierto.

[Robot] Ejecute el nodo de la cámara de seguimiento:
``` bash
roslaunch create_camera create_camera_t265.launch
```

Debe mostrar algo como:
```
...
[ INFO] [1592512640.819140598]: SELECTED BASE:Pose, 0
[ INFO] [1592512640.932357984]: RealSense Node Is Up!
[ INFO] [1592512640.990396968]: Subscribing to in_odom topic: /odom_wheels_fixed
 ...
```

Si muestra un INFO sin contenido, y no avanza, o dice que no encontró la cámara, reinicie el nodo, o desconecte y vuelva a conectar la cámara.

[PC] Inicie *RViz* para confirmar la imagen de profundidad:

``` bash
roslaunch create_rviz create_rviz_view_d435.launch
```

Si la cámara de profundidas está ejecutándose, se mostrará también su nube de puntos. Algunas veces no muestra la nube de puntos, si esto ocurre, desactive y vuelva a activar el ítem *DepthCloud*.

La visualización debe presentarse así:

![rviz_d435.png](https://github.com/edgarcamilocamacho/create_xl/blob/master/doc/img/rviz_t265.png?raw=true)

Teniendo en cuenta que esta visualización se encuentra referenciada al frame *odom*, al desplazar el robot, tanto el modelo del robot como el láser y la nube de puntos de desplazarán igualmente.

## Mapeo manual

Finalice RViz, si se encuentra abierto.

[PC o Robot] Teniendo andando los nodos del movimiento manual, el *teleop*, y las dos cámaras, ejecute el siguiente *launchfile* (*Importante:/ ésto eliminará el último mapa creado, si no quiere esto, haga una copia de seguridad del archivo `~/.ros/rtabmap.db`):

``` bash

```

Los nodos de *rtabmap* arrancarán, y debe mostrarse algo como lo siguiente:

```

```

Si aparece el mensje `Did not receive data since 5 seconds! Make sure the input topics are published`, asegúrese que los siguientes tópicos se están publicando:

* `/odom`
* `/camera/color/image_raw`
* `/camera/aligned_depth_to_color/image_raw`
* `/camera/color/camera_info`
* `/scan`

[PC] Si todo está bien, inicie la visualización para el mapeo:

``` bash

```

A través de la terminal del *teleop*, navegue manualmente por el entorno a mapear, tanto el mapa 3D como el 2D proyectado empezarán a formarse, como se muestra a continuación:

***Rviz Mapeando***

También puede observar este video: [mapeo con rtabmap]().

Se puede desactivar el mapa 3D para poder observar el 2D, sin embargo, al volver a activar el 3D, este no aparecerá hasta que ....

Para finalizar el mapeo, simplemente finalice el *launchfile* llamado *----* (presionando *Ctrl+C* en la terminal correspondiente). El mapa se guardará automáticamente en `~/.ros/rtabmap.db`, considere crear una copia de dicho archivo, ya que será reaamplazado al lanzar de nuevo el *launchfile* de mapear.

## Localización

Finalice RViz, si se encuentra abierto.

[PC o Robot] Lance el *launchfile* de localización:
``` bash

```

Los nodos de *rtabmap* arrancarán, y debe mostrarse algo como lo siguiente:

```

```

Si aparece el mensaje `Did not receive data since 5 seconds! Make sure the input topics are published`, asegúrese que los siguientes tópicos se están publicando:

* `/odom`
* `/camera/color/image_raw`
* `/camera/aligned_depth_to_color/image_raw`
* `/camera/color/camera_info`
* `/scan`

[PC] Si todo está bien, inicie la visualización para la localización:

``` bash

```

La visualización se debe presentar así:

***Rviz Mapeando***

También puede observar este video: [localización con rtabmap]().

La posición inicial debe ser detectada automáticamente. A través de la terminal del *teleop*, navegue manualmente y verifique que se localiza correctamente. Por defecto se muestra únicamente el mapa 3D, para cargar el 3D,  ....

## Navegación (en proceso)

Opción 1:
``` bash

```

Opción 2:
``` bash

```

