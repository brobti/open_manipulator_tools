# 05 Telemanipulátor - Amőba

## Migrálás, setup

A tanszéki nagy gépre telepítésre került az Ubuntu 20.04.3 és a ROS Noetic (1.15.13) distroja.
Telepített package-ek:
- ros-noetic-ros-controllers 
- ros-noetic-gazebo 
- ros-noetic-moveit 
- ros-noetic-industrial-core
- ros-noetic-dynamixel-sdk 
- ros-noetic-dynamixel-workbench
- ros-noetic-robotis-manipulator
- git clone -b noetic-devel-mod https://github.com/brobti/open_manipulator.git
- git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
- git clone -b noetic-devel https://github.com/ROBOTIS-GIT/open_manipulator_simulations.git
- git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
- ros-noetic-joint-state-publisher-gui
- git clone -b telemanipulator https://github.com/brobti/open_manipulator_controls.git
- git clone -b ticTacToe https://github.com/brobti/open_manipulator_tools.git
- git clone https://github.com/MOGI-ROS/gazebo_ros_link_attacher.git
- git clone https://github.com/MOGI-ROS/open_manipulator_ikfast_plugin

### Robot fizikai irányításának parancsai

Billentyűzettel:
```
roscore
```
```
roslaunch open_manipulator_controller open_manipulator_controller.launch
```
```
roslaunch open_manipulator_description open_manipulator_rviz.launch
```
```
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```
GUI-val:
```
roscore
```
```
roslaunch open_manipulator_controller open_manipulator_controller.launch
```
```
roslaunch open_manipulator_description open_manipulator_rviz.launch
```
```
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```
RVizben: Timer Start -> adatok bevitele -> Send
### Robot szimulációs irányításának parancsai

Billentyűzettel:
```
roscore
```
```
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```
```
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch 
```
```
roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch
```
Gazebóban alul a play gombra kell kattintani, hogy elinduljon a szimuláció.

GUI-val:
```
roscore
```
```
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```
```
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```
```
roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch
```
MoveIt!-tal:
```
roscore
```
```
roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false
```
```
roslaunch open_manipulator_controllers joint_trajectory_controller.launch 
```
Valós kamera elindítása:
Feltéve, hogy a felhasznált Rasberry Pi kapcsolódik a wifi hálózatra, csatlakozzunk rá, ez lesz a '[PI terminal]':

IP: 10.0.0.11

Jelszó: turtlebot

```
[PI terminal] ssh ubuntu@10.0.0.11
[PI terminal] rosrun mecanum_anomaly_det StreamCam.py
```
### Bugok
- rostopic pub /option std_msgs/String "print_open_manipulator_setting" -> nem írja ki az infókat a controlleres terminálablakba
- teleop_keyboard néha random lefagy -> indítsd újra a controllert és a teleop_keyboardot is!
- gazeboban a robot megfogója open állapotban ugrál, closed állapotban nem
- gravity compensationhöz Dynamixel Wizardban kéne beállítani valamit, amit nem tudtam telepíteni (nekünk nem is feltétlen kell)
### Rövid összefoglalás
A projektünk során Robotis OpenMANIPULATOR-X robottal valósítottunk meg egy színfeismeréssel működő amőbaprogramot, aminek során a robot lejátszik magával egy amőba játszmát, és minden lépés előtt megnézi és feldolgozza a jelenlegi állást. Azért nem a korábbi lépések elmentését használtuk a következő lépés eldöntéséhez, mert így a projekt átalakítható később olyanra, ahol a robot ember ellen játszik.
A játék pályája 3x3-as, a robot emellől veszi fel a bábukat, 4 kéket és 4 pirosat. Ezeket a Polimertechnológia Tanszék készítette el számunkra. A projekt 3 fő részből áll, a következőkben ezeken fogunk végigmenni.
### Forkolt repositoryk
Kezdetben egy GitLab repositoryban próbáltunk meg dolgozni, de oda nem tudtuk forkolni a hivatalos openmanipulator repositorykat, ezért áttértünk githubra.
A package-ek a dokumentum elején láthatók.
### Színfelismerés
A színfelismeréshez kiinduló projektnek a https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors 5. pontjában használt kódból indultunk ki. A szimulált és a valós kamera a 'head_camera/image_raw' Ros topic-on keresztül folyamatosan publish-olja a képet, erre csatlakozik az `open_manipulator/open_manipulator_controller/scripts/color_recognition.py` subscribere.
Az implementált color recognition open_cv használatával két, előre beállított RGB értéket keres a képen. Az ismert RGB értékek alapján `binary treshold` algoritmussal mindhárom színcsatornára elkészíti a maszkot, majd ezek metszetéből meghatározza a tényleges bináris képet.
A bináris képből az open_cv beépített `cv2.findContours()` algoritmusával meghatározzuk a bábuink felületét és azok középpontjait.
A megharározott középpontokat az egyszerűség kedvéért egy String-be összefűzve publisholjuk a `/color_recognition` topicra. Ezen felül a bináris képeket elhelyeztük a nyers képre, és azt vizuálisan megjelenítettük a könnyű kezelhetőség érdekében.
#### A színfelismerés elindítása:
Szimulációval:
```
[PC terminal 1] roscore
[PC terminal 2] roslaunch open_manipulator_controller open_manipulator_controller.launch
[PC terminal 3] rosrun open_manipulator_controller color_recognition.py
```

Valós kamerával:
```
[PC terminal 1] roscore
[PI terminal] rosrun mecanum_anomaly_det StreamCam.py
[PC terminal 2] rosrun open_manipulator_controller color_recognition.py
```
### Inverz kinematika
Az inverz kinematika tényleges implementációja már a https://github.com/MOGI-ROS/open_manipulator_tools repositoryban megtörtént. Ebből készítettük a https://github.com/brobti/open_manipulator_tools.git forkot, majd ennek a ticTacToe branch-óét. Ezen a branchen került implementálásra a robot és az amőbázó script közötti kommunikációt megvalósító 'action server'. 
# To do
- [ ] kép a színfelismerésről
