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
A színfelismerést az órai anyag átírásával valósítottuk meg. A szimulált kamera egy Ros node-on keresztül folyamatosan frissíti a képet, amit küld.
