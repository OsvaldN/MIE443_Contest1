# MIE 443 - Contest 1 Deliverable

Date: Feb 23, 2021
Group: 20

## Team:

|       Name       | Student Number |
|:----------------:|:--------------:|
|   Stefan Albers  |   1003476204   |
| Mithun Jothiravi |   1002321258   |
|   Osvald Nitski  |   1002456987   |
|    David Rolko   |   1003037420   |

## Setup:
1. git clone this project into the `./src` directory within the `catkin_ws` workspace.

2. Partition current terminal into 4 terminals.

3. In *Terminal #1*, run the following to launch the simulation world:

    i) `$ catkin_make`
    
    ii) `$ source devel/setup.zsh`
    **NOTE:** If you use `bash` terminal, source the following: `$ source devel/setup.zsh` in all terminals.

    iii)`$ roslaunch mie443_contest1 turtlebot_world.launch world:=practice`

4. In *Terminal #2*, run the following to launch gmapping utility:

    i) `$ catkin_make`

    ii) `$ source devel/setup.zsh`

    iii) `$ roslaunch mie443_contest1 gmapping.launch`

5. In *Terminal #3*, run the following to launch contest1 code from Group 20.

    i) `$ catkin_make`

    ii) `$ source devel/setup.zsh`

    iii) `$ rosrun mie443_contest1 contest1`

6. In *Terminal #4*, run the following to launch Rviz mapping utility. 

    i) `$ catkin_make`

    ii) `$ source devel/setup.zsh`

    iii) `$ roslaunch turtlebot_rviz_launchers view_navigation.launch`

## Repository:

Entire project repo can be accessed [here](https://github.com/OsvaldN/MIE443_Contest1/tree/main).

