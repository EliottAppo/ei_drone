# EI Semi autonomous navigation of UAVs : Mercury Team

## Team members

-   **Eliott DUMONT**
-   **Hanna OBEID**
-   **Nour EL HASSAN**
-   **Samer LAHOUD**

<br />
<br />

## Getting started

-   Create a link in your ROS workspace, so that the ROS package sees a directory that is directly the directory in your git repository

```bash
$ cd <some-path>/my_ros_workspace/src
$ ln -s <some-path>/my_git_repo/my_ros_package
```

-   To install `visual_package_lib` go to it's working directory then run

```bash
$ python3 setup.py install --user
```

-   Build the workspace

```bash
$ catkin build
```

<br />
<br />

## Joypad mapping

-   ### Direct Mode

    -   `AXIS_RIGHT_FRONT` : Deadman button (Hold -> **TakeOff**, Release -> **Land**)
    -   `BUTTON_A` : Put the drone in **Hover** mode
    -   `AXIS_LEFT_PAD_VERTICAL` **_Up_** : Move the drone **Forward**
    -   `AXIS_LEFT_PAD_VERTICAL` **_Down_** : Move the drone **Backward**
    -   `AXIS_LEFT_PAD_HORIZONTAL` **_Left_** : Turn (over the `z` axis) the drone to the **Left**
    -   `AXIS_LEFT_PAD_HORIZONTAL` **_Right_** : Turn (over the `z` axis) the drone to the **Right**
    -   `AXIS_RIGHT_PAD_VERTICAL` **_Up_** : Move (over the `z` axis) the drone to the **Up**
    -   `AXIS_RIGHT_PAD_VERTICAL` **_Down_** : Move (over the `z` axis) the drone to the **Down**
    -   `AXIS_RIGHT_PAD_HORIZONTAL` **_Left_** : Move the drone to the **Left**
    -   `AXIS_RIGHT_PAD_HORIZONTAL` **_Right_** : Move the drone to the **Right**
    -   `AXIS_CROSS_HORIZONTAL` **_Left_** : U-Turn in the counter clockwise direction
    -   `AXIS_CROSS_HORIZONTAL` **_Right_** : U-Turn in the clockwise direction
    -   `BUTTON_X` : Slide the drone in the **_Left_** direction
    -   `BUTTON_B` : Slide the drone in the **_Right_** direction

<br/>

-   ### Logical Mode:

    `AXIS_LEFT_FRONT` : Enter logical mode (Hold -> **Enter**, Release -> **Leave**)

    -   `AXIS_LEFT_PAD_VERTICAL` **_Up_** : Move the drone forward following the **Vanishing Point**
    -   `AXIS_LEFT_PAD_HORIZONTAL` **_Left_** : Turn the drone 90 degrees in the clockwise direction and move the drone to the left direction towards the **Vanishing Point**
    -   `AXIS_LEFT_PAD_HORIZONTAL` **_Right_** : Turn the drone 90 degrees in the counter clockwise direction and move the drone to the right direction towards the **Vanishing Point**
    -   `AXIS_RIGHT_PAD_HORIZONTAL` **_Left_** : Turn the drone 90 degrees in the clockwise direction and move the drone to the left until it finds a door, once found, the drone enters it
    -   `AXIS_RIGHT_PAD_HORIZONTAL` **_Right_** : Turn the drone 90 degrees in the counter clockwise direction and move the drone to the right until it finds a door, once found, the drone enters it
