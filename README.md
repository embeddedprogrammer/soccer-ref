Robot Soccer Referee
====================

This ROS package contains a PyQt GUI (see screenshot below) whose job is to publish the `GameState` message, which all robot soccer teams (and the [soccersim](https://github.com/embeddedprogrammer/soccersim) Gazebo simulator) will consume.

Inspired by the [Chicken McThuggets Command Center](https://www.youtube.com/watch?v=bMZNFNGh_Pk&feature=youtu.be).

### `GameState` Message ###

The message definition contains the following:

```bash
int32 home_score            # score of the home/away team
int32 away_score            #
int32 home_bot_count        # how many robots will the home/away team be playing with?
int32 away_bot_count        # 
int32 remaining_seconds     # how many more seconds are remaining in this half?
bool play                   # Play/Pause -- robots should freeze in place when false
bool reset_field            # Robots should go to their home positions and freeze while true
bool second_half            # Second half of match. Robots should switch sides.
```

### Adding this Package to your Project ###

To access the `GameState` message in your own ROS package, you must add this package to your `catkin_ws` workspace folder. This will also allow you to run the `soccerref` on your own during testing (see below).

Start by cloning this repo into your `catkin_ws/src` directory. It is best to `git clone` the code so that as changes are released you can simply `git pull` to update.

```bash
$ cd catkin_ws/src
$ git clone https://github.com/embeddedprogrammer/soccerref.git
```

Your `catkin_ws/src` directory should look like:

```bash
catkin_ws/
  src/
    soccerref
    your_team_package/
      CMakeLists.txt
      package.xml
```

Next, you will want to edit your team's `CMakeLists.txt` and `package.xml` so that catkin knows that your package depends on `soccerref`:

```cmake
# CMakeLists.txt

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ...
  soccerref
)

...

catkin_package(
 CATKIN_DEPENDS roscpp ... soccerref
)
```

```xml
<!-- package.xml -->

<buildtool_depend>catkin</buildtool_depend>
...
<build_depend>soccerref</build_depend>
...
<run_depend>soccerref</run_depend>
```

Now that your package knows it needs `soccerref`, you can run `catkin_make` to generate the ROS message for `GameState.msg`:

```bash
$ catkin_make   # make sure you are in 'catkin_ws' and not 'catkin_ws/src'
```

### Starting the `soccerref` ###

Use `roslaunch`:

```bash
$ roslaunch soccerref simref.py
```

### Screenshots ###

The main `soccerref` GUI:

![soccerref gui](https://github.com/embeddedprogrammer/soccerref/wiki/assets/soccerref.png)

--------------------------

The `soccerref` GUI refereing a simulated match:

![soccerref match](https://github.com/embeddedprogrammer/soccerref/wiki/assets/soccerref_sim_action.png)