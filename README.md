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

### Screenshots ###

The main `soccerref` GUI:

![soccerref gui](https://github.com/embeddedprogrammer/soccerref/wiki/assets/soccerref.png)

--------------------------

The `soccerref` GUI refereing a simulated match:

![soccerref match](https://github.com/embeddedprogrammer/soccerref/wiki/assets/soccerref_sim_action.png)