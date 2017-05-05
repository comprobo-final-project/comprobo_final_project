# Project Nevo

### [Shane Kelly](https://github.com/shanek21), [Kevin Zhang](https://github.com/kzhang8850), [David Zhu](https://github.com/hdavidzhu)

Spring 2017

https://comprobo-final-project.github.io/


## Run

*Note: these steps require that you have a Neato or other mobile robot and are already connected to said robot. *

Change directory into the ROS package. Because of the way that sections of our code are imported into other sections, it is important that you run all commands from this directory.

`roscd comprobo_final_project`

Run the genetic algorithm on the desired task with the desired options.

`python -m scripts.tasks.<TASK_NAME> --<COMMAND>`

`<TASK_NAME>` can be replaced with:
- `goal_task`
- `colinear_task`
- `tag_task`

`<COMMAND>` can be replaced with:
- `train`
- `visualize`
- `gazebo`
- `real`

For example, to train an organism to complete the goal task, run

`python -m scripts.tasks.goal_task --train`
