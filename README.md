# SafeSetAlgorithm
This repo contains implementations of safe set algorithm (SSA) and interactive simulations. 

C. Liu, and M. Tomizuka, "Control in a safe set: addressing safety in human-robot interactions", in Dynamic Systems and Control Conference. ASME, 2014, p. V003T42A003. [Preprint](http://web.stanford.edu/~cliuliu/files/dscc14.pdf)

## SCARA_Interactive

run two_agent.m to play with a SCARA robot arm.

> Place cursor on top of the blue circles for 3 seconds for calibration (one circle will be at the center, the other will be at the up right corner)
>
> You control the blue agent to pick the blue dots. The SCARA robot is in red. It tries to pick the red dots.
>
> The robot behavior is specified in robot_move.m, which implements on the safe set algorithm.
>
> You may change the setting in two_agent.m to test in diverse environments.

![](https://github.com/changliuliu/SafeSetAlgorithm/blob/master/SCARA_Interactive/outcome.gif)
