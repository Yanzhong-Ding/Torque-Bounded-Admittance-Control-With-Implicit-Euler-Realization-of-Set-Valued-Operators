# Torque-Bounded Admittance Control with Meta-Learning Adaptive Algorithm

This repository accompanies the paper "Torque-Bounded Admittance Control with
Meta-Learning Adaptive Algorithm". And the complete code will be made available after the publication of this manuscript.


## Getting started

Ensure you are using Python 3 and Ubuntu operating system. Clone this repository and install the packages listed in `requirements.txt`. In particular, this code uses [JAX](https://github.com/google/jax).


## Reproducing results

Training data, trained parameters, and test results are all conveniently saved in this repository, since it can take a while to re-generate them. 

Training data can be generated with the command `python generate_data.py`.

Parameters can then be trained with the command `./train_rk38.sh`. This will take a while.

Finally, test results can be produced with the commands `test_single_AdmDI.py` , `python test_DISM.py` and `./test_single_ML.sh` respectively. 


## References
[1] Richards S M, Azizan N, Slotine J J, et al. Control-oriented meta-learning[J]. The International Journal of Robotics Research, 2023, 42(10): 777-797.

[2] X. Yuan, Y. Ding, X. Xiong and Y. Lou, "Torque-Bounded Admittance Control With Implicit Euler Realization of Set-Valued Operators," in IEEE/ASME Transactions on Mechatronics, doi: 10.1109/TMECH.2023.3342479. 

[3] O’Connell M, Shi G, Shi X, et al. Neural-fly enables rapid learning for agile flight in strong winds[J]. Science Robotics, 2022, 7(66): eabm6597.
