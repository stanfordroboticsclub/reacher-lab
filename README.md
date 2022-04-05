# Pupper Sim
Simulation and Reinforcement Learning for DJI Pupper v2 Robot

## System setup
### Operating system requirements
* Mac
* Linux
* Windows (untested, not recommended)

### Mac-only setup
Install xcode command line tools.
```bash
xcode-select --install
```
If you already have the tools installed you'll get an error saying so, which you can ignore.

### Conda setup
Install [miniconda](https://docs.conda.io/en/latest/miniconda.html), then
```
conda create --name rl_pupper python=3.7
conda activate rl_pupper
pip install ray arspb
```

## Getting the code ready
```bash
git clone https://github.com/stanfordroboticsclub/reacher-lab.git
cd puppersim
pip install -e .
```
Then to verify the installation, run
```bash
python3 reacher/reacher_env_example.py
```
You should see the PyBullet GUI pop up and see Reacher doing an exercise.
<br/>