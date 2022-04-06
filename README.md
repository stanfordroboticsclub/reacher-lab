# Reacher Sim
Simulation and Reinforcement Learning for Reacher Robot

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
conda create --name reacher python=3.8
conda activate reacher
pip install ray arspb
```

## Getting the code ready
```bash
git clone https://github.com/stanfordroboticsclub/reacher-lab.git
cd reacher-lab
pip install -e .
```
Then to verify the installation, run
```bash
python3 reacher/reacher_env_example.py
```
You should see the PyBullet GUI pop up and see Reacher doing an exercise.
<br/>