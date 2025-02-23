# Autonomous Exploration with Expectation-Maximization
*Now in Python 3!*

Random Environment         |  Structured Environment
:-------------------------:|:-------------------------:
![](./figures/isrr2017_random.gif)  |  ![](./figures/isrr2017_structured.gif)

J. Wang and B. Englot, "Autonomous Exploration with Expectation-Maximization," International Symposium on Robotics Research, Accepted, To Appear, December 2017.

# Usage

## Build
```
mkdir build && cd build
cmake ..
make
```

## Install Python dependencies
```
pip3 install matplotlib scipy numpy
```

## Add library to python path and run
```
export PYTHONPATH=/path/to/repo/build
python ../script/isrr2017.py
```