# Module 1&2: Forward and Inverse Kinematics for Robot Manipulators

This repository accompanies the class activities in modules 1&2 focusing on the following:
1. **Forward position kinematics (FPK)**
2. **Forward velocity kinematics (FVK)**
3. **Inverse position kinematics (IPK)**
4. Applied on three different arm configurations: **Two-DOF arm, SCARA robot, 5-DOF Hiwonder robot**

This repository provides the **visualization tool (viz tool)** for testing your kinematics modeling and analysis code.

## Viz Tool

<img src = "media/FPK.png">

## Setting up your PC

- You can complete this assignment on any computer OS: Mac, Windows, Linux, etc. All you need is Python 3 interpreter (code was tested using Python 3.10).

#### Step 0 (Optional): Install VScode
- Install Visual Studio Code (I strongly recommend using this, if you don’t already do. It’s the best IDE in my humble opinion)
- Follow the instructions [here to install.](https://code.visualstudio.com/download)



#### Step 1: Install Python 3 (if not already installed)
- First, check if you have Python3, to do that, open your terminal and type:
```bash
$ python3 --version     # <--- type this
Python 3.10.12          # <--- you should see something like this
```
- If you don’t have Python installed, follow this [tutorial here](https://realpython.com/installing-python/) to install it.


#### Step 2: Create a virtual environment
- This is technically optional, but I strongly recommend that you create a new python virtual environment for this course.
- Follow this [tutorial here](https://docs.python.org/3/tutorial/venv.html).


#### Step 3: Get this repository from Github
- I recommend you fork this repository to the account of one of your teammates and then you all can clone from the forked version.
- Follow [this tutorial](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/fork_and_clone_github_repository/Fork-and-Clone-From-GitHub.html) to understand how to fork and clone repositories


#### Step 4: Install all required Python packages
```bash
# first: make sure you have activated the virtual environment (if you used one). See step 2 tutorial

# cd to the project folder
$ cd arm-kinematics-module

# install all required packages from requirements.txt
$ pip install -r requirements.txt
```


### How to Run

- If setup worked well, you should be able to run the main script with the command below:
``` bash
$ python main_arm.py 
# this configures the two-DOF arm
```

- There are options to pass command-line arguments to configure the viz tool to other arm configurations, i.e., SCARA and 5-DOF arm

``` bash
$ python main_arm.py -h

usage: main_arm.py [-h] [--robot_type ROBOT_TYPE] 

options:
  -h, --help            show this help message and exit
  --robot_type ROBOT_TYPE
                        insert robot type, e.g., '2-dof', 'scara', '5-dof'
```
- Example, for SCARA robot:
```bash
$ python main_arm.py --robot_type scara
```

### Usage Guide

<img src = "media/arm-kinematics-viz-tool.png">


### Generative AI Use Disclosure
- Please make sure to briefly describe what and how generative AI tools were used in developing the contents of your work.
- Acceptable use:
    - To research a related topic to the subject at hand
    - As a substitute to "Stackoverflow" guides on quick programming how-tos, etc.
- Unacceptable use:
    - Directly copying large swaths of code from a ChatGPT response to a prompt in part or entirely related to the assignment's problem

For instance, I used ChatGPT in generating the docstrings for the functions in this repository.

