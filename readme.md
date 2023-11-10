# AlphaCheese - The alpha version of AutoChess (2023)
A project that runs the robotic arm to do an autonomous pick and place the chess using stockfish as the chess engine.

Some contraints:
- Robotic arm will be playing as black
- Camera is set to be on top of the robotic arm (shown in the image)
- Chess board position has to be fixed as the inverse kinematics are pre-calculated


## 1. Preparation
### 1.1 Clone the repo
cd ~
mkdir robotic_arm_ws
cd robotic_arm_ws
git clone this_repo
mv MA4825-AlphaCheese src
cd ~/robotic_arm_ws
colcon build

### 1.2 Prepare a roboflow account
Create an account in roboflow to access the model that is uploaded to roboflow universe. After creating an account, copy your api-key to run the code.

Export the roboflow api key to your environment variable to be used inside the python code

1. Temporary env
   `export ROBOFLOW_API_KEY='your_api_key_here'`

2. Permanently add to your .bashrc which is bind to your user
   `echo "export ROBOFLOW_API_KEY='your_api_key_here'" >> ~/.bashrc`
   `source ~/.bashrc`

This env will be used inside chessVision.py in comp_vision package to call the model which is uploaded to the roboflow

## 2. Execution
Call the launch file by:
`ros2 launch ~/robotic_arm_ws/src/launch all.launch.py`

Or to be more systematic:
`ros2 run comp_vision chessVision` -> wait until it is ready to receive message from chess Algo, indicated by 'False' message printed which means not white's turn
`ros2 run motion_pkg moveAndTake` 
`ros2 run algorithm chessAlgo`
