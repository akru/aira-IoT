# AIRA ROS Bridge

This package provide bridging interface for the [Ethereum](https://www.ethereum.org) smart contracts to [ROS](http://www.ros.org).

## Description

The module is a part of AIRA IoT project. The second part is a Solidity contracts 
implemented [there](https://github.com/aira-dao/aira-IoT/blob/bridge/Ethereum%20smart%20contracts/ros.sol). The aim of this modules is make your smart contracts be able to interaction 
with Robot Operating System nodes, applications, robots, etc.

## Usage

### Solidity contracts

The first, is a writing smart contract that child of *ROSCompatible* contract.
Examples can be found [there](https://github.com/aira-dao/aira-IoT/tree/bridge/Ethereum%20smart%20contracts/example).

**!!!WARNING!!! Mining contracts and other interaction with Ethereum network is taking ethers. Use self created network for any testing. !!!WARNING!!!**

After the contract is writen do mining there and get contract address.

### ROS bridge

The second, you need ROS be installed, [installation instructions](http://wiki.ros.org/ROS/Installation). And also [ROS Bridge Suite](http://wiki.ros.org/rosbridge_suite) and [Node.js](https://nodejs.org) should be installed.

The next instructions:

    $ git clone https://github.com/aira-dao/aira-IoT.git
    $ cd aira-IoT/ROS/aira_ros_bridge && npm install

Node.js package manager do install deps.

[Run ROS Bridge Suite](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge) server in other terminal:

    $ roslaunch rosbridge_server rosbridge_websocket.launch

Run `start.js` with your mined contract address:

    $ node start.js 0xdc5e34c48d807cf2377cc93d0d4c711c8e2ac88d 
    util.debug: Use console.error instead
    DEBUG: ROSLib uses utf8 encoding by default.It would be more efficent to use ascii (if possible)
    Contract: 0xdc5e34c48d807cf2377cc93d0d4c711c8e2ac88d
    Publishers:
    /value :: std_msgs/Int64
    Subscribers:
    /add :: std_msgs/Int64
    Connected to websocket server.
