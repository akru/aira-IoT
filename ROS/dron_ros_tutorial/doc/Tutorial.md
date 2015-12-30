# AIRA IoT ROS Tutorials

## Simple publisher

This example makes only one `Publisher` object that provide ability to
publish message to ROS topic.

### Solidity contract

See the [example contract](https://raw.githubusercontent.com/aira-dao/aira-IoT/master/Ethereum%20smart%20contracts/example/simple_publisher.sol):

```JavaScript
import 'ROS';
```

Importing the basic ROS integration contract library.
`ROS` library defines some structures as `Publisher`, `Subscriber`, `Message` and etc.

```JavaScript
contract StdString is Message {
    string public data;
    function StdString(string _data) {
        data = _data;
    }
}
```

Message definition contract cotaints only one member `data` with simple type `string`.
ROS messages will be converted to `Message` contract and submitted by AIRA ROS Bridge,
or `Message` contracts will be converted to ROS message and published to associated topics.

```JavaScript
contract SimplePub is ROSCompatible {
```

The next is a main contract definition, this contract should be child of `ROSCompatible` contract.
The `ROSCompatible` contract contrains service members that used by AIRA ROS Bridge.

```JavaScript
    Publisher      myPub;
    function SampleContract() {
        // Create a new publisher by topic name and type
        myPub = mkPublisher('/hello', 'std_msgs/String');
    }
```

In this code contract constructor creates `Publisher` by `ROSCompatible` public 
method `mkPublisher` and arguments *topic name* and *message type*. `Publisher`
instance saved in private member for the future.

```JavaScript
    function echo(string str) {
        // Publish new message by received string
        myPub.publish(new StdString(str));
    }
}
```

The last method accepts `string` argument, packs this into `StdString` message container,
and call `publish` method of `Publisher` instance that publish ROS message to 
associated topic.

### ROS interaction

The first, [create](http://chriseth.github.io/browser-solidity/) Web3 deploy instructions
for the [example contract](https://raw.githubusercontent.com/aira-dao/aira-IoT/master/Ethereum%20smart%20contracts/example/simple_publisher.sol).
And put into the `geth console` window. After some time you see:

    Contract mined! address: 0xc52db6edc294ecf434a322b93e09d87d1aae80ae transactionHash: 0x4882579adef61f44fcef5847e93a5c998c81c30c9f7345f5c48e0f613a6e4bac

With contract address `0xc52db6edc294ecf434a322b93e09d87d1aae80ae`
you can start AIRA ROS Bridge now, full start instructions is [there](https://github.com/aira-dao/aira-IoT/tree/master/ROS/aira_ros_bridge):

```bash
$ node start.js 0xc52db6edc294ecf434a322b93e09d87d1aae80ae
util.debug: Use console.error instead
DEBUG: ROSLib uses utf8 encoding by default.It would be more efficent to use ascii (if possible)
Contract: 0xc52db6edc294ecf434a322b93e09d87d1aae80ae
Publishers:
/hello :: std_msgs/String
Subscribers:
```

Described on the top contract have a simple ROS interface:

Publishers:

* `/hello` :: **std_msgs/String** 

Using `rostopic` tools try to subscribe to `/hello` topic:

```bash
$ rostopic echo /hello
```

This tool should print received strings to screen.
Try to publish string by `sendTransaction` method in `geth console`:

```JavaScript
> var contract = eth.contract(simple_publisher_abi).at("0xc52db6edc294ecf434a322b93e09d87d1aae80ae")
> contract.echo.sendTransaction("Hello world!", {from: eth.accounts[0], gas: 500000})
```

After some time `rostopic` console show:

```bash
WARNING: topic [/hello] does not appear to be published yet
data: Hello world!
---
```

## Publisher/Subscriber combination

The next example combine `Subscriber` with `Publisher` and can
subscribe to topics and pubish new messages too.

Task is write simple integrator with ROS interface:

* `/add` :: **std_msgs/Int64** - Input integer value
* `/value` :: **std_msgs/Int64** - Sum of all input values

### Solidity contract

See the [example](https://raw.githubusercontent.com/aira-dao/aira-IoT/master/Ethereum%20smart%20contracts/example/pubsub.sol):

```JavaScript
contract Integrator is MessageHandler {
    Publisher myPub;
    int64     value;
```

Contract `Integrator` can handle incoming messages and have two private members:
`Publisher` and `int64`. `value` member used for storing current value and
publish into `myPub`.

```JavaScript
    function Integrator(Publisher _pub) {
        myPub = _pub;
        value = 0;
    }
```

Constructor just copy publisher from arguments and init `value` by zero.

```JavaScript
    function incomingMessage(Message _msg) {
        value += MsgInt64(_msg).data();
        myPub.publish(new MsgInt64(value));
    }
}
```
This method of `MessageHandler` interface takes `Message` argument.
From incoming message taked `data` and added to `value` member.
The next `myPub` publish `value` wrapped by `MsgInt64` object.

```JavaScript
contract PubSub is ROSCompatible {
    function PubSub() {
        var pub = mkPublisher("/value", "std_msgs/Int64");
        var hdl = new Integrator(pub);
        mkSubscriber("/add", "std_msgs/Int64", hdl);
    }
}
```

The last `PubSub` contract is `ROSCompatible` and have only one constructor.
Constructor create publisher, message handler with publisher as argument and
subscriber with message handler as argument.

### ROS interaction

Lets mine `PubSub` contract and start AIRA ROS Bridge with them address:

```bash
$ node start.js 0x8dc4bb82cbbdd0980cc8bd57c7ddc9682185acb7
util.debug: Use console.error instead
DEBUG: ROSLib uses utf8 encoding by default.It would be more efficent to use ascii (if possible)
Contract: 0x8dc4bb82cbbdd0980cc8bd57c7ddc9682185acb7
Publishers:
/value :: std_msgs/Int64
Subscribers:
/add :: std_msgs/Int64
Connected to websocket server.
```

Open terminal and start `rostopic` message subscriber node:

```bash
$ rostopic echo /value
```

The next publish by `rostopic` some messages to `/add`:

```bash
$ rostopic pub /add std_msgs/Int64 'data: 10'
publishing and latching message. Press ctrl-C to terminate
```

```bash
$ rostopic pub /add std_msgs/Int64 'data: -2'
publishing and latching message. Press ctrl-C to terminate
```

After some time the subscriber `rostopic` says:

```bash
$ rostopic echo /value
WARNING: topic [/value] does not appear to be published yet
data: 10
---
data: 8
---
```

## Dron GPS Destination contract

The task have a two parts:

* **path estimation** :: how cost to flight for this point? 
* **flight** :: dron flight and return back.

The [video](https://www.youtube.com/watch?v=nuf6JtocTTQ) of this task solution.

The ROS interface of *dron* is:

Subscribers:

* `/path_estimation/path` :: **dron_ros_tutorial/PathEstimate** - estimate cost of path
* `/dron_employee/target` :: **dron_ros_tutorial/SatPosition** - move dron to target point and go back

Publishers:

* `/path_estimation/cost` :: **dron_ros_tutorial/PathCost** - cost of path in ethers
* `/dron_employee/homebase` :: **dron_ros_tutorial/SatPosition** - dron current position

### Solidity contract

See the [gps-destination.sol](https://raw.githubusercontent.com/aira-dao/aira-IoT/master/Ethereum%20smart%20contracts/example/gps-destination.sol) contract: 

```JavaScript
contract EstimateListener is MessageHandler {
    GPSDestination parent;
    function EstimateListener(GPSDestination _parent) {
        parent = _parent;
    }

    function incomingMessage(Message _msg) {
        var cost = PathCost(_msg);
        parent.setEstimateCost(cost.ident(), cost.cost());
    }
}
```

This contract is path cost message handler, this call `parent` contract `setEstimateCost` method every new message is coming.

```JavaScript
contract HomebaseListener is MessageHandler {
    GPSDestination parent;
    
    function HomebaseListener(GPSDestination _parent) {
        parent = _parent;
    }
                        
    function incomingMessage(Message _msg) {
        SatPosition pos = SatPosition(_msg);
        parent.homebase(pos.longitude(), pos.latitude());
    }
}
```

This contract is homebase message handler, this call `parent` contract `homebase` method every new message is coming.

```JavaScript
contract GPSDestination is ROSCompatible { 
```

The `ROSCompatible` main contract is this.

```JavaScript
    EstimateListener estimateListener;
    HomebaseListener homebaseListener;
    Publisher estimatePub;
    Publisher targetPub;
```

Private members for message handlers and publishers.

```JavaScript
    address public currentCustomer;
    int256  public homebaseLongitude;
    int256  public homebaseLatitude;
    int256  public destinationLongitude;
    int256  public destinationLatitude;
    uint    public estimatesActualBefore;
```

Common dron information e.g. current customer, home and destination points.

        mapping (address => uint) customerEstimatesOf;

Customer to estimation request ID mapping.

```JavaScript 
    struct Estimate {
        address customerAddr;
        int256 destinationLongitude;
        int256 destinationLatitude;
        uint cost;
        uint actualBefore; 
    }

    Estimate[] public estimates; 
```

`Estimate` structure definition and array for storing path estimation requests.

```JavaScript
    event DroneComeback(uint estimateID);
    event EstimateCostReceive(uint estimateID, uint cost);
```

Two customer events: drone come back from mission, path cost is received from drone.

```JavaScript
    function GPSDestination(int256 _homebaseLongitude,
                            int256 _homebaseLatitude,
                            uint _estimatesActualBefore) {
        homebaseLatitude = _homebaseLatitude;
        homebaseLongitude = _homebaseLongitude;
        estimatesActualBefore = _estimatesActualBefore * 1 minutes;
    }
```

Constructor of contract, sets initial values of home point.

```JavaScript
    function initROS() returns (bool result) {
        estimatePub = mkPublisher('/path_estimation/path',
                                  'dron_ros_tutorial/PathEstimate');
        targetPub = mkPublisher('/dron_employee/target',
                                'dron_ros_tutorial/SatPosition');
```

Create two publishers for path estimation topic and dron target topic and save into private members for the future use.

```JavaScript
        estimateListener = new EstimateListener(this);
        homebaseListener = new HomebaseListener(this);
```

Create two message handlers for the path estimation and homebase cases.

```JavaScript
        mkSubscriber('/path_estimation/cost',
                     'dron_ros_tutorial/PathCost',
                     estimateListener);
        mkSubscriber('/dron_employee/homebase',
                     'dron_ros_tutorial/SatPosition',
                     homebaseListener);
```

Create two subscribers with previously instantied handlers.

```JavaScript
        return true;
    }
```

ROS communication initial method takes too much GAS and can not be placed in constructor. **Should be called after contract is created.**

```JavaScript
function homebase(int256 _currentLongitude, int256 _currentLatitude) returns(bool result) {
```

Takes two arguments: longitude and altitude of dron home position and sets contract members for this. Also reset current customer information and emit `DroneComeback` event.

```JavaScript
function setEstimateCost(uint _estimateID, uint _cost) returns(bool result) {
```
Takes two arguments: path estimation request ID and cost in ethers, saves cost into `estimates` array and emit `EstimateCostReceive` event.

```JavaScript
function setNewEstimate(int256 _destinationLongitude,
                        int256 _destinationLatitude) returns(uint estimateID) {
```

Takes two arguments: destination longitude and latitude and return path estimation request ID. This function create new item in `estimates` array with destination coords, timestamp and publish new message by `estimatePub` instance:

```JavaScript    
    var base = new SatPosition(homebaseLatitude, homebaseLongitude);
    var target = new SatPosition(_destinationLatitude, _destinationLongitude);
    estimatePub.publish(new PathEstimate(uint16(estimateID), base, target));
```

The last method is:

```JavaScript
function takeFlight() returns(bool result) {
    uint workEstimateID = customerEstimatesOf[msg.sender];
    Estimate e = estimates[workEstimateID];
    if(msg.value >= e.cost) {
```

This method checks amount of ethers sended with transaction to `takeFlight` method, when count is big or eq do publish the dron target:

```JavaScript
        targetPub.publish(new SatPosition(destinationLongitude, destinationLatitude));
```

In other case - return money to sender:

```JavaScript
    } else msg.sender.send(msg.value);
```

### ROS interaction

The *path estimation* task have solved by [path_estimator.py](https://github.com/aira-dao/aira-IoT/blob/master/ROS/dron_ros_tutorial/scripts/path_estimator.py) ROS node.

And the *target motion* task have solved by [quad_controller.py](https://github.com/aira-dao/aira-IoT/blob/master/ROS/dron_ros_tutorial/scripts/quad_controller.py) ROS node.

The package [dron_ros_tutorial](https://github.com/aira-dao/aira-IoT/tree/master/ROS/dron_ros_tutorial) is a valid ROS package and can be build by [catkin](http://wiki.ros.org/catkin/Tutorials/using_a_workspace).

```bash
$ mkdir -p catkin_ws/src && cd catkin_ws/src && catkin_init_workspace
$ cp -r /path/to/dron_ros_tutorial .
$ cd .. && catkin_make
$ source devel/setup.bash
```

After building you can just launch `apm_sitl.launch` for [local simulation cycle](https://github.com/aira-dao/aira-IoT/blob/master/ROS/dron_ros_tutorial/doc/Simulation.md).

```bash
$ launch dron_ros_tutorial apm_sitl.launch
```

The next is mine [gps-destination.sol](https://github.com/aira-dao/aira-IoT/blob/master/Ethereum%20smart%20contracts/example/gps-destination.sol) contract and taking the address. With contract address do call `initROS` method of contract for creating ROS interface of this.

```JavaScript
> gpsdestionationContract.initROS.sendTransaction({from: eth.accounts[0], gas: 3000000})
```

The last is start [AIRA ROS Bridge](https://github.com/aira-dao/aira-IoT/tree/master/ROS/aira_ros_bridge) with current contract address. 

```bash
$ node start.js 0x0ba4478113b307e5053366b4ef0d049801268a7e
util.debug: Use console.error instead
DEBUG: ROSLib uses utf8 encoding by default.It would be more efficent to use ascii (if possible)
Contract: 0x0ba4478113b307e5053366b4ef0d049801268a7e
Publishers:
/path_estimation/path :: dron_ros_tutorial/PathEstimate
/dron_employee/target :: dron_ros_tutorial/SatPosition
Subscribers:
/path_estimation/cost :: dron_ros_tutorial/PathCost
/dron_employee/homebase :: dron_ros_tutorial/SatPosition
```
