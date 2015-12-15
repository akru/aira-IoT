# AIRA IoT ROS Tutorials

## Simple publisher

This example makes only one `Publisher` object that provide ability to
publish message to ROS topic.

### Solidity contract

See the [example contract](https://raw.githubusercontent.com/aira-dao/aira-IoT/master/Ethereum%20smart%20contracts/example/simple_publisher.sol):

    import 'ROS';

Importing the basic ROS integration contract library.
`ROS` library defines some structures as `Publisher`, `Subscriber`, `Message` and etc.

    contract StdString is Message {
        string public data;
        function StdString(string _data) {
            data = _data;
        }
    }

Message definition contract cotaints only one member `data` with simple type `string`.
ROS messages will be converted to `Message` contract and submitted by AIRA ROS Bridge,
or `Message` contracts will be converted to ROS message and published to associated topics.

    contract SimplePub is ROSCompatible {

The next is a main contract definition, this contract should be child of `ROSCompatible` contract.
The `ROSCompatible` contract contrains service members that used by AIRA ROS Bridge.

        Publisher      myPub;
        function SampleContract() {
            // Create a new publisher by topic name and type
            myPub = mkPublisher('/hello', 'std_msgs/String');
        }

In this code contract constructor creates `Publisher` by `ROSCompatible` public 
method `mkPublisher` and arguments *topic name* and *message type*. `Publisher`
instance saved in private member for the future.

        function echo(string str) {
            // Publish new message by received string
            myPub.publish(new StdString(str));
        }
    }

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

    $ node start.js 0xc52db6edc294ecf434a322b93e09d87d1aae80ae
    util.debug: Use console.error instead
    DEBUG: ROSLib uses utf8 encoding by default.It would be more efficent to use ascii (if possible)
    Contract: 0xc52db6edc294ecf434a322b93e09d87d1aae80ae
    Publishers:
    /echo :: std_msgs/String
    Subscribers:

Described on the top contract have a simple ROS interface:

Publishers:

* `/hello` :: **std_msgs/String** 

Using `rostopic` tools try to subscribe to `/hello` topic:

    $ rostopic echo /hello

This tool should print received strings to screen.
Try to publish string by `sendTransaction` method in `geth console`:

    > var contract = eth.contract(simple_publisher_abi).at("0xc52db6edc294ecf434a322b93e09d87d1aae80ae")
    > contract.echo.sendtransaction("Hello world!", {from: eth.accounts[0], gas: 500000})

After some time `rostopic` console show:

    WARNING: topic [/hello] does not appear to be published yet
    data: Hello world!
    ---

## Publisher/Subscriber combination

The next example combine `Subscriber` with `Publisher` and can
subscribe to topics and pubish new messages too.

Task is write simple integrator with ROS interface:

* `/add` :: **std_msgs/Int64** - Input integer value
* `/value` :: **std_msgs/Int64** - Sum of all input values

### Solidity contract

See the [example](https://raw.githubusercontent.com/aira-dao/aira-IoT/master/Ethereum%20smart%20contracts/example/pubsub.sol):

    contract Integrator is MessageHandler {
        Publisher myPub;
        int64     value;

Contract `Integrator` can handle incoming messages and have two private members:
`Publisher` and `int64`. `value` member used for storing current value and
publish into `myPub`.

        function Integrator(Publisher _pub) {
            myPub = _pub;
            value = 0;
        }

Constructor just copy publisher from arguments and init `value` by zero.

        function incomingMessage(Message _msg) {
            value += MsgInt64(_msg).data();
            myPub.publish(new MsgInt64(value));
        }
    }

This method of `MessageHandler` interface takes `Message` argument.
From incoming message taked `data` and added to `value` member.
The next `myPub` publish `value` wrapped by `MsgInt64` object.

    contract PubSub is ROSCompatible {
        function PubSub() {
            var pub = mkPublisher("/value", "std_msgs/Int64");
            var hdl = new Integrator(pub);
            mkSubscriber("/add", "std_msgs/Int64", hdl);
        }
    }

The last `PubSub` contract is `ROSCompatible` and have only one constructor.
Constructor create publisher, message handler with publisher as argument and
subscriber with message handler as argument.

### ROS interaction

Lets mine `PubSub` contract and start AIRA ROS Bridge with them address:

    $ node start.js 0x8dc4bb82cbbdd0980cc8bd57c7ddc9682185acb7
    util.debug: Use console.error instead
    DEBUG: ROSLib uses utf8 encoding by default.It would be more efficent to use ascii (if possible)
    Contract: 0x8dc4bb82cbbdd0980cc8bd57c7ddc9682185acb7
    Publishers:
    /value :: std_msgs/Int64
    Subscribers:
    /add :: std_msgs/Int64
    Connected to websocket server.

Open terminal and start `rostopic` message subscriber node:

    $ rostopic echo /value

The next publish by `rostopic` some messages to `/add`:

    $ rostopic pub /add std_msgs/Int64 'data: 10'
    publishing and latching message. Press ctrl-C to terminate

    $ rostopic pub /add std_msgs/Int64 'data: -2'
    publishing and latching message. Press ctrl-C to terminate

After some time the subscriber `rostopic` says:

    $ rostopic echo /value
    WARNING: topic [/value] does not appear to be published yet
    data: 10
    ---
    data: 8
    ---
