# AIRA IoT ROS Tutorials

## Simple publisher

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
