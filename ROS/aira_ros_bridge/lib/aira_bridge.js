module.exports = bridge

/*** Libraries import ***/
var Web3   = require('web3');
var ROSLIB = require('roslib');
var ABI    = require('./abi');
var ROSMSG = require('./rosmsg');

/*** Libraries init ***/
var web3 = new Web3();
web3.setProvider(new web3.providers.HttpProvider('http://localhost:8545'));

var ros = new ROSLIB.Ros({url : 'ws://localhost:9090'});
ros.on('connection', function() {console.log('Connected to websocket server.')});

/*** Functions ***/
function topicOn(accessor, contract_abi, fun) {
    var ix = 0;
    while (accessor(ix) != "0x") {
        var address = accessor(ix++); 
        var eth_topic = web3.eth.contract(contract_abi).at(address);
        // Log debug info
        console.log(eth_topic.name() + " :: " + eth_topic.messageType());
        // Create a topic
        var topic = new ROSLIB.Topic({
            ros: ros,
            name: eth_topic.name(),
            messageType: eth_topic.messageType()
        });
        // Run user function
        fun(eth_topic, topic);
    }
}

function bridge(contract_address) {
    /* Load publishers */
    var ros_contract = web3.eth.contract(ABI.ros_compatible).at(contract_address);

    console.log("Contract: " + contract_address);
    console.log("Publishers:");
    topicOn(ros_contract.publishers, ABI.publisher, function(eth_topic, topic) {
        // Make event listener for eth topic publish action
        eth_topic.PubMessage({}, '', function(e, r) {
            if (!e) {
                // Take message by message address received from event
                var msg_type = eth_topic.messageType();
                var rosmsg = ROSMSG.load(msg_type);
                var msg = web3.eth.contract(rosmsg.abi).at(r.args.msg);
                // Publish ROS topic with converted message
                topic.publish(rosmsg.eth2ros(msg));
            }
            else
                console.log("Error: " + e);
        });
    });

    console.log("Subscribers:");
    topicOn(ros_contract.subscribers, ABI.subscriber, function(eth_topic, topic) {
        // Subscribe the ROS topic
        topic.subscribe(function (msg) {
            // Make mining the message contract
            ROSMSG.load(eth_topic.messageType()).ros2eth(msg, web3, function(e, address) {
                if (!e)
                    // Send transaction with message contract address
                    eth_topic.SubMessage.sendTransaction(address,
                        {from: web3.eth.accounts[0], gas: 500000});
                else
                    console.log("Error: " + e);
            });
        });
    });
}
