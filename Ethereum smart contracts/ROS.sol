/***
 * Very basic message, do not contain anything.
 * Parent of all messages.
 ***/
contract Message {}

/***
 * Message handler interface should be implemented 
 * by any message handlers sended to Subscriber.
 ***/
contract MessageHandler {
    /*** Public methods ***/
    function incomingMessage(Message msg);
}

/***
 * Entry point of typed and named messages.
 ***/
contract Topic {
    /*** Contract members ***/
    // The name of ROS topic, e.g '/hello'
    string public name;
    // The type of topic message, e.g. 'std_msgs/UInt8'
    string public messageType;
    /*** Public constructor ***/
    function Topic(string _name, string _type) {
        name = _name;
        messageType = _type;
    }
}

/***
 * The subscriber contract.
 * Implements message subscribing action,
 * gives message transactions and runs the Runnable callback.
 * TODO: Security questions.
 ***/
contract Subscriber is Topic {
    /*** Private members ***/
    MessageHandler callback;
    /*** Service events & functions ***/
    function SubMessage(Message _msg) public {
        callback.incomingMessage(_msg);
    }
    /*** Public constructor ***/
    function Subscriber(string _name,
                        string _type,
                        MessageHandler _callback) Topic(_name, _type) {
        callback = _callback;
    }
}

/***
 * The publisher contract.
 * Implements message publish action,
 * gives message and emit the event.
 ***/
contract Publisher is Topic {
    /*** Service events & functions ***/
    event PubMessage(address msg);
    /*** Public constructor ***/
    function Publisher(string _name, string _type) Topic(_name, _type) {
    }
    /*** Public methods ***/
    // Publish new message
    function publish(Message _msg) {
        PubMessage(_msg);
    }
}

/***
 * The main ROS bridge contract.
 * Should be super for any ROS compatible contracts.
 ***/
contract ROSCompatible {
    /*** Contract members ***/
    // List of all contract publishers
    Publisher[]  public publishers;
    // List of all contract subscribers
    Subscriber[] public subscribers;
    /*** Public methods ***/
    // Create a new Subscriber instance
    function mkSubscriber (string _name,
                            string _type,
                            MessageHandler _callback) returns (Subscriber) {
        var subscriber = new Subscriber(_name, _type, _callback);
        subscribers[subscribers.length++] = subscriber;
        return subscriber;
    }
    // Create a new Publisher instance
    function mkPublisher (string _name, string _type) returns (Publisher) {
        var publisher = new Publisher(_name, _type);
        publishers[publishers.length++] = publisher;
        return publisher;
    }
}
