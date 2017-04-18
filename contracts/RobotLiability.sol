pragma solidity ^0.4.4;

import 'ROS/ROSBridge.sol';
import 'dao/Liability.sol';

contract Bytes32 is Message {
    bytes32 public data;
    function Bytes32(bytes32 _data) {
        data = _data;
    }
}

/**
 * @title Robot liability contract
 */
contract RobotLiability is Liability, ROSBridge, MessageHandler{
    /**
     * @dev Result subscriber contract
     */
    Subscriber resultSub;

    /**
     * @dev Create robot liability contract
     * @param _promisor Promisor (robot) account
     * @param _promisee Promisee account
     * @param _token Payment token contract
     * @param _cost Liability cost
     */
    function RobotLiability(address _promisor,
                            address _promisee,
                            address _token,
                            uint256 _cost)
            Liability(_promisor, _promisee, _token, _cost)
            ROSBridge(_promisee) {
        resultSub = mkSubscriber("result", "robot_liability/Bytes32", this);
    }

    /**
     * @dev Subscriber callback method
     */
    function incomingMessage(Message _msg) {
        // Check for message sender
        if (msg.sender != address(resultSub)) throw;

        // Handle result hash
        if (!resultHash(Bytes32(_msg).data())) throw;
    }
}
