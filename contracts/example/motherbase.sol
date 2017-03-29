pragma solidity ^0.4.4;

contract motherbase {
    address creator;
    int256  public homebaseLongitude;
    int256  public homebaseLatitude;
    mapping (address => bool) droneActiveOf;
    address public airwayAdminAddr;


    modifier creatorCheck { if (msg.sender == creator) _; }

    /* Functions */
    function motherbase(int256 _homebaseLongitude,
                        int256 _homebaseLatitude,
                        address _airwayAdminAddr) {
        creator = msg.sender;
        homebaseLatitude = _homebaseLatitude;
        homebaseLongitude = _homebaseLongitude;
        airwayAdminAddr = _airwayAdminAddr;
    }

    function setDrone(address _droneAddr) creatorCheck returns(bool result) {
        droneActiveOf[_droneAddr] = true;
        result = true;
    }

    function inactiveDrone(address _droneAddr) creatorCheck returns(bool result) {
        droneActiveOf[_droneAddr] = false;
        result = true;
    }
}
