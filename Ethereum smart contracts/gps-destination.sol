contract gpsdestination { 
    address dronAccount;
    address public currentCustomer;
    string public currentLongitude;
    string public currentLatitude;
    string public destinationLongitude;
    string public destinationLatitude;


    event NewGPSCoordinates(string destinationLongitude, string destinationLatitude);
    event droneComeback(string currentLongitude, string currentLatitude);

    /*Initial */
    function gpsdestination(string _currentLongitude, string _currentLatitude) {
        dronAccount = msg.sender;
        currentLongitude = _currentLongitude;
        currentLatitude = _currentLatitude;
    }
    
    /* Drone functions */
        function homebase(string _currentLongitude, string _currentLatitude) returns(bool result) {
        if(msg.sender==dronAccount) {
        currentLongitude = _currentLongitude;
        currentLatitude = _currentLatitude;
        return true;}
    }


    /* Customer functions */

    function getFlightCost(string _destinationLongitude, string _destinationLatitude) returns(uint flightCost) {
        flightCost = 5;
        return flightCost;
    }

    function takeFlight(string _destinationLongitude, string _destinationLatitude) returns(bool result) {
        if(msg.value>=this.getFlightCost(_destinationLongitude, _destinationLatitude)) {
        currentCustomer = msg.sender;
        destinationLongitude = _destinationLongitude;
        destinationLatitude = _destinationLatitude;
        NewGPSCoordinates(destinationLongitude, destinationLatitude);
        return true;
        }
    }
}
