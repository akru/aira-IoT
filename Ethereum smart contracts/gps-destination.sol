contract gpsdestination { 
    address dronAccount;
    address public currentCustomer;
    string public currentLongitude;
    string public currentLatitude;
    string public homebaseLongitude;
    string public homebaseLatitude;
    string public destinationLongitude;
    string public destinationLatitude;
    uint public estimatesActualBefore;

    /* Estimates data */

    mapping (address => uint) customerEstimatesOf;

    struct Estimate {
        address customerAddr;
        string destinationLongitude;
        string destinationLatitude;
        uint cost;
        uint actualBefore; 
    }

    Estimate[] public estimates; 

    /* Events */

    event NewGPSCoordinates(string destinationLongitude, string destinationLatitude);
    event DroneComeback(string currentLongitude, string currentLatitude);
    event NewEstimate(uint estimateID, string destinationLongitude, string destinationLatitude, string homebaseLongitude, string homebaseLatitude);

    /*Initial */
    function gpsdestination(string _homebaseLongitude, string _homebaseLatitude, uint _estimatesActualBefore) {
        dronAccount = msg.sender;
        currentLongitude = _homebaseLongitude;
        currentLatitude = _homebaseLatitude;
        estimatesActualBefore = _estimatesActualBefore * 1 minutes;
    }
    
    /* Drone functions */
    function homebase(string _homebaseLongitude, string _homebaseLatitude) returns(bool result) {
        if(msg.sender==dronAccount) {
        homebaseLongitude = _homebaseLongitude;
        homebaseLatitude = _homebaseLatitude;
        return true;}
    }



    function setEstimateCost(uint _estimateID, uint _cost) returns(bool result) {
        if(msg.sender==dronAccount) {
            Estimate e = estimates[_estimateID];
            e.cost = _cost;
            return true;
        }
    }


    /* Customer functions */

    function setNewEstimate(string _destinationLongitude, string _destinationLatitude) returns(uint estimateID) {
        estimateID = estimates.length++;
        Estimate e = estimates[estimateID];
        e.customerAddr = msg.sender;
        e.destinationLongitude = _destinationLongitude;
        e.destinationLatitude = _destinationLatitude;
        e.actualBefore = now + estimatesActualBefore;
        customerEstimatesOf[msg.sender] = estimateID;
        NewEstimate(estimateID, _destinationLongitude, _destinationLongitude, homebaseLongitude, homebaseLatitude);
        return estimateID;
    }

    function takeFlight() returns(bool result) {
        uint workEstimateID;
        workEstimateID = customerEstimatesOf[msg.sender];
        Estimate e = estimates[workEstimateID];
        if(msg.value>=e.cost) {
        currentCustomer = msg.sender;
        destinationLongitude = e.destinationLongitude;
        destinationLatitude = e.destinationLatitude;
        NewGPSCoordinates(destinationLongitude, destinationLatitude);
        return true;
        }
    }
}
