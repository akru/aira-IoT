contract gpsdestination { 
    string public longitude;
	string public latitude;

    event NewGPSCoordinates(string longitude_dest, string latitude_dest);

    /*Initial */
    function gpsdestination() {
    }

    /* Founder functions */
    function newGPSCoordinates(string _longitude_dest, string _latitude_dest) returns(bool result) {
		longitude = _longitude_dest;
		latitude = _latitude_dest;
		NewGPSCoordinates(longitude, latitude);
		return true;
    }
}