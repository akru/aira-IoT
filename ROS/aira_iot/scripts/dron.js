//loadScript('gps_contract.js');
var mission_id = "";
var e = [
  contract.NewGPSCoordinates({}, '', function(err, r){
    if (!err)
      console.log('\n'
		        + 'Payment receive. Take new mission #'+mission_id+': \n'
                + 'Destination longitude: ' + r.args.destinationLongitude + '\n'
                + 'Destination latitude: ' + r.args.destinationLatitude + '\n'
      );
  }),
  contract.NewEstimate({}, '', function(err, r){
	if (!err) {
      mission_id = r.args.estimateID;
      console.log('\n'
		        + 'New estimate #'+mission_id+' for calculate. \n'
                + 'Destination longitude: ' + r.args.destinationLongitude + '\n'
                + 'Destination latitude: ' + r.args.destinationLatitude + '\n');
    }
  })
];
