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
  contract.EstimateCostReceive({}, '', function(err, r){
	if (!err) {
      mission_id = r.args.estimateID;
      console.log('\n'
		        + 'DAO "Smartcontractbase.com" services event:\n'
		        + 'Data for estimate #'+mission_id+' from drone received!\n'
				+ 'Distance: ' + r.args.distance + ' m\n'
				+ 'Cost: ' + r.args.cost + ' Ether\n'
      );
    }
  }),
  contract.DroneComeback({}, '', function(err, r){
	if(!err)
	  console.log('\n'
		        + 'DAO "Smartcontractbase.com" services event:\n'
                + 'Mission #' + mission_id + ' successful complete! Drone back to homebase.\n'
				+ 'Thank you for using our services.\n'
				+ 'If you have any questions please contact us on: support@smartcontractbase.com\n'
				+ 'email: support@drone-employee.com\n'
      );
  })
];

function client_info() {
    console.log('User address om Ethereum network: ' + eth.accounts[0]);
    console.log('Ether amount: ' + web3.fromWei(eth.getBalance(eth.accounts[0]), "ether"));
}
