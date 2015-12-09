loadScript('gps_contract_abi.js');
var contract = eth.contract(gps_contract_abi).at("0xf5331f5af75dd361a870d4dd57772e4bad7dcda0");
var e = [
  contract.NewGPSCoordinates({}, '', function(err, r){
    if (!err)
      console.log('GPS ' + r.args.destinationLongitude + ' ' + r.args.destinationLatitude);
  }),
  contract.NewEstimate({}, '', function(err, r){
    if (!err)
      console.log('EST ' + r.args.estimateID.toString() +
                     ' ' + r.args.homebaseLongitude +
                     ' ' + r.args.homebaseLatitude +
                     ' ' + r.args.destinationLongitude +
                     ' ' + r.args.destinationLatitude);
  })];
