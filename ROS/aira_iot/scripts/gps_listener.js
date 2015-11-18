var r = loadScript('gps_contract.js');
var e = gps.NewGPSCoordinates({}, '', function(e, r){
    if (!e)
        console.error('GPS ' + r.args.longitude_dest + ' ' + r.args.latitude_dest);
});
