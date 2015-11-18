var abidef = [{"constant":true,"inputs":[],"name":"latitude","outputs":[{"name":"","type":"string"}],"type":"function"},{"constant":true,"inputs":[],"name":"longitude","outputs":[{"name":"","type":"string"}],"type":"function"},{"constant":false,"inputs":[{"name":"_longitude_dest","type":"string"},{"name":"_latitude_dest","type":"string"}],"name":"newGPSCoordinates","outputs":[{"name":"result","type":"bool"}],"type":"function"},{"inputs":[],"type":"constructor"},{"anonymous":false,"inputs":[{"indexed":false,"name":"longitude_dest","type":"string"},{"indexed":false,"name":"latitude_dest","type":"string"}],"name":"NewGPSCoordinates","type":"event"}];

var gps = eth.contract(abidef).at("0x741f57c903222577cd3d4fee171526303a7bf016");

function go(lat, lon) {
    gps.newGPSCoordinates.sendTransaction(lat, lon, 
            {from:eth.accounts[0], to: '0xcc0adde68413aae6fc07b403d93cffb2f88e6dfb', gas: 1000000});
}
