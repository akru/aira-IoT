module.exports = {
    newContract: function(abi, bytecode, web3, args, fun) {
        var contract = web3.eth.contract(abi);
        var args = args.concat([
            { from: web3.eth.accounts[0],
              data: bytecode,
              gas: 3000000
            }, function(e, contract) {
                if (typeof contract != 'undefined' &&
                    typeof contract.address != 'undefined')
                    fun(null, contract.address);
                else
                    if (e) console.log("Error: " + e);
            }]);
        contract.new.apply(contract, args); 
    },
    getContract: function(abi, address, web3) {
        return web3.eth.contract(abi).at(address);
    }
}
