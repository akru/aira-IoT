#!/usr/bin/env node

const parity = process.env.PARITY_NODE;
const config = require('/conf/market');

const market_abi = [{"constant":true,"inputs":[],"name":"name","outputs":[{"name":"","type":"string"}],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"asks","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_owner","type":"address"}],"name":"setOwner","outputs":[],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_beneficiary","type":"address"},{"name":"_promisee","type":"address"},{"name":"_price","type":"uint256"}],"name":"limitSell","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"asksLength","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"bids","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"hammer","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"bidsLength","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_id","type":"uint256"},{"name":"_beneficiary","type":"address"},{"name":"_promisee","type":"address"}],"name":"sellAt","outputs":[],"payable":false,"type":"function"},{"constant":false,"inputs":[],"name":"destroy","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[],"name":"owner","outputs":[{"name":"","type":"address"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_id","type":"uint256"},{"name":"_candidates","type":"uint256"}],"name":"sellConfirm","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"","type":"uint256"}],"name":"priceOf","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_price","type":"uint256"}],"name":"limitBuy","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"","type":"address"},{"name":"","type":"uint256"}],"name":"ordersOf","outputs":[{"name":"","type":"uint256"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_id","type":"uint256"}],"name":"buyAt","outputs":[],"payable":false,"type":"function"},{"constant":true,"inputs":[{"name":"_i","type":"uint256"}],"name":"getOrder","outputs":[{"name":"","type":"address[]"},{"name":"","type":"address[]"},{"name":"","type":"address"},{"name":"","type":"bool"}],"payable":false,"type":"function"},{"constant":false,"inputs":[{"name":"_hammer","type":"address"}],"name":"setHammer","outputs":[],"payable":false,"type":"function"},{"inputs":[{"name":"_name","type":"string"}],"payable":false,"type":"constructor"},{"payable":true,"type":"fallback"},{"anonymous":false,"inputs":[{"indexed":true,"name":"order","type":"uint256"}],"name":"OpenAskOrder","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"order","type":"uint256"}],"name":"OpenBidOrder","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"order","type":"uint256"}],"name":"CloseAskOrder","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"order","type":"uint256"}],"name":"CloseBidOrder","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"order","type":"uint256"},{"indexed":true,"name":"beneficiary","type":"address"},{"indexed":true,"name":"promisee","type":"address"}],"name":"AskOrderCandidates","type":"event"},{"anonymous":false,"inputs":[{"indexed":true,"name":"liability","type":"address"}],"name":"NewLiability","type":"event"}];

var Web3 = require('web3');
var web3 = new Web3(); 
web3.setProvider(new web3.providers.HttpProvider('http://'+parity+':8545'));
const me = web3.eth.accounts[0];

var m = web3.eth.contract(market_abi).at(config['market']);
console.log('Connected to Market: '+m.name());
console.log('My account: '+me);

m.OpenAskOrder({}, '', (e, r) => {
    if (!e) {
        const id = r.args.order;
        console.log('Opened ASK order with ID='+id);
        if (m.priceOf(id) >= parseInt(config['ask'])) {
            console.log('Accepted price '+m.priceOf(id));
            m.sellAt(id, config['beneficiary'], me, {from: me});
        }
    }
});

m.CloseBidOrder({}, '', (e, r) => {
    if (!e) {
        console.log('Closed BID order with ID='+r.args.order);
        placeOrder();
    }
});

placeOrder();

function placeOrder() {
    console.log("Try to place the order...");
    for (var i = 0; m.ordersOf(me, i) != 0; i += 1) {
        if (!m.getOrder(m.ordersOf(me, i))) {
            console.log("My open order found at "+i);
            return;
        }
    }
    m.limitSell(config['beneficiary'], me, parseInt(config['bid']), {from: me});
    console.log('Order placed');
}
