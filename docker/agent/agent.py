#!/usr/bin/env python3

from ipfsapi import connect
from flask import Flask
import os

app = Flask(__name__)

@app.route('/')
def index():
    return gen_descriptor()

def gen_descriptor():
    ipfs = connect(os.environ['IPFS_NODE'], 5001)

    desc = {}
    desc['IPNS'] = ipfs.id()['ID']
    desc['address'] = open('/chain/address.txt', 'r').readline()[:-1]

    return ipfs.add_json(desc) 

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=int(os.environ['PORT']))
