#!/usr/bin/env python3

from flask import Flask, Response, request, redirect
from flask_restful import Resource, Api
from ipfsapi import connect
from os import environ
import requests

app = Flask(__name__)
api = Api(app)

@app.route('/')
def index():
    return redirect('/ipns/{0}'.format(environ['DAPP_IPNS']))

@app.route('/ipns/<path:url>')
def dapp(url):
    dapp_url = 'http://{0}/ipns/{1}'.format(environ['IPFS_NODE'], url)
    r = requests.get(dapp_url, stream=True, params=request.args)
    def generate():
        for chunk in r.iter_content(1024):
            yield chunk
    return Response(generate(), headers=r.raw.headers.items())

class Descriptor(Resource):
    def get(self):
        ipfs = connect(environ['IPFS_NODE'], 5001)

        desc = {}
        desc['IPNS'] = ipfs.id()['ID']
        desc['address'] = open('/chain/address.txt', 'r').readline()[:-1]

        return ipfs.add_json(desc)

api.add_resource(Descriptor, '/api/v1/descriptor')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=80)
