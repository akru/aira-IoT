FROM node

RUN npm install -g web3
ADD ./market.js /usr/local/bin/market.js

ENTRYPOINT ["market.js"]
