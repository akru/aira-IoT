{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE QuasiQuotes #-}
module Main where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3

import Control.Concurrent.Chan (Chan, newChan, readChan, writeChan)
import Control.Monad.IO.Class (liftIO)
import System.Environment (getEnv)
import Data.String (fromString)

[abiFrom|abi/Market.json|]

enableLeft  = putStrLn "Enable LEFT railway!"
enableRigth = putStrLn "Enable RIGHT railway!"

data Parity

instance Provider Parity where
    rpcUri = return "http://parity:8545"

marketMonitor :: Address
              -> Address
              -> Web3 Parity (Chan (Either Integer Integer))
marketMonitor ml mr = do
    c <- liftIO newChan

    event ml $ \(OrderClosed o) -> do
        price <- priceOf ml o
        liftIO $ writeChan c (Left price)
        return ContinueEvent

    event mr $ \(OrderClosed o) -> do
        price <- priceOf mr o
        liftIO $ writeChan c (Right price)
        return ContinueEvent

    return c

controller :: Chan (Either Integer Integer)
           -> Integer
           -> IO ()
controller c best = readChan c >>= switch >>= controller c
  where switch (Right price)
            | price > best = enableRigth >> return price
            | otherwise    = return best

        switch (Left price)
            | price > best = enableLeft >> return price
            | otherwise    = return best

main :: IO ()
main = do
    ml <- fromString <$> getEnv "MARKET_LEFT_ADDRESS"
    mr <- fromString <$> getEnv "MARKET_RIGHT_ADDRESS"
    monitor <- runWeb3' (marketMonitor ml mr)
    case monitor of
        Left e -> do
            putStrLn "Unable to start market monitor:"
            print e
        Right m -> controller m 0
