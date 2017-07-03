{-# LANGUAGE OverloadedStrings #-}
{-# LANGUAGE QuasiQuotes #-}
module Main where

import Network.Ethereum.Web3.TH
import Network.Ethereum.Web3

import Control.Concurrent.Chan (Chan, newChan, readChan, writeChan)
import qualified System.Hardware.Z21 as Z
import Control.Monad.IO.Class (liftIO)
import System.Environment (getEnv)
import Data.String (fromString)

[abiFrom|abi/Market.json|]

enableLeft :: IO ()
enableLeft = do
    putStrLn "Enable LEFT railway!"
    Z.runZ21 "192.168.0.111" 21105 $ Z.setLocoDrive (Z.Address 0 3) 0x10 0x06

enableRigth :: IO ()
enableRigth = do
    putStrLn "Enable RIGHT railway!"
    Z.runZ21 "192.168.0.111" 21105 $ Z.setLocoDrive (Z.Address 0 3) 0x10 0xF6

data Parity

instance Provider Parity where
    rpcUri = return "http://localhost:8545"

marketMonitor :: Address
              -> Address
              -> Web3 Parity (Chan (Either Integer Integer))
marketMonitor ml mr = do
    c <- liftIO newChan

    liftIO (putStrLn "Connected to:")
    liftIO . (putStr "Market A - " >>) . print =<< name ml
    liftIO . (putStr "Market B - " >>) . print =<< name mr

    event ml $ \(OrderClosed o) -> do
        price <- priceOf ml o
        liftIO $ do
            putStrLn $ "Order on market A closed, price = " ++ show price
            writeChan c (Left price)
        return ContinueEvent

    event mr $ \(OrderClosed o) -> do
        price <- priceOf mr o
        liftIO $ do
            putStrLn $ "Order on market B closed, price = " ++ show price
            writeChan c (Right price)
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
