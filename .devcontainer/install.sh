#!/usr/bin/env bash

deps=(
    bc
    bison
    flex
    gcc-arm-linux-gnueabi
    gcc-arm-linux-gnueabihf
    make
    ncurses-dev
    pkg-config
    python3
)

sudo apt-get update
sudo apt-get install -y --no-install-recommends "${deps[@]}"
