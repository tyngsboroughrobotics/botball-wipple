#!/bin/bash

set -e

target="armv7-unknown-linux-gnueabihf"

cargo install cross

cross build --target=$target --release
cp target/$target/release/libplugin.so $target.wplplugin
