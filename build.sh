#!/bin/bash

set -e

target="armv7-unknown-linux-gnueabihf"

rustup target add $target

cargo build --target=$target --release
cp target/$target/release/libplugin.so $target.wplplugin
