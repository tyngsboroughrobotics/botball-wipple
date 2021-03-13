#!/bin/bash

set -e

host="$(rustc --version --verbose | grep host: | cut -c 7-)"

cargo build --release

for f in $(find target/release -maxdepth 1 -name libplugin.dylib -o -name libplugin.so); do
    cp $f "$host.wplplugin"
done
