#!/bin/bash

set -e

host="$(rustc --version --verbose | grep host: | cut -c 7-)"

cargo build --release
cp target/release/libplugin.{dylib,so} "$host.wplplugin"
