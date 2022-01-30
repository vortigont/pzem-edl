#!/bin/sh

dst='./lib'

echo "Downloading libs"
mkdir -p $dst
git clone --depth 1 https://github.com/vortigont/pzem-edl $dst/pz
mv $dst/pz/src $dst/pzem-edl
rm -rf $dst/pz
#git clone --depth 1 -b vortigont https://github.com/vortigont/LinkedList $dst/LinkedList
git clone --depth 1 https://github.com/vortigont/LinkedList $dst/LinkedList
