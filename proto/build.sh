#!/bin/bash
protoc -I=./ -I=./mirabuf/ --cpp_out=../BulletSynthesis/Proto/ ./*.proto

