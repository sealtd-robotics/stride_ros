#!/bin/bash

echo "Compiling protobuf message for Python"
rm -f proto_src/*
protoc -I=. --python_out=proto_src ext-interface.proto

echo "Done compiling"

