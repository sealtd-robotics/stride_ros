README
---
This package uses `protobuf` to serialize data and communicate through zeroMQ.

### Install Protobuf
For developer if need to rebuild protobuf message

__On Ubuntu x64__
```bash
PROTOC_ZIP=protoc-3.17.3-linux-x86_64.zip
curl -OL https://github.com/protocolbuffers/protobuf/releases/download/v3.17.3/$PROTOC_ZIP
sudo unzip -o $PROTOC_ZIP -d /usr/local bin/protoc
sudo unzip -o $PROTOC_ZIP -d /usr/local 'include/*'
rm -f $PROTOC_ZIP
```
__On Ubuntu aarch 64 (arm)__
```bash
PROTOC_ZIP=protoc-3.17.3-linux-aarch_64.zip
curl -OL https://github.com/protocolbuffers/protobuf/releases/download/v3.17.3/$PROTOC_ZIP
sudo unzip -o $PROTOC_ZIP -d /usr/local bin/protoc
sudo unzip -o $PROTOC_ZIP -d /usr/local 'include/*'
rm -f $PROTOC_ZIP
```

Build Python Protobuf

```bash
protoc -I=. --python_out=proto_src ext-interface.proto
```