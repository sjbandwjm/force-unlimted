
```
pip install grpcio-tools
pip install mypy-protobuf

mkdir -p generate
python -m grpc_tools.protoc     -I.  --mypy_out=generate/   --python_out=generate/      */*.proto

# isacclab
rm -rf generate
mkdir -p generate

python -m grpc_tools.protoc \
    -I. \
    --python_out=generate \
    --grpc_python_out=generate \
    */*.proto
```