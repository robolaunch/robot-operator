#!/bin/bash

/home/tuna/Desktop/playground/crd-ref-docs/crd-ref-docs \
    --source-path=./api/roboscale.io/v1alpha1 \
    --renderer=markdown \
    --max-depth=10 \
    --output-path=./docs/reference/reference.md \
    --config=./hack/dev/reference/ref_gen.yaml