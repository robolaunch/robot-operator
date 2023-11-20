#!/bin/bash

set eux;

if ! command -v nvidia-smi &> /dev/null
then
    echo "nvidia-smi command not found!";
    exit 1;
fi

if [[ -z "${INTERVAL}" ]]; then
    echo "Environment INTERVAL should be set.";
    exit 1;
fi

if [[ -z "${METRICS_EXPORTER_NAME}" ]]; then
    echo "Environment METRICS_EXPORTER_NAME should be set.";
    exit 1;
fi

if [[ -z "${METRICS_EXPORTER_NAMESPACE}" ]]; then
    echo "Environment METRICS_EXPORTER_NAMESPACE should be set.";
    exit 1;
fi

while [ true ]
do
    MODEL=$(nvidia-smi --query-gpu=gpu_name --format=csv | sed -n 2p);
    sleep "$INTERVAL";
    kubectl patch metricsexporter $METRICS_EXPORTER_NAME \
        --type=merge \
        --subresource status \
        -n $METRICS_EXPORTER_NAMESPACE \
        --patch \
        "{\"status\": {\"usage\": {\"gpuModel\": \"$MODEL\"}}}";
done
