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
    PERCENTAGE=$(nvidia-smi | grep "%" | awk '{print $13}');
    sleep "$INTERVAL";
    kubectl patch metricsexporter $METRICS_EXPORTER_NAME \
        --type=merge \
        --subresource status \
        -n $METRICS_EXPORTER_NAMESPACE \
        --patch \
        "{\"status\": {\"usage\": {\"gpu\": {\"utilization\": \"$PERCENTAGE\"}}}}";
done
