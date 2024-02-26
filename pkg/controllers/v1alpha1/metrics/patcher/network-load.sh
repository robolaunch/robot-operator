#!/bin/bash

set eux;

trap ctrl_c INT;

function ctrl_c() {
    echo "Ending collecting network load data...";
    pkill -P $$;
    exit 0;
}

if [[ -z "${METRICS_EXPORTER_NAME}" ]]; then
    echo "Environment METRICS_EXPORTER_NAME should be set.";
    exit 1;
fi

if [[ -z "${METRICS_EXPORTER_NAMESPACE}" ]]; then
    echo "Environment METRICS_EXPORTER_NAMESPACE should be set.";
    exit 1;
fi

if [[ -z "${INTERVAL}" ]]; then
    echo "Environment INTERVAL should be set.";
    exit 1;
fi

if [[ -z "${NETWORK_INTERFACES}" ]]; then
    echo "Environment NETWORK_INTERFACES should be set.";
    exit 1;
fi

ifstat -nb -i "$NETWORK_INTERFACES" $INTERVAL > network-load.txt &
sleep $INTERVAL;

while [ true ]
do
    COLLECTIVE_LOAD="";
    INDEX=0;
    IFS=,;
    for INTERFACE in $NETWORK_INTERFACES;
    do
        if [[ $INDEX -ne 0 ]]; then
            COLLECTIVE_LOAD="$COLLECTIVE_LOAD, "
        fi
        COLUMN_NUMBER_IN=$((INDEX*2+1))
        COLUMN_NUMBER_OUT=$((COLUMN_NUMBER_IN+1))
        INTERFACE_IN=$(cat network-load.txt | tail -1 | awk "{print \$$COLUMN_NUMBER_IN}")
        INTERFACE_OUT=$(cat network-load.txt | tail -1 | awk "{print \$$COLUMN_NUMBER_OUT}")
        COLLECTIVE_LOAD="$COLLECTIVE_LOAD\"$INTERFACE\": {\"in\": \"$INTERFACE_IN Kbps\", \"out\": \"$INTERFACE_OUT Kbps\"}"
        INDEX=$((INDEX+1))
    done
    IFS=" ";
    COLLECTIVE_LOAD="{$COLLECTIVE_LOAD}";
    kubectl patch metricsexporter $METRICS_EXPORTER_NAME \
        --type=merge \
        --subresource status \
        -n $METRICS_EXPORTER_NAMESPACE \
        --patch \
        "{\"status\": {\"usage\": {\"network\": {\"load\": $COLLECTIVE_LOAD}}}}"
    sleep $INTERVAL;
done
