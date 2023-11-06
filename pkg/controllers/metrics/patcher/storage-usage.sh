#!/bin/bash

set eux;

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
    RESPONSE=$(cat /etc/fstab);
    IFS=$'\n';
    INDEX=0;
    LINES=( $RESPONSE );
    FS_ARR=()
    for LINE in "${LINES[@]}";
    do
        IFS=" ";
        LINE_ARR=( $LINE );
        
        # getting fs fields, calculating percentage
        FS_UUID=$(echo ${LINE_ARR[0]} | xargs)
        FS_MOUNTED_ON=$(echo ${LINE_ARR[1]} | xargs)
        if [[ "$FS_UUID" == "#" ]]; then
            continue;
        elif [[ "$FS_MOUNTED_ON" == "none" ]]; then
            continue;
        elif [[ "$FS_MOUNTED_ON" == "" ]]; then
            continue;
        fi

        FS_ARR+=($FS_MOUNTED_ON)
    done

    RESPONSE=$(df);
    IFS=$'\n';
    INDEX=0;
    LINES=( $RESPONSE );
    LINES=( "${LINES[@]:1}" );
    COLLECTIVE_LOAD=""

    for LINE in "${LINES[@]}";
    do
        IFS=" ";
        LINE_ARR=( $LINE );
        
        # getting fs fields, calculating percentage
        FS_NAME=$(echo ${LINE_ARR[0]} | xargs)
        FS_SIZE=$(echo ${LINE_ARR[1]} | xargs)
        FS_USED=$(echo ${LINE_ARR[2]} | xargs)
        FS_MOUNTED_ON=$(echo ${LINE_ARR[5]} | xargs)
        
        if [[ ! ${FS_ARR[@]} =~ $FS_MOUNTED_ON ]]
        then
            continue;
        fi

        FS_PERCENTAGE="$(((FS_USED * 100 + FS_SIZE - 1 ) / FS_SIZE))"
        
        if [[ $INDEX -ne 0 ]]; then
            COLLECTIVE_LOAD="$COLLECTIVE_LOAD, "
        fi
        COLLECTIVE_LOAD="$COLLECTIVE_LOAD\
            \"$FS_NAME\":\
                {\
                    \"size\": \"$FS_SIZE\",
                    \"used\": \"$FS_USED\",
                    \"percentage\": \"$FS_PERCENTAGE\",
                    \"mountedOn\": \"$FS_MOUNTED_ON\"
                }"


        INDEX=$((INDEX+1))
    done

    IFS=" ";
    COLLECTIVE_LOAD="{$COLLECTIVE_LOAD}"
    sleep "$INTERVAL";
    PATCH_BODY="{\"status\": {\"usage\": {\"storage\": {\"usage\": $COLLECTIVE_LOAD}}}}";

    kubectl patch metricsexporter $METRICS_EXPORTER_NAME \
        --type=merge \
        --subresource status \
        -n $METRICS_EXPORTER_NAMESPACE \
        --patch \
        "$PATCH_BODY";
done
