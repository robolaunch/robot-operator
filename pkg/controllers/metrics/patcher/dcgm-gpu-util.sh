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

if [[ -z "${DCGM_METRICS_ENDPOINT}" ]]; then
    echo "Environment DCGM_METRICS_ENDPOINT should be set.";
    exit 1;
fi

declare -A keys=(
    ["DCGM_FI_DEV_GPU_TEMP"]="temp"
    ["DCGM_FI_DEV_POWER_USAGE"]="powerUsage"
    ["DCGM_FI_DEV_GPU_UTIL"]="gpuUtil"
    ["DCGM_FI_DEV_MEM_COPY_UTIL"]="memoryUtil"
    ["DCGM_FI_DEV_FB_FREE"]="fbMemoryFree"
    ["DCGM_FI_DEV_FB_USED"]="fbMemoryUsed"
)

exists(){
  if [ "$2" != in ]; then
    return
  fi   
  eval '[ ${'$3'[$1]+ok} ]'  
}

while [ true ]
do
    DCGM_RESPONSE_RAW=$(curl $DCGM_METRICS_ENDPOINT);
    
    METRIC_KEY=""
    gpus_json_body=""
    metrics_json_body=""
    gpus=()
    while IFS= read -r line
    do
        METRIC_KEY=$(awk -F '{' '{print$1}' <<< $line)
        if [[ $METRIC_KEY == "# "* ]]; then
            continue
        fi

        if ! exists $METRIC_KEY in keys;
        then
            continue
        fi

        rest=$(awk -F '{' '{print$2}' <<< $line)
        json_data=$(awk -F '}' '{print$1}' <<< $rest)
        METRIC_VALUE=$(awk -F '}' '{print$2}' <<< $rest | xargs)
        TEMP_IFS=$IFS;
        IFS=",";
        INDEX=0;
        gpu="";
        deviceName="";
        UUID="";
        modelName="";
        for DATA in $json_data;
        do
            TEMP_IFS2=$IFS;
            IFS="=";
            kvdata=( $DATA );
            key=${kvdata[0]};
            value=${kvdata[1]};
            if [[ "$key" == "gpu" ]]; then
                gpu=$(echo $value | sed 's/"//g');
            fi
            if [[ "$key" == "device" ]]; then
                deviceName=$(echo $value | sed 's/"//g');
            fi
            if [[ "$key" == "UUID" ]]; then
                UUID=$(echo $value | sed 's/"//g');
            fi
            if [[ "$key" == "modelName" ]]; then
                modelName=$(echo $value | sed 's/"//g');
            fi
            IFS=$TEMP_IFS2;
            INDEX=$((INDEX+1));
        done

        if [[ ${gpus[@]} =~ $gpu ]]
        then
            continue;
        else
            gpus+=($gpu)
            if (( ${#gpus[@]} > 1 )); then
                gpus_json_body="$gpus_json_body,"
            fi
            gpus_json_body="$gpus_json_body \"$gpu\": {\"device\": \"$deviceName\", \"uuid\": \"$UUID\", \"model\": \"$modelName\"}"
        fi

        IFS=$TEMP_IFS;
    done <<< "$DCGM_RESPONSE_RAW" > ./metrics.local

    METRIC_KEY=""
    OLD_METRIC_KEY=""
    while IFS= read -r line
    do

        METRIC_KEY=$(awk -F '{' '{print$1}' <<< $line)
        if [[ $METRIC_KEY == "# "* ]]; then
            continue
        fi

        if ! exists $METRIC_KEY in keys;
        then
            continue
        fi
        
        readable_key="${keys[$METRIC_KEY]}"

        rest=$(awk -F '{' '{print$2}' <<< $line)
        json_data=$(awk -F '}' '{print$1}' <<< $rest)
        METRIC_VALUE=$(awk -F '}' '{print$2}' <<< $rest | xargs)
        TEMP_IFS=$IFS;
        IFS=",";
        INDEX=0;
        gpu="";

        for DATA in $json_data;
        do
            TEMP_IFS2=$IFS;
            IFS="=";
            kvdata=( $DATA );
            key=${kvdata[0]};
            value=${kvdata[1]};
            if [[ "$key" == "gpu" ]]; then
                gpu=$(echo $value | sed 's/"//g');
            fi
            IFS=$TEMP_IFS2;
            INDEX=$((INDEX+1));
        done

        if [[ "$OLD_METRIC_KEY" == "$METRIC_KEY" ]]; then
            metrics_json_body="$metrics_json_body, \"$gpu\": \"$METRIC_VALUE\""
        else
            if [[ "$OLD_METRIC_KEY" == "" ]]; then
                metrics_json_body="\"$readable_key\": {\"$gpu\": \"$METRIC_VALUE\""
            else
                metrics_json_body="$metrics_json_body}, \"$readable_key\": {\"$gpu\": \"$METRIC_VALUE\""
            fi
        fi

        OLD_METRIC_KEY=$METRIC_KEY;

        IFS=$TEMP_IFS;
    done <<< "$DCGM_RESPONSE_RAW" > ./metrics.local

    metrics_json_body="$metrics_json_body}"
  
    request_body="{\"devices\": {$gpus_json_body}, \"metrics\": {$metrics_json_body}}";
    echo "$request_body";

    sleep "$INTERVAL";
    
    kubectl patch metricsexporter $METRICS_EXPORTER_NAME \
        --type=merge \
        --subresource status \
        -n $METRICS_EXPORTER_NAMESPACE \
        --patch \
        "{\"status\": {\"usage\": {\"gpuDeviceStatuses\": $request_body}}}";
done
