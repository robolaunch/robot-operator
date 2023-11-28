#
#   Image: robolaunchio/custom-metrics-patcher:focal-v1.24.10
#

FROM ubuntu:focal

SHELL [ "/bin/bash", "-c" ]

RUN set eux; \
    apt-get update; \
    apt-get install -y ifstat curl

RUN set eux; \
    curl -LO https://dl.k8s.io/release/v1.24.10/bin/linux/amd64/kubectl; \
    install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl

COPY . .

RUN set eux; \
    chmod +x dcgm-gpu-util.sh network-load.sh storage-usage.sh

CMD [ "./network-load.sh" ]