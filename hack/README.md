# Useful Commands for Development Environment

Create directories for development manifests:

```bash
mkdir -p hack/deploy.local/manifests
mkdir -p hack/deploy.local/chart
```

Building operator with single architecture (architecture is chosen according to your system):

```bash
make docker-build docker-push extract helm IMG=robolaunchio/robot-controller-manager-dev:<VERSION> RELEASE=<VERSION>
```

Building operator with multiple architecture (for `amd64` and `arm64`):

```bash
make docker-buildx extract helm IMG=robolaunchio/robot-controller-manager-dev:<VERSION> RELEASE=<VERSION>
```

Fetching logs:

```bash
kubectl logs --follow pod/$(kubectl get pods -n robot-system | tail -n1 | awk '{print $1}') -n robot-system -c manager
```