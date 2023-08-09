# Useful Commands for Development Environment

Create directories for development manifests:

```bash
mkdir -p hack/deploy.local/manifests
mkdir -p hack/deploy.local/chart
```

Building operator with single architecture (architecture is chosen according to your system):

```bash
make docker-build docker-push helm IMG=robolaunchio/robot-controller-manager-dev:<VERSION> RELEASE=<VERSION>
```

Building operator with multiple architecture (for `amd64` and `arm64`):

```bash
make docker-buildx helm IMG=robolaunchio/robot-controller-manager-dev:<VERSION> RELEASE=<VERSION>
```