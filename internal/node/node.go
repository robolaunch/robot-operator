package node

import (
	corev1 "k8s.io/api/core/v1"
)

func HasGPU(node corev1.Node) bool {
	_, ok := node.Status.Allocatable["nvidia.com/gpu"]
	return ok
}
