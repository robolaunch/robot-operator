package node

import (
	corev1 "k8s.io/api/core/v1"
)

func HasGPU(node corev1.Node) bool {
	_, ok := node.Status.Allocatable["nvidia.com/gpu"]
	return ok
}

func IsK3s(node corev1.Node) bool {
	if val, ok := node.Labels["node.kubernetes.io/instance-type"]; ok && val == "k3s" {
		return true
	}
	return false
}
