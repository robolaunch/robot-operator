package v1alpha1

import corev1 "k8s.io/api/core/v1"

type OwnedPodStatus struct {
	Created bool            `json:"created,omitempty"`
	Phase   corev1.PodPhase `json:"phase,omitempty"`
}

type DetailedOwnedPodStatus struct {
	Created bool            `json:"created,omitempty"`
	Phase   corev1.PodPhase `json:"phase,omitempty"`
	IP      string          `json:"ip,omitempty"`
}

type OwnedJobStatus struct {
	Created bool     `json:"created,omitempty"`
	Phase   JobPhase `json:"phase,omitempty"`
}
