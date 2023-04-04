package v1alpha1

import corev1 "k8s.io/api/core/v1"

type OwnedResourceStatus struct {
	Created   bool                   `json:"created,omitempty"`
	Reference corev1.ObjectReference `json:"reference,omitempty"`
}

type OwnedPodStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	Phase    corev1.PodPhase     `json:"phase,omitempty"`
}

type DetailedOwnedPodStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	Phase    corev1.PodPhase     `json:"phase,omitempty"`
	IP       string              `json:"ip,omitempty"`
}

type OwnedJobStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	Phase    JobPhase            `json:"phase,omitempty"`
}
