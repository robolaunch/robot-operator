package v1alpha1

import corev1 "k8s.io/api/core/v1"

type OwnedResourceStatus struct {
	Created   bool                   `json:"created,omitempty"`
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	Phase     string                 `json:"phase,omitempty"`
}

type DetailedOwnedPodStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	IP       string              `json:"ip,omitempty"`
}
