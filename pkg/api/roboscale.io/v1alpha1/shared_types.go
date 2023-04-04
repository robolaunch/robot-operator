package v1alpha1

import corev1 "k8s.io/api/core/v1"

type OwnedResourceStatus struct {
	Created   bool                   `json:"created"`
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	Phase     string                 `json:"phase,omitempty"`
}

type OwnedPodStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	IP       string              `json:"ip,omitempty"`
}

type DiscoveryServerInstanceStatus struct {
	Resource OwnedResourceStatus   `json:"resource,omitempty"`
	Status   DiscoveryServerStatus `json:"status,omitempty"`
}

type ROSBridgeInstanceStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	Status   ROSBridgeStatus     `json:"status,omitempty"`
}

type RobotDevSuiteInstanceStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	Status   RobotDevSuiteStatus `json:"status,omitempty"`
}

type WorkspaceManagerInstanceStatus struct {
	Resource OwnedResourceStatus    `json:"resource,omitempty"`
	Status   WorkspaceManagerStatus `json:"status,omitempty"`
}

type StepStatus struct {
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	Step     Step                `json:"step,omitempty"`
}
