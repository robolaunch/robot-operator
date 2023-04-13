package v1alpha1

import corev1 "k8s.io/api/core/v1"

// Generic status for any owned resource.
type OwnedResourceStatus struct {
	// Shows if the owned resource is created.
	Created bool `json:"created"`
	// Reference to the owned resource.
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	// Phase of the owned resource.
	Phase string `json:"phase,omitempty"`
}

type OwnedPodStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// IP of the pod.
	IP string `json:"ip,omitempty"`
}

type DiscoveryServerInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the DiscoveryServer instance.
	Status DiscoveryServerStatus `json:"status,omitempty"`
}

type ROSBridgeInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the ROSBridge instance.
	Status ROSBridgeStatus `json:"status,omitempty"`
}

type RobotDevSuiteInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the RobotDevSuite instance.
	Status RobotDevSuiteStatus `json:"status,omitempty"`
}

type WorkspaceManagerInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the WorkspaceManager instance.
	Status WorkspaceManagerStatus `json:"status,omitempty"`
}

type StepStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the step.
	Step Step `json:"step,omitempty"`
}
