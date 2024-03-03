package v1alpha2

import (
	corev1 "k8s.io/api/core/v1"
)

// Generic status for any owned resource.
type OwnedResourceStatus struct {
	// Shows if the owned resource is created.
	Created bool `json:"created"`
	// Reference to the owned resource.
	Reference corev1.ObjectReference `json:"reference,omitempty"`
	// Phase of the owned resource.
	Phase string `json:"phase,omitempty"`
}

type ROS2BridgeInstanceStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the ROS2Bridge instance.
	Status ROS2BridgeStatus `json:"status,omitempty"`
	// Address of the robot service that can be reached from outside.
	Connection string `json:"connection,omitempty"`
}

type OwnedPVCStatus struct {
	// Generic status for any owned resource.
	Resource OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the ROS2Bridge instance.
	Status corev1.PersistentVolumeClaimStatus `json:"status,omitempty"`
}
