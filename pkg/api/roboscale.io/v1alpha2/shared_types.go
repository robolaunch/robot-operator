package v1alpha2

import robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"

type ROS2BridgeInstanceStatus struct {
	// Generic status for any owned resource.
	Resource robotv1alpha1.OwnedResourceStatus `json:"resource,omitempty"`
	// Status of the ROS2Bridge instance.
	Status ROS2BridgeStatus `json:"status,omitempty"`
	// Address of the robot service that can be reached from outside.
	Connection string `json:"connection,omitempty"`
}
