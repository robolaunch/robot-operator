/*
Copyright 2022.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

package v1alpha2

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"

	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
)

func init() {
	SchemeBuilder.Register(&ROS2Workload{}, &ROS2WorkloadList{})
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// ROS2Workload is the Schema for the ros2workloads API
type ROS2Workload struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`
	// Specification of the desired behavior of the ROS2Workload.
	Spec ROS2WorkloadSpec `json:"spec,omitempty"`
	// Most recently observed status of the ROS2Workload.
	Status ROS2WorkloadStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// ROS2WorkloadList contains a list of ROS2Workload
type ROS2WorkloadList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []ROS2Workload `json:"items"`
}

// ********************************
// ROS2Workload types
// ********************************

type LaunchContainer struct {
	// Replica number of the stateful set.
	Replicas *int32 `json:"replicas"`
	// Single container configuration for stateful set.
	Container corev1.Container `json:"container,omitempty"`
}

// ROS2WorkloadSpec defines the desired state of ROS2Workload.
type ROS2WorkloadSpec struct {
	// Discovery server configurational parameters.
	// +kubebuilder:validation:Required
	DiscoveryServerTemplate robotv1alpha1.DiscoveryServerSpec `json:"discoveryServerTemplate"`
	// ROS 2 Bridge configurational parameters.
	// +kubebuilder:validation:Required
	ROS2BridgeTemplate ROS2BridgeSpec `json:"ros2BridgeTemplate,omitempty"`
	// Volume templates for ROS 2 workload.
	// For each volume template, operator will create a PersistentVolumeClaim
	// that can be mounted to the ROS 2 workload.
	VolumeClaimTemplates []corev1.PersistentVolumeClaimTemplate `json:"volumeClaimTemplates,omitempty"`
	// Configurational parameters for containers that will be encapsulated within the ROS 2 workload StatefulSet.
	LaunchContainers []LaunchContainer `json:"launchContainers,omitempty"`
}

// ROS2WorkloadStatus defines the observed state of ROS2Workload.
type ROS2WorkloadStatus struct {
	// Phase of ROS2Workload. It sums the general status of ROS 2 workload(s).
	Phase ROS2WorkloadPhase `json:"phase,omitempty"`
	// Discovery server instance status.
	DiscoveryServerStatus robotv1alpha1.DiscoveryServerInstanceStatus `json:"discoveryServerStatus,omitempty"`
	// ROS 2 Bridge instance status.
	ROS2BridgeStatus ROS2BridgeInstanceStatus `json:"ros2BridgeStatus,omitempty"`
	// Statuses of owned PersistentVolumeClaims.
	PVCStatuses []OwnedPVCStatus `json:"pvcStatuses,omitempty"`
	// Status of owned StatefulSet and containers.
	StatefulSetStatuses []OwnedStatefulSetStatus `json:"statefulSetStatuses,omitempty"`
}
