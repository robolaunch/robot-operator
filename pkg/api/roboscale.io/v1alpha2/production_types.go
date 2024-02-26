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

	Spec   ROS2WorkloadSpec   `json:"spec,omitempty"`
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

// ROS2WorkloadSpec defines the desired state of ROS2Workload.
type ROS2WorkloadSpec struct {
	// Discovery server configurational parameters.
	DiscoveryServerTemplate robotv1alpha1.DiscoveryServerSpec `json:"discoveryServerTemplate,omitempty"`
}

// ROS2WorkloadStatus defines the observed state of ROS2Workload.
type ROS2WorkloadStatus struct {
	// Phase of ROS2Workload. It sums the general status of ROS 2 workload(s).
	Phase ROS2WorkloadPhase `json:"phase,omitempty"`
	// Discovery server instance status.
	DiscoveryServerStatus robotv1alpha1.DiscoveryServerInstanceStatus `json:"discoveryServerStatus,omitempty"`
}
