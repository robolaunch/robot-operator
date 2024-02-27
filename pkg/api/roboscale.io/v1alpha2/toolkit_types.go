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
	robotv1alpha1 "github.com/robolaunch/robot-operator/pkg/api/roboscale.io/v1alpha1"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

func init() {
	SchemeBuilder.Register(&ROS2Bridge{}, &ROS2BridgeList{})
}

//+genclient
//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// ROS2Bridge is the Schema for the ros2bridges API
type ROS2Bridge struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   ROS2BridgeSpec   `json:"spec,omitempty"`
	Status ROS2BridgeStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// ROS2BridgeList contains a list of ROS2Bridge
type ROS2BridgeList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []ROS2Bridge `json:"items"`
}

// ********************************
// ROS2Bridge types
// ********************************

// ROS2BridgeSpec defines the desired state of ROS2Bridge.
type ROS2BridgeSpec struct {
	// Configurational parameters for ROS 2 bridge.
	// +kubebuilder:validation:Required
	Distro robotv1alpha1.ROSDistro `json:"distro"`
	// Object reference to DiscoveryServer.
	DiscoveryServerReference corev1.ObjectReference `json:"discoveryServerRef,omitempty"`
	// Service type of ROS2Bridge. `ClusterIP` and `NodePort` is supported.
	// +kubebuilder:validation:Enum=ClusterIP;NodePort
	ServiceType corev1.ServiceType `json:"serviceType,omitempty"`
	// ROS2Bridge will create an Ingress resource if `true`.
	Ingress bool `json:"ingress,omitempty"`
}

// ROS2BridgeStatus defines the observed state of ROS2Bridge.
type ROS2BridgeStatus struct {
	// Phase of ROS2Bridge.
	Phase ROS2BridgePhase `json:"phase,omitempty"`
	// Object reference to DiscoveryServer.
	DiscoveryServerReference corev1.ObjectReference `json:"discoveryServerRef,omitempty"`
	// Status of ROS2Bridge pod.
	PodStatus robotv1alpha1.OwnedResourceStatus `json:"podStatus,omitempty"`
	// Status of ROS2Bridge service.
	ServiceStatus robotv1alpha1.OwnedServiceStatus `json:"serviceStatus,omitempty"`
	// Status of ROS2Bridge Ingress.
	IngressStatus robotv1alpha1.OwnedResourceStatus `json:"ingressStatus,omitempty"`
}
