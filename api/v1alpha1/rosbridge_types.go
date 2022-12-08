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

package v1alpha1

import (
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

type BridgeDistro struct {
	Enabled bool      `json:"enabled,omitempty"`
	Distro  ROSDistro `json:"distro,omitempty"`
}

// ROSBridgeSpec defines the desired state of ROSBridge
type ROSBridgeSpec struct {
	ROS   BridgeDistro `json:"ros,omitempty"`
	ROS2  BridgeDistro `json:"ros2,omitempty"`
	Image string       `json:"image,omitempty"`
}

type BridgePodStatus struct {
	Created bool            `json:"created,omitempty"`
	Phase   corev1.PodPhase `json:"phase,omitempty"`
}

type BridgeServiceStatus struct {
	Created bool `json:"created,omitempty"`
}

type BridgePhase string

const (
	BridgePhaseCreatingService BridgePhase = "CreatingService"
	BridgePhaseCreatingPod     BridgePhase = "CreatingPod"
	BridgePhaseReady           BridgePhase = "Ready"
)

// ROSBridgeStatus defines the observed state of ROSBridge
type ROSBridgeStatus struct {
	Phase         BridgePhase         `json:"phase,omitempty"`
	PodStatus     BridgePodStatus     `json:"podStatus,omitempty"`
	ServiceStatus BridgeServiceStatus `json:"serviceStatus,omitempty"`
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status

// ROSBridge is the Schema for the rosbridges API
type ROSBridge struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   ROSBridgeSpec   `json:"spec,omitempty"`
	Status ROSBridgeStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// ROSBridgeList contains a list of ROSBridge
type ROSBridgeList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []ROSBridge `json:"items"`
}

func init() {
	SchemeBuilder.Register(&ROSBridge{}, &ROSBridgeList{})
}

func (rosbridge *ROSBridge) GetBridgePodMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      rosbridge.Name,
		Namespace: rosbridge.Namespace,
	}
}

func (rosbridge *ROSBridge) GetBridgeServiceMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      rosbridge.Name,
		Namespace: rosbridge.Namespace,
	}
}

func (rosbridge *ROSBridge) GetOwnerMetadata() *types.NamespacedName {
	return &types.NamespacedName{
		Name:      rosbridge.OwnerReferences[0].Name,
		Namespace: rosbridge.Namespace,
	}
}
